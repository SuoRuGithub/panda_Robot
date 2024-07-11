#coding=gbk
import pybullet as p
import pybullet_data
import time 
import math 
import numpy as np

# 机械臂的初始configuration，别问我怎么来的……
jointPositions=[0.5, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
# 物体的初始位置
obj_start_coordinate = np.array([0.5, 0.5, 0.])
# 每一步仿真之间的间隔
TIMESTEP = 0.01

# 整个工作流程是，先从初始位置拿起放在地面上的物体，然后以圆形轨迹运动。
# 0：初始状态，钳子打开 ->
# 1: 位于物体正上方，钳子朝正下方->
# 2: 抵达物体位置，准备抓取->
# 3: 成功抓取（钳子闭合）->
# 4: 按照给定的轨迹运动.

class myPanda():
    def __init__(self, bullet_client, offset):
        self.bullet_client = bullet_client
        self.bullet_client.loadURDF("plane.urdf")
        
        self.obj = self.bullet_client.loadURDF("cube_small.urdf", obj_start_coordinate)

        self.pandaID = self.bullet_client.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase = True)
        self.numJoints = self.bullet_client.getNumJoints(self.pandaID)
        self.endEffectorIndex = 11

        self.t = 0      # 这个参数是用在最后按照轨迹运动的地方的。
        
        self.target_position = self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[-2]
        self.firger_target = 0.04
        
        # self.finger_state = 0.04    # 初始手指张开
        # 初始化机械臂的configuration，是为state0
        self.state = 0
        for i in range(9):
            self.bullet_client.resetJointState(self.pandaID, i, jointPositions[i])
        # 初始化手指，让手指张开
        for i in [9, 10]:
            self.bullet_client.setJointMotorControl2(self.pandaID, i, self.bullet_client.POSITION_CONTROL, self.firger_target, force = 10)
        # for test
        # 获取末端执行器的坐标 和角度
        eePos, eeOrn = self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[:2]
        # 打印末端执行器的坐标
        print("End Effector Position:", eePos)
        print("End Effector Orientation:", eeOrn)
    
    def update_state(self):
        # 根据当前状态，位置和当前手指情况判断更新状态
        if self.state == 0: # 0 --> 1要求当前位置的距离到目标位置距离足够小
            endEffectorPosition = np.array(self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[0])
            if np.linalg.norm(endEffectorPosition - self.target_position) < 0.05:   # 我真是没想到这个参数也蛮关键的，稍大一点机械臂还没有就位就开始抓取了，如果太小，机械臂精度没有那么高，就会卡住
                self.state = 1

        elif self.state == 1: # 1 --> 2要求当前位置的距离到目标位置距离足够小
            endEffectorPosition = np.array(self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[0])
            if np.linalg.norm(endEffectorPosition - self.target_position) < 0.03:
                self.state = 2
        
        elif self.state == 2: # 2 --> 3要求计算目前手指张开的程度
            joint_state = p.getJointState(self.pandaID, 9)
            finger_position = abs(joint_state[0] - 0) / 0.04
            if finger_position < 0.68:  # 别问我这个数字怎么来的，问就是调参
                self.state = 3


    def step(self, trajectory):
        self.update_state()
        
        if self.state == 0 or self.state == 1 or self.state == 2:
            # 拿方块块
            if self.state == 0: # 0 --> 1
                self.target_position = np.array([obj_start_coordinate[0], obj_start_coordinate[1], 0.5])
                # target_orn = self.bullet_client.getQuaternionFromEuler([-math.pi/2., 0., 0.])
                # 我实在是搞不清楚这个四元数到底是什么样子了，直接copy一下初始状态
                target_orn = (0.9704765949509994, -0.24119311625005094, 0.0007556229518208201, 0.0006988274880907078)
                targetPose = self.bullet_client.calculateInverseKinematics(self.pandaID, self.endEffectorIndex, self.target_position, target_orn)
                self.finger_target = 0.04
            elif self.state == 1:   # 1 --> 2 
                self.target_position = np.array([obj_start_coordinate[0], obj_start_coordinate[1], 0.0])
                target_orn = (0.9704765949509994, -0.24119311625005094, 0.0007556229518208201, 0.0006988274880907078)
                targetPose = self.bullet_client.calculateInverseKinematics(self.pandaID, self.endEffectorIndex, self.target_position, target_orn)
                self.finger_target = 0.04
            elif self.state == 2:   # 2 --> 3
                self.target_position = np.array([obj_start_coordinate[0], obj_start_coordinate[1], 0.0])
                target_orn = (0.9704765949509994, -0.24119311625005094, 0.0007556229518208201, 0.0006988274880907078)
                targetPose = self.bullet_client.calculateInverseKinematics(self.pandaID, self.endEffectorIndex, self.target_position, target_orn)
                self.finger_target = 0.01
            # else:   # 3 --> 4
                # self.target_position = np.array([obj_start_coordinate[0], obj_start_coordinate[1], 0.5])
                # target_orn = (0.9704765949509994, -0.24119311625005094, 0.0007556229518208201, 0.0006988274880907078)
                # targetPose = self.bullet_client.calculateInverseKinematics(self.pandaID, self.endEffectorIndex, self.target_position, target_orn)
                # self.finger_target = 0.0
            for i in range(9):
                self.bullet_client.setJointMotorControl2(bodyIndex = self.pandaID, 
                                                        jointIndex = i,
                                                        targetPosition = targetPose[i],
                                                        targetVelocity = 0,
                                                        controlMode = self.bullet_client.POSITION_CONTROL,
                                                        force = 100,
                                                        positionGain = 0.03,
                                                        velocityGain = 1)
            for i in [9, 10]:
                self.bullet_client.setJointMotorControl2(self.pandaID, i, self.bullet_client.POSITION_CONTROL, self.finger_target, force = 35)

            self.bullet_client.stepSimulation()
            time.sleep(TIMESTEP)
        
        elif self.state == 3:
            # 转圈圈
            # 转圈我本来也想写进来，但是不知道为什么写在这里就不work
            # 最后这个问题排查出来了，是p.stepSimulation位置的问题，当你想要画圆的时候，应该是每一次移动到圆上一个新的点就需要stepSimulate一下
            # 但是我在一开始把stepSimulation这个函数写在while循环底下了，这就相当于是每一次机械臂移动到轨迹上的最后一个点之后才会执行一次仿真，最后的结果当然就是像一开始的错误一样，不动
            # 所以我还是把stepSimulation写在step里吧，别写在main函数里面

            cycle = len(trajectory)     # 一个周期一共有多少点
            self.t += 1
            if self.t >= cycle:
                self.t = 0
            
            self.target_position = trajectory[self.t]
            target_orn = (0.9704765949509994, -0.24119311625005094, 0.0007556229518208201, 0.0006988274880907078)
            targetPose = self.bullet_client.calculateInverseKinematics(self.pandaID, self.endEffectorIndex, self.target_position, target_orn)
            for i in range(9):
                self.bullet_client.setJointMotorControl2(bodyIndex = self.pandaID, 
                                                        jointIndex = i,
                                                        targetPosition = targetPose[i],
                                                        targetVelocity = 0,
                                                        controlMode = self.bullet_client.POSITION_CONTROL,
                                                        force = 500,
                                                        positionGain = 0.03,
                                                        velocityGain = 1 )

            self.bullet_client.stepSimulation()
            time.sleep(0.01)


