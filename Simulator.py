#coding=gbk
import pybullet as p
import pybullet_data
import time 
import math 
import numpy as np

# ��е�۵ĳ�ʼconfiguration����������ô���ġ���
jointPositions=[0.5, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
# ����ĳ�ʼλ��
obj_start_coordinate = np.array([0.5, 0.5, 0.])
# ÿһ������֮��ļ��
TIMESTEP = 0.01

# �������������ǣ��ȴӳ�ʼλ��������ڵ����ϵ����壬Ȼ����Բ�ι켣�˶���
# 0����ʼ״̬��ǯ�Ӵ� ->
# 1: λ���������Ϸ���ǯ�ӳ����·�->
# 2: �ִ�����λ�ã�׼��ץȡ->
# 3: �ɹ�ץȡ��ǯ�ӱպϣ�->
# 4: ���ո����Ĺ켣�˶�.

class myPanda():
    def __init__(self, bullet_client, offset):
        self.bullet_client = bullet_client
        self.bullet_client.loadURDF("plane.urdf")
        
        self.obj = self.bullet_client.loadURDF("cube_small.urdf", obj_start_coordinate)

        self.pandaID = self.bullet_client.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase = True)
        self.numJoints = self.bullet_client.getNumJoints(self.pandaID)
        self.endEffectorIndex = 11

        self.t = 0      # �����������������չ켣�˶��ĵط��ġ�
        
        self.target_position = self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[-2]
        self.firger_target = 0.04
        
        # self.finger_state = 0.04    # ��ʼ��ָ�ſ�
        # ��ʼ����е�۵�configuration����Ϊstate0
        self.state = 0
        for i in range(9):
            self.bullet_client.resetJointState(self.pandaID, i, jointPositions[i])
        # ��ʼ����ָ������ָ�ſ�
        for i in [9, 10]:
            self.bullet_client.setJointMotorControl2(self.pandaID, i, self.bullet_client.POSITION_CONTROL, self.firger_target, force = 10)
        # for test
        # ��ȡĩ��ִ���������� �ͽǶ�
        eePos, eeOrn = self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[:2]
        # ��ӡĩ��ִ����������
        print("End Effector Position:", eePos)
        print("End Effector Orientation:", eeOrn)
    
    def update_state(self):
        # ���ݵ�ǰ״̬��λ�ú͵�ǰ��ָ����жϸ���״̬
        if self.state == 0: # 0 --> 1Ҫ��ǰλ�õľ��뵽Ŀ��λ�þ����㹻С
            endEffectorPosition = np.array(self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[0])
            if np.linalg.norm(endEffectorPosition - self.target_position) < 0.05:   # ������û�뵽�������Ҳ���ؼ��ģ��Դ�һ���е�ۻ�û�о�λ�Ϳ�ʼץȡ�ˣ����̫С����е�۾���û����ô�ߣ��ͻῨס
                self.state = 1

        elif self.state == 1: # 1 --> 2Ҫ��ǰλ�õľ��뵽Ŀ��λ�þ����㹻С
            endEffectorPosition = np.array(self.bullet_client.getLinkState(self.pandaID, self.endEffectorIndex)[0])
            if np.linalg.norm(endEffectorPosition - self.target_position) < 0.03:
                self.state = 2
        
        elif self.state == 2: # 2 --> 3Ҫ�����Ŀǰ��ָ�ſ��ĳ̶�
            joint_state = p.getJointState(self.pandaID, 9)
            finger_position = abs(joint_state[0] - 0) / 0.04
            if finger_position < 0.68:  # ���������������ô���ģ��ʾ��ǵ���
                self.state = 3


    def step(self, trajectory):
        self.update_state()
        
        if self.state == 0 or self.state == 1 or self.state == 2:
            # �÷����
            if self.state == 0: # 0 --> 1
                self.target_position = np.array([obj_start_coordinate[0], obj_start_coordinate[1], 0.5])
                # target_orn = self.bullet_client.getQuaternionFromEuler([-math.pi/2., 0., 0.])
                # ��ʵ���Ǹ㲻��������Ԫ��������ʲô�����ˣ�ֱ��copyһ�³�ʼ״̬
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
            # תȦȦ
            # תȦ�ұ���Ҳ��д���������ǲ�֪��Ϊʲôд������Ͳ�work
            # �����������Ų�����ˣ���p.stepSimulationλ�õ����⣬������Ҫ��Բ��ʱ��Ӧ����ÿһ���ƶ���Բ��һ���µĵ����ҪstepSimulateһ��
            # ��������һ��ʼ��stepSimulation�������д��whileѭ�������ˣ�����൱����ÿһ�λ�е���ƶ����켣�ϵ����һ����֮��Ż�ִ��һ�η��棬���Ľ����Ȼ������һ��ʼ�Ĵ���һ��������
            # �����һ��ǰ�stepSimulationд��step��ɣ���д��main��������

            cycle = len(trajectory)     # һ������һ���ж��ٵ�
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


