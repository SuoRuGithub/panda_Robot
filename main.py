#coding=gbk
import pybullet as p
import pybullet_data
import time 
import math 
import numpy as np
import Simulator

# 初始化环境
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
TIMESTEP = 0.01

# 定义圆轨迹，轨迹是三维数组的列表
circle_radius = 0.4
circle_center = [0.5, 0.5]
numPoints = 180
angles = [2 * math.pi * i / numPoints for i in range(numPoints)]
trajectory = np.array([[circle_center[0] + circle_radius * math.cos(angle), 
                        circle_center[1] + circle_radius * math.sin(angle), 
                        0.5] for angle in angles])
# trajectory = [[0.5, 0.5, 0], [1, 0, 0], [0.5, 0, 0]]
def main():
    # 这个main函数还不如不写……
    panda = Simulator.myPanda(p, [0, 0, 0])

    while True:
        panda.step(trajectory)
        

if __name__ == "__main__":
    main()

p.disconnect()