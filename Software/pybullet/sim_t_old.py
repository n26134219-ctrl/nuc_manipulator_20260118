import pybullet as p
import pybullet_data
import numpy as np
import math
import time
from math import pi
import tkinter as tk


#p.GUI : 圖形化界面

physicsClient = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=1 --background_color_blue=0.9')#設定物理引擎 # Connect to pybullet GUI
p.setGravity(0, 0, -9.8) #設定重力加速度 (gravity force along the X world axis,gravity force along the Y world axis,gravity force along the Z world axis)
p.resetDebugVisualizerCamera(
                            cameraDistance=2,            # 相機與目標位置之間的距離(m)
                            cameraYaw=0, # 水平旋轉角度,0 度代表正視，90 度表示從右側觀察，-90 度則表示從左側觀察,角度單位(度)
                            cameraPitch=-30, # 垂直旋轉角度 0 度表示水平視角，負值表示向下看，正值表示向上看 角度單位
                            cameraTargetPosition=(0, 0, 0.55) # 相機瞄準的位置
                            )
p.setRealTimeSimulation(1)# 設定模擬
# 添加資源路徑，內建urdf檔案的路徑添加進來
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

#mugId = p.loadURDF("urdf/mug.urdf",basePosition = [1,0,0], baseOrientation = p.getQuaternionFromEuler([0, 0, 0]), globalScaling = 1, useFixedBase=True)#比例:globalScaling

StartPos = [0, 0, 0.5]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# RobotId = p.loadURDF('mini_arm_URDF_V10/urdf/mini_arm_URDF_V10.urdf', StartPos, StartOrientation,useFixedBase=True)
# RobotId = p.loadURDF('/home/aiRobots/pybullet/open_manipulator_p_description/urdf/open_manipulator_p.urdf', StartPos, StartOrientation,useFixedBase=True)
RobotId = p.loadURDF('/home/aiRobots/pybullet/open_manipulator_p_description/urdf/open_manipulator_p_with_gripper.urdf', StartPos, StartOrientation,useFixedBase=True)

end_effector_index = 6  # 末端執行器的連結編號（依模型而定）
target_pos = [0.3, 0.3, 0.0]
target_orn = p.getQuaternionFromEuler([0, 0, 0])
# 建立小方塊碰撞形狀，尺寸為邊長0.1m的立方體
box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01])

# 建立方塊剛體，質量1公斤，位置在原點上方0.1m處
box_id = p.createMultiBody(baseMass=0,
                           baseCollisionShapeIndex=box_collision,
                           basePosition=[target_pos[0], target_pos[1], target_pos[2]+0.5 ],
                           baseOrientation=[0, 0, 0])
joint_start_index = 1  # 關節索引起點
joint_end_index = 6    # 關節索引終點

while True:
    joint_angles = p.calculateInverseKinematics(RobotId, end_effector_index, target_pos, target_orn)
    # 控制關節索引 1 到 6
    for i in range(joint_start_index, joint_end_index + 1):
        # calculateInverseKinematics 有時會回傳 >6 個角度，這裡用 i-1 對應計算結果
        p.setJointMotorControl2(RobotId, i, p.POSITION_CONTROL, joint_angles[i-1])
    p.stepSimulation()
    time.sleep(1/240)

