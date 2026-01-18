import pybullet as p
import pybullet_data
import numpy as np
import math
import time

# import tkinter as tk

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

StartPos = [0, 0, 1]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# RobotId = p.loadURDF('mini_arm_URDF_V10/urdf/mini_arm_URDF_V10.urdf', StartPos, StartOrientation,useFixedBase=True)
# RobotId = p.loadURDF('/home/aiRobots/pybullet/open_manipulator_p_description/urdf/open_manipulator_p.urdf', StartPos, StartOrientation,useFixedBase=True)
# RobotId = p.loadURDF('/home/aiRobots/pybullet/open_manipulator_p_description/urdf/open_manipulator_p_with_gripper.urdf', StartPos, StartOrientation,useFixedBase=True)

# RobotId = p.loadURDF('/home/aiRobots/pybullet/單手URDF_第二張圖_第一版20250921.SLDASM/urdf/單手URDF_第二張圖_第一版20250921.SLDASM.urdf', StartPos, StartOrientation,useFixedBase=True)
# RobotId = p.loadURDF('/home/aiRobots/pybullet/單手URDF_第二張圖_第二版20250922.SLDASM/urdf/單手URDF_第二張圖_第二版20250922.SLDASM.urdf', StartPos, StartOrientation,useFixedBase=True)
RobotId = p.loadURDF('/home/aiRobots/Software/pybullet/全身URDF_第二版20250923.SLDASM/urdf/全身URDF_第二版20250923.SLDASM.urdf', StartPos, StartOrientation,useFixedBase=True)



# 建立滑桿控制每個關節
joint_sliders = []
num_joints = p.getNumJoints(RobotId)

for j in range(num_joints):
    info = p.getJointInfo(RobotId, j)
    print(info)
    joint_angle = p.getJointState(RobotId, j)[0]
    print("joint angle:", joint_angle)
    joint_name = info[1].decode('utf-8')
    lower = info[8]  # lower limit
    upper = info[9]  # upper limit
    # 如果沒有設定限制，就使用預設 -pi ~ pi
    if lower > upper:
        lower = -3.14159
        upper = 3.14159
    slider = p.addUserDebugParameter(joint_name, lower, upper, 0)
    joint_sliders.append(slider)

# 模擬迴圈
while True:
    for j in range(num_joints):

        target_pos = p.readUserDebugParameter(joint_sliders[j])
        p.setJointMotorControl2(RobotId, j, p.POSITION_CONTROL, targetPosition=target_pos)
    p.stepSimulation()
    time.sleep(1./240.)