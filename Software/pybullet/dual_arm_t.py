import pybullet as pb
import numpy as np
import ArmInfo
import time
from math import *
import pybullet_data
# import rospy
# from std_msgs.msg import Int16MultiArray
import threading
import sys
from scipy.spatial.transform import Rotation as R
sys.path.append('/home/airobots/gpt-oss/GroundingDINO')

import camera_class 
# from GroundingDINO import camera_class 
# import scipy.linalg

# set up simulation
physicsClient = pb.connect(pb.GUI,  options='--background_color_red=0.0 --background_color_green=0.66 --background_color_blue=0.66')

chest_position = 1.1
StartPos = [0, 0, chest_position]
StartOrientation = pb.getQuaternionFromEuler([0, 0, 0])
## loading robot urdf
flags = pb.URDF_USE_SELF_COLLISION        \
      | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT


# RobotId = pb.loadURDF('/home/aiRobots/pybullet/全身URDF_第二版20250923.SLDASM/urdf/全身URDF_第二版20250923.SLDASM.urdf', StartPos, StartOrientation, useFixedBase=True, flags=flags)
RobotId = pb.loadURDF('/home/airobots/gpt-oss/pybullet/全身URDF_第二版20250923.SLDASM/urdf/全身URDF_第二版20250923.SLDASM.urdf', StartPos, StartOrientation, useFixedBase=True)

## loading plane urdf
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF("plane.urdf")


pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)
pb_lock = threading.Lock()


left_arm_DH_table = np.array([[0,                     0,       0,     233], #159
                                [0,                  -pi/2,       0,      0],
                                [0,                     0,     265.7,   0],
                                [0,                 -pi/2,     30,      0],
                                [0,                  pi/2,       0,      264],
                                [0,                 -pi/2,       0,      0],
                                [0,                     0,       0,     235.27]
                                ])
right_arm_DH_table = np.array([[0,                     0,       0,     233],
                                [0,                  -pi/2,       0,      0],
                                [0,                     0,     265.7,   0],
                                [0,                 -pi/2,     30,      0],
                                [0,                  pi/2,       0,      264],
                                [0,                 -pi/2,       0,      0],
                                [0,                     0,       0,     235.27]
                                ])
## left id 0~6
left_arm = ArmInfo.ArmInfo(left_arm_DH_table, start_id=0, end_id=6)
## right id 7~12
right_arm = ArmInfo.ArmInfo(right_arm_DH_table, start_id=7, end_id=12)

left_arm.set_world_to_base_transform(-pi/2, 0)# 座標點不動，逆旋轉矩陣來轉換點座標
right_arm.set_world_to_base_transform(pi/2, 0)# 座標點不動，逆旋轉矩陣來轉換點座標

left_box_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01], rgbaColor=[0, 0, 0, 1])  # 黑色，最後一個是透明度)
right_box_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01], rgbaColor=[1, 0, 0, 1])  # 灰白色，最後一個是透明度)
object_box_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[0, 1, 0, 1])  # 綠色，最後一個是透明度)
# robot workspace limit
max_z = 750
min_z = -150
max_x = 700
min_x = 200
max_y = 350
min_y = -400
gripper_offset = 100 # in mm
object_offset = 100 # in mm
def update_arm_info():
    for i in range(0, 6):
        left_arm.motor_angles[i] = pb.getJointState(RobotId, i)[0]
    left_arm.set_delta_angle(left_arm.motor_angles)
    print("left_arm.delta_angle:", left_arm.delta_angle)
    left_arm.update()

    for i in range(6, 12):
        right_arm.motor_angles[i-6] = pb.getJointState(RobotId, i)[0]
    
    right_arm.set_delta_angle(right_arm.motor_angles )
    print("right_arm.delta_angle:", right_arm.delta_angle)
    right_arm.update()

def initialize():    
    initial_angle = np.array([0, -pi/2, 0,  0, 0, 0])# -pi*2/3 
    # initial_angle = np.array([pi/2, -pi/2, 0,  0, 0, 0])
    print("Initialize robot pose...")
    # intial robot pose
    ## left pose
    for i in range(0, 6):
        pb.setJointMotorControl2(RobotId, jointIndex=i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition =initial_angle[i],
                                    force = 5000)
        # left_arm.motor_angles[i] = initial_angle[i]
        # print(pb.getJointState(RobotId, i)[0])  
    
     
    time.sleep(2)
    for _ in range(1500):
        pb.stepSimulation()   
    for i in range(0, 6):
        left_arm.motor_angles[i] = pb.getJointState(RobotId, i)[0]
    left_arm.set_delta_angle(left_arm.motor_angles)
    print("left_arm.delta_angle:", left_arm.delta_angle)
    left_arm.update()

    # right pose
    for i in range(6, 12):
        pb.setJointMotorControl2(RobotId, jointIndex=i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition =initial_angle[i-6],
                                    force = 5000)
        # right_arm.motor_angles[i-6] = initial_angle[i-6]
        # print(pb.getJointState(RobotId, i)[0])  
    time.sleep(1)

    for _ in range(1500):
        pb.stepSimulation() 
    # print(pb.getJointState(RobotId, 7)[0])
    for i in range(6, 12):
        right_arm.motor_angles[i-6] = pb.getJointState(RobotId, i)[0]
    
    right_arm.set_delta_angle(right_arm.motor_angles )
    right_arm.update()
    ## neck pose
    for i in range(12, 14):
        pb.setJointMotorControl2(RobotId, jointIndex=i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition =0,
                                    force = 50000)
    time.sleep(1)

    for _ in range(1000):
        pb.stepSimulation()
   
    
    print("robot pose initialized.")
    print("===================================")
    print("Current left Position = {}".format(np.around(left_arm.current_position, 0)))
    print("Current left Orientation = {}".format(np.around(left_arm.get_current_orientation_in_degree(), 0)))
    print("===================================")
    print("===================================")
    print("Current right Position = {}".format(np.around(right_arm.current_position, 0)))
    print("Current right Orientation = {}".format(np.around(right_arm.get_current_orientation_in_degree(), 0)))
    print("===================================")
    time.sleep(1)
    # TESTING
    # print("===== Testing Jacobian Matrix =====")
    # # single_arm.delta_angle = np.array([0, -pi/2, 0, 0, 0, 0]) 
    # # print("single_arm.delta_angle",single_arm.delta_angle)
    # for i in range(6, 12):
    #     pb.setJointMotorControl2(RobotId, jointIndex=i,
    #                                 controlMode=pb.POSITION_CONTROL,
    #                                 targetPosition =initial_angle[i-6]+[0, 0, 0, 0, 0, 0][i-6],
    #                                 force = 50000)
    # for _ in range(1000):
    #     pb.stepSimulation()
    # time.sleep(1)
    # for i in range(0, 6):
    #     # print(pb.getJointState(RobotId, i)[0])        
    #     left_arm.set_motor_angles(i, pb.getJointState(RobotId, i)[0] )
    #     # single_arm.delta_angle[i] = single_arm.motor_angles[i]                              
    # left_arm.set_delta_angle( left_arm.motor_angles )
    # left_arm.update()
    # print("self.jocobian_matrix = {}".format(np.around(single_arm.jacobian_matrix, 2)))

    print("==============================================================")
   
    
    # print(f"right_w_to_b:{right_w_to_b}")
    # print(single_arm.current_position)
    # matrix= np.matmul(right_w_to_b,[single_arm.current_position[0], single_arm.current_position[1], single_arm.current_position[2],1])
    left_box_position = left_arm.transform_to_world_position(left_arm.current_position)

    left_initial = pb.createMultiBody(baseMass=0,
                            baseVisualShapeIndex=left_box_id,
                            basePosition=[left_box_position[0]/1000, left_box_position[1]/1000, left_box_position[2]/1000 + chest_position],
                            baseOrientation=[0, 0, 0, 1])
    
    print("left box position:", [left_box_position[0], left_box_position[1], left_box_position[2]])
    
    right_box_position = right_arm.transform_to_world_position(right_arm.current_position)

    right_initial = pb.createMultiBody(baseMass=0,
                            baseVisualShapeIndex=right_box_id,
                            basePosition=[right_box_position[0]/1000, right_box_position[1]/1000, right_box_position[2]/1000 + chest_position],
                            baseOrientation=[0, 0, 0, 1])
    
    print("right box position:", [right_box_position[0], right_box_position[1], right_box_position[2]])
 
    return left_initial, right_initial
 
    


   
    # print(np.around(single_arm.joint_velocity, 2))
def stop_all_motors(mode = "left"):
    if mode == "left":
        for i in range(0, 6):
                pb.setJointMotorControl2(RobotId, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = 0,
                                            force = 5000)
    if mode == "right":
        for i in range(6, 12):
                pb.setJointMotorControl2(RobotId, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = 0,
                                            force = 5000)
    if mode == "both":
        for i in range(0, 12):
                pb.setJointMotorControl2(RobotId, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = 0,
                                            force = 5000)
    for _ in range(1000):
        pb.stepSimulation() 
def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])

def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)
def step_IK( ox_deg, oy_deg, oz_deg, px, py, pz, mode="left", single_arm=left_arm):
    velocity_factor = 0.1
    vel_gain_= 1.5
    vel_gain_angular_ = 1.2
    ki_gain_=0.1
    angular_threshold = pi/180
    linear_threshold = 1 # in mm
    stop_counter = 0
    acceleration_factor = 0.0
    acceleration_counter = 0
    
    single_arm.set_target_orientation(ox_deg, oy_deg, oz_deg)# single_arm.target_orientation in radian
    single_arm.set_target_position(px, py, pz)

    print("Target Orientation = {}".format(np.around(single_arm.target_orientation )))
    
    print("single_arm.current_orientation = {}".format(np.around(single_arm.current_orientation, 2)))

    print("Target  in base = [{}, {}, {}]".format(px, py, pz))
    print("single_arm.current_position = {}".format(np.around(single_arm.current_position, 2)))
    last_position = single_arm.current_position
    single_arm.update()

   
    ## workspace limit
    change_t = False
    if px > max_x:
        px = max_x
        change_t = True
    if px < min_x:
        px = min_x
        change_t = True
    if py > max_y:
        py = max_y
        change_t = True
    if py < min_y:
        py = min_y
        change_t = True
    if pz > max_z:
        pz = max_z
        change_t = True
    if change_t:
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(f"Warning: Target ({px:.1f}, {py:.1f}, {pz:.1f}) is out of workspace limit")
        single_arm.set_target_position(px, py, pz)
        print(f"Adjusted target to: ({single_arm.target_position[0]:.1f}, {single_arm.target_position[1]:.1f}, {single_arm.target_position[2]:.1f})")

    
    # set up angular error 
    # q_target = pb.getQuaternionFromEuler([float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])])
    # r_target = R.from_euler('XYZ', [float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])], degrees=False)
    r_target = R.from_euler('XYZ', [float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])], degrees=False)

    q_target = r_target.as_quat()  # (x, y, z, w)
    r_current = R.from_euler('XYZ', [float(single_arm.current_orientation[0]), float(single_arm.current_orientation[1]), float(single_arm.current_orientation[2])], degrees=False)
    # q_current = pb.getQuaternionFromEuler([float(single_arm.current_orientation[0]),
    #                                         float(single_arm.current_orientation[1]),
    #                                         float(single_arm.current_orientation[2])])
    q_current = r_current.as_quat()  # (x, y, z, w)
    print("q_current =", q_current)
    print("q_target  =", q_target)
    

    dot = q_current[0]*q_target[0] + q_current[1]*q_target[1] + q_current[2]*q_target[2] + q_current[3]*q_target[3]
    # 若內積為負，翻轉目標四元數
    if dot < 0.0:
        q_target = (-q_target[0], -q_target[1], -q_target[2], -q_target[3])
    q_err = quat_mul(q_target, quat_conjugate(q_current))
    print("q_err     =", q_err)
    
    qw = max(min((q_err[3]), 1.0), -1.0)  # 取絕對值
    angle = 2.0 * acos(qw)
    print("angle rad =", angle)
    
    # 檢查是否接近零旋轉
    if abs(angle) < 1e-6:
        axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    else:
        # 計算旋轉軸
        s = sqrt(1.0 - qw*qw)
        if s < 1e-8:
            axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        else:
            axis = np.array([q_err[0]/s, q_err[1]/s, q_err[2]/s], dtype=np.float32)
    # 確保選擇最短路徑（角度小於π）
    print("angle rad =", angle)
    print("axis      =", axis)
    if angle > np.pi:
        angle = 2*np.pi - angle
        axis = -axis
    error_ang_vec = axis * angle
    print("angle rad =", angle)
    print("axis      =", axis)
    print("error_ang_vec = {}".format(np.around(error_ang_vec, 2)))
    # time.sleep(100)
    # error_ang_vec = single_arm.target_orientation - single_arm.current_orientation
    angular_error_norm = np.linalg.norm(error_ang_vec)
    # # ang_err_norm = np.linalg.norm(error_ang_vec)
    # set up linear error
    error_pos = single_arm.target_position - single_arm.current_position
    linear_error_norm = np.linalg.norm(error_pos)

    # check if reached

    if linear_error_norm < linear_threshold and angular_error_norm < angular_threshold:
        stop_all_motors(mode=mode) 
        single_arm.set_error_integral(np.zeros(6)) 
        return True

    # # 積分

    # new_error_integral = single_arm.error_integral_ 
    # new_error_integral[:3] += error_ang_vec * single_arm.dt_
    # new_error_integral[3:] += error_pos * single_arm.dt_
    # new_error_integral = np.clip(new_error_integral, -single_arm.max_integral_, single_arm.max_integral_)
    # single_arm.set_error_integral(new_error_integral)

    # # PI
    # w = error_ang_vec * vel_gain_angular_ + single_arm.error_integral_[:3] * ki_gain_
    # v = error_pos * vel_gain_ + single_arm.error_integral_[3:] * ki_gain_

    # P
    w = error_ang_vec * vel_gain_angular_ 
    v = error_pos * vel_gain_ 

    # 沒有四元數
    # error_angu = np.array((single_arm.target_orientation[0] - single_arm.current_orientation[0], single_arm.target_orientation[1] - single_arm.current_orientation[1], single_arm.target_orientation[2] - single_arm.current_orientation[2]), dtype=np.float32)
    v_mirror = np.array([v[0], -v[1], v[2]], dtype=np.float32)
   
    # w = error_angu * vel_gain_angular_

    # 距離自適應
    if linear_error_norm > 100:
        print("Speed up")
        v *= 1.5
    elif linear_error_norm < 10:
        print("Slow down")
        v *= 0.7

    # [角, 線] -> xdot
    xdot = np.concatenate([w, v]).astype(np.float32)
   
    # q'=inverse(J) * xdot
    print(  "xdot = {}".format(np.around(xdot, 2)))
  
    single_arm.set_joint_velocity( xdot)
    
    # qdot = solveDLS(self.J_, xdot, self.lambda_dls_)

    # 限速
    single_arm.check_motor_limit()

    # deadzone
    deadzone = 5e-4
    single_arm.joint_velocity[np.abs(single_arm.joint_velocity) < deadzone] = 0.0
    
    # control in velocity mode
    if mode == "left":
        range_start = 0
        range_end = 6
        # client = physicsClient
    if mode == "right":
        range_start = 6
        range_end = 12
    
            # print(  "right_arm.joint_velocity = {}".format(np.around(right_arm.joint_velocity, 2)))
            # client = physicsClient_R
    # if mode == "both":
    #     range_start = 0
    #     range_end = 12
    
    with pb_lock:
        for i in range(range_start, range_end):
            pb.setJointMotorControl2(RobotId, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = float(single_arm.joint_velocity[i-range_start]),
                                        force = 1500)
    
    for i in range(range_start, range_end):
        single_arm.set_motor_angles(i-range_start, pb.getJointState(RobotId, i)[0])
    single_arm.set_delta_angle(single_arm.motor_angles)
    single_arm.update()              

    # # 卡住檢測
    # if not hasattr(self, 'stall_counter'):
    #     self.stall_counter = 0
    # if not hasattr(self, 'last_position'):
    #     self.last_position = self.current_position_.copy()

    # position_change = np.linalg.norm(single_arm.current_position - last_position)
    # if position_change < 0.1:
    #     single_arm.set_stall_counter(single_arm.stall_counter + 1)
    #     if single_arm.stall_counter > 50:
    #         single_arm.set_error_integral(single_arm.error_integral_ * 0.5)
    #         single_arm.set_stall_counter(0)
            
    # else:
    #     single_arm.set_stall_counter(0)
    return False



def trajectory_planning(ox, oy, oz, px, py, pz, mode = "left",single_arm=left_arm):

    start_time = time.time()
    timeout = 30  # seconds
    iteration = 0
    # trajectory_planning(ox, oy, oz, px, py, pz)
    while time.time() - start_time < timeout:
            iteration += 1
            
            if step_IK(ox, oy, oz, px, py, pz, mode, single_arm):
                single_arm.update()
                right_arm.update()
                error = np.linalg.norm(np.array([px, py, pz]) - single_arm.current_position)
                print(f"Target reached! distance error: {error:.2f} mm (iter: {iteration})")
                print(f"Final base position: ({single_arm.current_position[0]:.1f}, {single_arm.current_position[1]:.1f}, {single_arm.current_position[2]:.1f}) mm")
                current_position_world = single_arm.transform_to_world_position(single_arm.current_position)
                print(f"Final  position: ({current_position_world[0]:.1f}, {current_position_world[1]:.1f}, {current_position_world[2]:.1f}) mm in world coordinate")
                stop_all_motors(mode)
                return True
            time.sleep(single_arm.dt_)
    print("Timeout: Failed to reach the target within the time limit.")
def open_gripper():
    print("Open gripper")
    pb.setJointMotorControl2(RobotId, jointIndex=8,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = 0.0,
                                    force = 50000)
    pb.setJointMotorControl2(RobotId, jointIndex=9,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = 0.0,
                                    force = 50000)
    
def close_gripper():
    print("Close gripper")
    pb.setJointMotorControl2(RobotId, jointIndex=8,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = 1.1,
                                    force = 50000)
    pb.setJointMotorControl2(RobotId, jointIndex=9,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = 1.1,
                                    force = 50000)

def callback(data):
    global fx
    global fy
    global fz

    fx = data.data[0]
    fy = data.data[1]
    fz = data.data[2]





# 關節角度範圍（rad）
# joint_limits = [
#     (-np.pi, np.pi),
#     (-np.pi/2, np.pi/2),
#     (-np.pi/2, 3*np.pi/4),
#     (-np.pi, np.pi),
#     (-np.pi/2, np.pi/2),
#     (-np.pi, np.pi)
# ]

# def listener():    
#     rospy.Subscriber("ForseSensor_C2Python", Int16MultiArray, callback)
#     rospy.spin()

# rospy.init_node("aiRobots_Python", anonymous=True)
# sub_td = threading.Thread(target=listener)
# sub_td.start()


left_initial, right_initial = initialize()
time.sleep(1)
# close_gripper()
# time.sleep(2)
# open_gripper()
# time.sleep(3)

# for _ in range(1000):
#     print("Step simulation")
#     pb.setJointMotorControl2(RobotId, jointIndex=2,
#                             controlMode=pb.VELOCITY_CONTROL,
#                             targetVelocity=-0.2,
#                             force=50000)
#     pb.stepSimulation()

#######################
# target_position_world = np.array([450, 0, 0]) # in world coordinate, in mm
# target_position_world = np.array([300, 200, -100]) # in world coordinate, in mm
def target_ori_arrange(target_position_world, target_orientation, mode = "left", pick_mode = "auto", angle=90):
    target_orientation = [-180, 0, 0]
    if pick_mode == "auto":
        if mode == "left":
            x = abs(target_position_world[0])
            y = target_position_world[1]
            if y >= 0 and y <= 250 or y <= 0 and y >= -150:
                if x >= 0 and x <= 350:
                    target_orientation[0] = -165
                    target_orientation[1] = 0 - 10 * (x/350)
                elif x > 350 and x <= 500:
                    target_orientation[1] = -15 - 15 * ((x - 350)/150)
                elif x > 500 and x < 700:
                    target_orientation[1] = -30 - 20 * (x - 500)/200 
                elif x >= 700:
                    target_orientation[1] = -80   
            elif y > 250 and y <= 450:
                # target_orientation[0] = -10
                target_orientation[1] = -90
                target_orientation[2] = 0 #-4
                # target_orientation[1] = -80
            elif y > 450:
                target_orientation[0] = 0
                target_orientation[1] = -30  #-30 - 20 * ((y - 450)/250)
                target_orientation[2] = -180
                # target_orientation[1] = -90 - 20 * ((y - 500)/150)
        if mode == "right":
            x = abs(target_position_world[0])
            y = target_position_world[1]
            x = abs(target_position_world[0])
            y = target_position_world[1]
            if y <= 0 and y >= -250 or y >= 0 and y <= 150:
                if x >= 0 and x <= 350:
                    target_orientation[0] = -165
                    target_orientation[1] = 0 - 10 * (x/350)
                elif x > 350 and x <= 500:
                    target_orientation[1] = -15 - 15 * ((x - 350)/150)
                elif x > 500 and x < 700:
                    target_orientation[1] = -30 - 20 * (x - 500)/200 
                elif x >= 700:
                    target_orientation[1] = -80   
            elif y < -250 and y >= -450:
                # target_orientation[0] = -10
                target_orientation[1] = -90
                target_orientation[2] = 0 #-4
                # target_orientation[1] = -80
            elif y < -450:
                target_orientation[0] = 0
                target_orientation[1] = -30  #-30 - 20 * ((y - 450)/250)
                target_orientation[2] = -180
        if mode == "same":
            target_orientation = [-180, 0, -90]    
    elif pick_mode == "side" :
        target_orientation = [-180, 0, 0]
    elif pick_mode == "top":
        target_orientation = [-86, 0, 0]
        if angle > 90:
            target_orientation[1] = 180-angle
            print("turn in ")
        if angle <= 90:
            target_orientation[1] = -angle
            print("turn out ")
       
    return target_orientation

def is_coordinate_input(s: str) -> bool:
    """
    判斷輸入字串是否為三維座標格式。
    支援以逗號或空格分隔的三個數字，例如 "1,2,3" 或 "1 2 3"。
    """
    # 嘗試先以逗號切割，若不是三項則改以空格切割
    parts = s.split(',')
    if len(parts) != 3:
        parts = s.split()
    if len(parts) != 3:
        return False

    # 確認每一項都能轉成浮點數
    try:
        [float(p) for p in parts]
        return True
    except ValueError:
        return False

def parse_coordinates(s: str):  # -> (float, float, float)
    parts = s.replace(',', ' ').split()
    x, y, z = map(float, parts)
    return x, y, z

    
def mode_controll(x, y, z, mode="left", pick_mode = "auto", angle=90):
    print("User Mode:",mode)
    left_target_position_world = left_arm.transform_to_world_position(left_arm.current_position)
    right_target_position_world = right_arm.transform_to_world_position(right_arm.current_position)
    if mode == "left":
        left_target_position_world = np.array([x, y, z]) # in world coordinate, in mm

        target_left_orientation = target_ori_arrange(left_target_position_world, target_orientation, mode="left", pick_mode=pick_mode, angle=angle)
        left_target_base_position = left_arm.transform_to_base_position(left_target_position_world) # in base coordinate, in mm
        # target_left_orientation = [0, 90, -90]
        print("left_target_base_position:", left_target_base_position)
        trajectory_planning(target_left_orientation[0], target_left_orientation[1], target_left_orientation[2], left_target_base_position[0], left_target_base_position[1], left_target_base_position[2], "left", single_arm=left_arm)
        print("reach point !")
        print("Current left_arm Position = {}".format(np.around(left_arm.transform_to_world_position(left_arm.current_position), 1)))
        print("Current left_arm Orientation = {}".format(np.around(left_arm.get_current_orientation_in_degree(), 1)))
        print("Target left Position = {}".format(np.around(left_target_position_world, 1)))
        print("Target left Orientation = {}".format(np.around(target_left_orientation, 1)))
    elif mode == "right":
        right_target_position_world = np.array([x, y, z])
        target_right_orientation = target_ori_arrange(right_target_position_world, target_orientation, mode="right", pick_mode=pick_mode, angle=angle)
        print("right_ori", right_arm.get_current_orientation_in_degree())
        # target_right_orientation = [-180, 0, 0]
        right_target_base_position = right_arm.transform_to_base_position(right_target_position_world) # in base coordinate, in mm
        print("right_target_base_position:", right_target_base_position)
        
        print("Start moving right arm...")
        # time.sleep(10)
        trajectory_planning(target_right_orientation[0], target_right_orientation[1], target_right_orientation[2], right_target_base_position[0], right_target_base_position[1], right_target_base_position[2], "right", single_arm=right_arm)
        print("reach point !")
        print("Current right_arm Position = {}".format(np.around(right_arm.current_position, 1)))
        print("Current right_arm Orientation = {}".format(np.around(right_arm.get_current_orientation_in_degree(), 1)))

    elif mode == "both":
        z_offset = 50
        target_positin_world = np.array([x, y, z]) # in world coordinate, in mm
        left_target_position_world = np.array([target_positin_world[0], target_positin_world[1], target_positin_world[2]])
        left_target_position_world[2] += z_offset
        right_target_position_world = target_positin_world

        target_left_orientation = target_ori_arrange(left_target_position_world, target_orientation, mode="left", pick_mode=pick_mode, angle=angle)
        target_right_orientation = target_ori_arrange(right_target_position_world, target_orientation, mode="right", pick_mode=pick_mode, angle=angle)
        left_target_base_position = left_arm.transform_to_base_position(left_target_position_world) # in base coordinate, in mm
        right_target_base_position = right_arm.transform_to_base_position(right_target_position_world) # in base coordinate, in mm
        print("left_target_base_position:", left_target_base_position)
        print("target_left_orientation", target_left_orientation)

        print("right_target_base_position:", right_target_base_position)
        print("target_right_orientation", target_right_orientation)
        time.sleep(10)
        #  left
        left_args = (
            target_left_orientation[0],
            target_left_orientation[1],
            target_left_orientation[2],
            left_target_base_position[0],
            left_target_base_position[1],
            left_target_base_position[2],
        )
        left_kwargs = {
            'mode': 'left',
            'single_arm': left_arm}
        
        #  right
        right_args = (
            target_right_orientation[0],
            target_right_orientation[1],
            target_right_orientation[2],
            right_target_base_position[0],
            right_target_base_position[1],
            right_target_base_position[2],
        )
        right_kwargs = {
            'mode': 'right',
            'single_arm': right_arm}

        thread_left = threading.Thread(target = trajectory_planning, 
                                        name="thread-Left", 
                                        args=left_args, 
                                        kwargs=left_kwargs)
        thread_right = threading.Thread(target = trajectory_planning, 
                                        name="thread-Right", 
                                        args=right_args, 
                                        kwargs=right_kwargs)
        
        thread_left.start()
        thread_right.start()

        thread_left.join()
        thread_right.join()
        # trajectory_planning(target_left_orientation[0], target_left_orientation[1], target_left_orientation[2], left_target_base_position[0], left_target_base_position[1], left_target_base_position[2], single_arm=left_arm)
        # trajectory_planning(target_right_orientation[0], target_right_orientation[1], target_right_orientation[2], right_target_base_position[0], right_target_base_position[1], right_target_base_position[2], single_arm=right_arm)
        print("reach point !")
        print("Current left_arm Position = {}".format(np.around(left_arm.current_position, 1)))
        print("Current left_arm Orientation = {}".format(np.around(left_arm.get_current_orientation_in_degree(), 1)))
        print("Current right_arm Position = {}".format(np.around(right_arm.current_position, 1)))
        print("Current right_arm Orientation = {}".format(np.around(right_arm.get_current_orientation_in_degree(), 1)))
    elif mode == "same move":
        boject_with = 150 # mm
        left_target_position_world = np.array([x , y+ boject_with/2, z]) # in world coordinate, in mm
        right_target_position_world = np.array([x , y - boject_with/2, z]) # in world coordinate, in mm

        target_left_orientation = target_ori_arrange(left_target_position_world, target_orientation, mode="same", pick_mode=pick_mode, angle=angle)
        target_right_orientation = target_ori_arrange(right_target_position_world, target_orientation, mode="same", pick_mode=pick_mode, angle=angle)
        left_target_base_position = left_arm.transform_to_base_position(left_target_position_world) # in base coordinate, in mm
        right_target_base_position = right_arm.transform_to_base_position(right_target_position_world) # in base coordinate, in mm
        print("left_target_base_position:", left_target_base_position)
        print("target_left_orientation", target_left_orientation)

        print("right_target_base_position:", right_target_base_position)
        print("target_right_orientation", target_right_orientation)
        time.sleep(10)
        #  left
        left_args = (
            target_left_orientation[0],
            target_left_orientation[1],
            target_left_orientation[2],
            left_target_base_position[0],
            left_target_base_position[1],
            left_target_base_position[2],
        )
        left_kwargs = {
            'mode': 'left',
            'single_arm': left_arm}
        
        #  right
        right_args = (
            target_right_orientation[0],
            target_right_orientation[1],
            target_right_orientation[2],
            right_target_base_position[0],
            right_target_base_position[1],
            right_target_base_position[2],
        )
        right_kwargs = {
            'mode': 'right',
            'single_arm': right_arm}

        thread_left = threading.Thread(target = trajectory_planning, 
                                        name="thread-Left", 
                                        args=left_args, 
                                        kwargs=left_kwargs)
        thread_right = threading.Thread(target = trajectory_planning, 
                                        name="thread-Right", 
                                        args=right_args, 
                                        kwargs=right_kwargs)
        
        thread_left.start()
        thread_right.start()

        thread_left.join()
        thread_right.join()
        # trajectory_planning(target_left_orientation[0], target_left_orientation[1], target_left_orientation[2], left_target_base_position[0], left_target_base_position[1], left_target_base_position[2], single_arm=left_arm)
        # trajectory_planning(target_right_orientation[0], target_right_orientation[1], target_right_orientation[2], right_target_base_position[0], right_target_base_position[1], right_target_base_position[2], single_arm=right_arm)
        print("reach point !")
        print("Current left_arm Position = {}".format(np.around(left_arm.current_position, 1)))
        print("Current left_arm Orientation = {}".format(np.around(left_arm.get_current_orientation_in_degree(), 1)))
        print("Current right_arm Position = {}".format(np.around(right_arm.current_position, 1)))
        print("Current right_arm Orientation = {}".format(np.around(right_arm.get_current_orientation_in_degree(), 1)))

    else:
        print("Error: mode should be 'left', 'right', or 'both'")
        exit(1)
    if (left_initial is not None) and (right_initial is not None):
        try:
            
            pb.removeBody(left_initial)
            pb.removeBody(right_initial)
    
        except pb.error:
            # 物體已經不存在，忽略錯誤
            pass
    


    # pb.removeBody(left_initial_box_id)
    # pb.removeBody(left_initial_box_id)
    # visual_shape_id2 = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    left_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=left_box_id,
                                basePosition=[left_target_position_world[0]/1000, left_target_position_world[1]/1000, left_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])

    right_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=right_box_id,
                                basePosition=[right_target_position_world[0]/1000, right_target_position_world[1]/1000, right_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])
    
    return left_target_box, right_target_box

# if __name__ == "__main__":
    target_orientation = np.array([-180, 0, 0]) # in degree
    
    left_target_box = None
    right_target_box = None
    angle = 0
    try:
        while True:
            m = input("請選擇模式left, right, both, same, exit（輸入 l, r, b, s, e ）：").strip().lower()
            if m == 'l':
                mode = 'left'
            elif m == 'r':
                mode = 'right'
            elif m == 'b':
                mode = 'both'
            elif m == 's':
                mode = 'same move'
            if m == 'e':
                print("程式結束")
                pb.disconnect(physicsClientId=physicsClient)
                break
            if m not in ['l', 'r', 'b', 's', 'e']:
                print("無效的模式，請重新輸入。")
                continue
            if m in ['l', 'r', 'b', 's']:
                s = input("請輸入m（move) 或 c (camera_search) （輸入 e 離開）：").strip()
                if s.lower() == 'e':
                        print("程式結束")
                        pb.disconnect(physicsClientId=physicsClient)
                        break
                if s.lower() not in ['m', 'c', 'e']:
                    print("無效的輸入，請重新輸入。")
                    continue
                if s.lower() == 'm':
                    s = input("請輸入文字或 x,y,z 座標（輸入 e 離開）：").strip()
                    if s.lower() == 'e':
                        print("程式結束")
                        pb.disconnect(physicsClientId=physicsClient)
                        break
            
                    if is_coordinate_input(s):
                        x, y, z = parse_coordinates(s)
                        print(f"偵測到座標輸入：x={x}, y={y}, z={z}")
                        p = input("請輸入姿態a(auto), s(side), t(top)（輸入 e 離開）：").strip()
                        if p.lower() == 'e':
                            print("程式結束")
                            pb.disconnect(physicsClientId=physicsClient)
                            break
                        if p.lower() not in ['a', 's', 't']:
                            print("無效的輸入，請重新輸入。")
                            continue
                        if p.lower() == 'a':
                            pick_mode = "auto"
                        elif p.lower() == 's':
                            pick_mode = "side"
                        elif p.lower() == 't':
                            pick_mode = "top"
                            angle = float(input("請輸入旋轉角度(0-180)：").strip())
                            if angle < 0 or angle > 180:
                                print("無效的輸入，請重新輸入。")
                                continue

                        
                        time.sleep(3)
                        left_target_box, right_target_box = mode_controll(x, y, z, mode, pick_mode=pick_mode, angle=angle)
                elif s.lower() == 'c':
                    object=camera_class.object_detection()
                    if object is not None:
                        for obj in object:
                            if obj['name'] == 'cup':
                                print("偵測到物體：", obj['name'])

                                camera_pos = obj['camera_position']
                                print("camera_pos", camera_pos)
                                camera_position = np.array([camera_pos[0]*1000, camera_pos[1]*1000, camera_pos[2]*1000]) # in mm
                                print("camera_position", camera_position)
                                update_arm_info()
                                camera_position[2] = camera_position[2] - gripper_offset - object_offset # gripper offset
                                pos=left_arm.transform_to_endeffector_position(camera_position)
                                print(f"偵測到ee座標輸入：x={pos[0]}, y={pos[1]}, z={pos[2]}")
                                print("object_in_base_position", left_arm.object_in_base_position)
                                object_in_world_position = left_arm.transform_to_world_position(left_arm.object_in_base_position) #in mm
                                print("object_in_world_position", object_in_world_position)
                                object_target_box = pb.createMultiBody(baseMass=0,
                                    baseVisualShapeIndex=object_box_id,
                                    basePosition=[object_in_world_position[0]/1000, object_in_world_position[1]/1000, object_in_world_position[2]/1000 + chest_position],
                                    baseOrientation=[0, 0, 0, 1])
                                x, y, z = object_in_world_position[0], object_in_world_position[1], object_in_world_position[2]
                                # x= x - object_offset
                                print(f"目標座標輸入：x={x}, y={y}, z={z}")
                                left_target_box, right_target_box = mode_controll(x, y, z, mode, pick_mode="side")
                                time.sleep(3)
                            # else:
                                # print("偵測到物體：", obj['name'])
                        # left_target_box, right_target_box = mode_controll(x, y, z, mode)
                    else:
                        print("未偵測到物體，請重新嘗試。")
                else:
                    print("未偵測到物體，請重新嘗試。")
            else:
                print(f"偵測到文字輸入：\"{s}\"")
                # 在此處可呼叫文字處理函式，例如對話或指令解析
                # handle_text_command(s)
    except KeyboardInterrupt:
        print("捕捉到 KeyboardInterrupt，結束程式")
    
    
    
    # mode = "both" # "left", "right", "both"


if __name__ == "__main__":
    target_orientation = np.array([-180, 0, 0]) # in degree
    
    left_target_box = None
    right_target_box = None
    angle = 0
    try:
        while True:
            # m = input("請選擇模式left, right, both, same, exit（輸入 l, r, b, s, e ）：").strip().lower()
            
            mode = 'right'

            s = input("請輸入文字或 ox,oy,oz 座標（輸入 e 離開）：").strip()
            if s.lower() == 'e':
                print("程式結束")
                pb.disconnect(physicsClientId=physicsClient)
                break
        
            if is_coordinate_input(s):
                ox, oy, oz = parse_coordinates(s)
                print(f"偵測到座標輸入：ox={ox}, oy={oy}, oz={oz}")
                p = input("請輸入文字或 x,y,z 座標（輸入 e 離開）：").strip()
                if p.lower() == 'e':
                    print("程式結束")
                    pb.disconnect(physicsClientId=physicsClient)
                    break
            
                if is_coordinate_input(p):
                    x, y, z = parse_coordinates(p)
                    print(f"偵測到座標輸入：x={x}, y={y}, z={z}")
                right_target_base_position = right_arm.transform_to_base_position([x, y, z ]) # in base coordinate, in mm
                # right_target_base_position = right_arm.transform_to_base_position([300, 150, -200]) # in base coordinate, in mm
                print("right_target_base_position:", right_target_base_position)
                # print("right_target_base_position:", right_target_base_position)
                time.sleep(3)
                trajectory_planning(ox, oy, oz, right_target_base_position[0], right_target_base_position[1], right_target_base_position[2], "right", single_arm=right_arm)


                    
                time.sleep(3)
                
            else:
                print(f"偵測到文字輸入：\"{s}\"")
                # 在此處可呼叫文字處理函式，例如對話或指令解析
                # handle_text_command(s)
    except KeyboardInterrupt:
        print("捕捉到 KeyboardInterrupt，結束程式")
     


    
# target_position_world = np.array([600, 0, -300]) # in world coordinate, in mm
# target_orientation = np.array([-150, 0, 0]) # in degree



    


