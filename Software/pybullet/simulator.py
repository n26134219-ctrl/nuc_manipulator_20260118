import pybullet as pb
import numpy as np
import ArmInfo
import Robot
from math import *
import pybullet_data
# import rospy
# from std_msgs.msg import Int16MultiArray
import threading
# import sys
# sys.path.append('/home/airobots/gpt-oss/GroundingDINO')
import time
# import camera_class 
# import rospy
# from std_msgs.msg import Float64MultiArray
import numpy as np
from math import *
import numpy as np
from scipy.spatial.transform import Rotation as R


# set up simulation
physicsClient = pb.connect(pb.GUI,  options='--background_color_red=0.0 --background_color_green=0.66 --background_color_blue=0.66')
chest_position = 1.1
StartPos = [0, 0, chest_position]
StartOrientation = pb.getQuaternionFromEuler([0, 0, 0])
## loading robot urdf
RobotId = pb.loadURDF('/home/aiRobots/Software/pybullet/全身URDF_第二版20250923.SLDASM/urdf/全身URDF_第二版20250923.SLDASM.urdf', StartPos, StartOrientation, useFixedBase=True)

## loading plane urdf
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF("plane.urdf")

## object
left_box_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01], rgbaColor=[0, 0, 0, 1])  # 黑色，最後一個是透明度)
right_box_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01], rgbaColor=[1, 0, 0, 1])  # 灰白色，最後一個是透明度)
object_id_list=[]


## set gravity and real time simulation
pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)
pb_lock = threading.Lock()

# set up robot
Nova_Robot = Robot.Robot()
# update arm state function


# pub = rospy.Publisher('delta_angle_topic', Float64MultiArray, queue_size=10)

# def publish_delta_angle():
        
#         msg = Float64MultiArray()
#         msg.data = Nova_Robot.left_arm.delta_angle.tolist()  # 將 numpy array 轉為 list
#         pub.publish(msg)

def update_Arm_state():
    for i in range(0, 6):
        Nova_Robot.left_arm.set_motor_angles(i, pb.getJointState(RobotId, i)[0])
        Nova_Robot.right_arm.set_motor_angles(i, pb.getJointState(RobotId, i+6)[0])

    Nova_Robot.left_arm.set_delta_angle(Nova_Robot.left_arm.motor_angles)
    print("left_arm.delta_angle:", Nova_Robot.left_arm.delta_angle)


    Nova_Robot.right_arm.set_delta_angle(Nova_Robot.right_arm.motor_angles )
    print("right_arm.delta_angle:", Nova_Robot.right_arm.delta_angle)
    Nova_Robot.update_robot_pose()
    # publish_delta_angle()



def initialize():   
    print("Initialize robot pose...")
    right_ini_ang=np.array([-3.47893164, -2.18037179,  3.41286134,  3.14146773,  1.23196564,  2.8003281])
    # intial robot pose
    for i in range(0, 6):
        pb.setJointMotorControl2(RobotId, jointIndex = i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = Nova_Robot.initial_angle[i],
                                    force = 5000)
        pb.setJointMotorControl2(RobotId, jointIndex = i + 6,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition = right_ini_ang[i],
                                    force = 5000)
    time.sleep(2)
    for _ in range(3000):
        pb.stepSimulation()   
    update_Arm_state()
    print("robot pose initialized.")
    print("===================================")
    print("Current left Position = {}".format(np.around(Nova_Robot.left_world_pos, 0)))
    print("Current left Orientation = {}".format(np.around(Nova_Robot.left_orientation_deg, 0)))
    print("===================================")
    print("===================================")
    print("Current right Position = {}".format(np.around(Nova_Robot.right_world_pos, 0)))
    print("Current right Orientation = {}".format(np.around(Nova_Robot.right_orientation_deg, 0)))
    print("===================================")
    left_box_position = Nova_Robot.left_world_pos
    left_initial = pb.createMultiBody(baseMass=0,
                            baseVisualShapeIndex=left_box_id,
                            basePosition=[left_box_position[0]/1000, left_box_position[1]/1000, left_box_position[2]/1000 + chest_position],
                            baseOrientation=[0, 0, 0, 1])
    
    # print("left box position:", [left_box_position[0], left_box_position[1], left_box_position[2]])

    right_box_position = Nova_Robot.right_world_pos

    right_initial = pb.createMultiBody(baseMass=0,
                            baseVisualShapeIndex=right_box_id,
                            basePosition=[right_box_position[0]/1000, right_box_position[1]/1000, right_box_position[2]/1000 + chest_position],
                            baseOrientation=[0, 0, 0, 1])
    
    # print("right box position:", [right_box_position[0], right_box_position[1], right_box_position[2]])
    time.sleep(1)
    
    # separateMove( 370, 100, -130,  "side", 370,  -100, -130, "side", 0, 0)


    return left_initial, right_initial


def stop_all_motors(mode):
    if mode == "left":
        for i in range(0, 6):
                pb.setJointMotorControl2(RobotId, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = 0,
                                            force = 5000)
    elif mode == "right":
        for i in range(6, 12):
                pb.setJointMotorControl2(RobotId, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = 0,
                                            force = 5000)
    elif mode == "both":
        for i in range(0, 12):
                pb.setJointMotorControl2(RobotId, jointIndex=i,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocity = 0,
                                            force = 5000)
    else:
        print("mode error in stop_all_motors()")
    for _ in range(1500):
        pb.stepSimulation() 

def step_move(ox_deg, oy_deg, oz_deg, px, py, pz, mode): 
    '''
    ox_deg, oy_deg, oz_deg: orientation in degree
    px, py, pz: base position in mm
    mode: "left" or "right"
    return: Finished (bool)
    '''
    
    if mode == "left":
        single_arm = Nova_Robot.left_arm
    elif mode == "right":
        single_arm = Nova_Robot.right_arm

    Finished = Nova_Robot.step_IK(ox_deg, oy_deg, oz_deg, px, py, pz,  single_arm)
    with pb_lock:
        for i in range(single_arm.start_id, single_arm.end_id):
            pb.setJointMotorControl2(RobotId, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = float(single_arm.joint_velocity[i-single_arm.start_id]),
                                        force = 1500)
    # pb.setJointMotorControl2(RobotId, jointIndex=0,
    #                                     controlMode = pb.VELOCITY_CONTROL,
    #                                     targetVelocity = float(0.001),
    #                                     force = 1500)
    update_Arm_state()
    time.sleep(0.1) # 100ms
    # time.sleep(0.01) # 10ms
    stop_all_motors(mode)
    # s = 0
    # s = input("請輸入:").strip()
    return Finished
   
def step_move_acc(ox_deg, oy_deg, oz_deg, px, py, pz, mode,acceleration_factor=0.1): 
    '''
    ox_deg, oy_deg, oz_deg: orientation in degree
    px, py, pz: base position in mm
    mode: "left" or "right"
    return: Finished (bool)
    '''
    
    if mode == "left":
        single_arm = Nova_Robot.left_arm
    elif mode == "right":
        single_arm = Nova_Robot.right_arm

    Finished = Nova_Robot.step_IK_acc(ox_deg, oy_deg, oz_deg, px, py, pz,  single_arm, acceleration_factor)
    with pb_lock:
        for i in range(single_arm.start_id, single_arm.end_id):
            pb.setJointMotorControl2(RobotId, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = float(single_arm.joint_velocity[i-single_arm.start_id]),
                                        force = 1500)

    update_Arm_state()
    time.sleep(0.01) # 10ms
    # stop_all_motors(mode)
    # s = 0
    # s = input("請輸入:").strip()
    return Finished
   
def trajectory_planning(ox, oy, oz, px, py, pz, mode = "left"):
    '''
    ox, oy, oz: orientation in degree
    px, py, pz: base position in mm
    mode: "left" or "right"
    '''
    if mode == "left":
        single_arm = Nova_Robot.left_arm
    elif mode == "right":
        single_arm = Nova_Robot.right_arm
    else:
        print("mode error in trajectory_planning()")
        return False
    start_time = time.time()
    timeout = 500  # seconds
    iteration = 0
    acceleration_factor = 0.5
    # trajectory_planning(ox, oy, oz, px, py, pz)
    
    while time.time() - start_time < timeout:

    # while True:
            iteration += 1
            if iteration% 20 == 0 and acceleration_factor<1.0:
                acceleration_factor += 0.1
            
                # print("acceleration_factor:", acceleration_factor)
            acceleration_factor= min(acceleration_factor, 1.00)
            # if step_move(ox, oy, oz, px, py, pz, mode):
            if step_move_acc(ox, oy, oz, px, py, pz, mode, acceleration_factor):
                update_Arm_state()

                error = np.linalg.norm(np.array([px, py, pz]) - single_arm.current_position)

                print(f"Target reached! distance error: {error:.2f} mm (iter: {iteration})")
                print(f"acceleration_factor final:", acceleration_factor)
                Nova_Robot.get_arm_information()
                stop_all_motors(mode)
                return True
            time.sleep(0.02)
    print("Timeout: Failed to reach the target within the time limit.")
    update_Arm_state()

    error = np.linalg.norm(np.array([px, py, pz]) - single_arm.current_position)

    print(f"Target reached! distance error: {error:.2f} mm (iter: {iteration})")
    Nova_Robot.get_arm_information()
    stop_all_motors(mode)


def single_arm_controll(ox, oy, oz, X, Y, Z, mode, pick_mode="side", angle=90):
    update_Arm_state()
    left_target_position_world = Nova_Robot.left_world_pos
    right_target_position_world = Nova_Robot.right_world_pos
    print("User Mode:",mode)
    if mode == "left":
        single_arm = Nova_Robot.left_arm
    
    elif mode == "right":
        single_arm = Nova_Robot.right_arm
    else:
        print("Error: mode should be 'left' or 'right'")
        exit(1)
    POS, ori, pick_mode = Nova_Robot.check_position_workspace(np.array([X, Y, Z]), np.array([ox, oy, oz]),pick_mode, mode)
    
    target_base_position = single_arm.transform_to_base_position([POS[0], POS[1], POS[2]])
    trajectory_planning(ori[0], ori[1], ori[2], target_base_position[0], target_base_position[1], target_base_position[2], mode=mode)
    print("reach point !")
    Nova_Robot.get_arm_information()
    print("Target  Position = {}".format(np.around(POS, 1)))
    print("Target  Orientation = {}".format(np.around(ori, 1)))

    
    if mode == "left":
        left_target_position_world = POS
    elif mode == "right":
        right_target_position_world = POS
    left_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=left_box_id,
                                basePosition=[left_target_position_world[0]/1000, left_target_position_world[1]/1000, left_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])

    right_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=right_box_id,
                                basePosition=[right_target_position_world[0]/1000, right_target_position_world[1]/1000, right_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])
    return left_target_box, right_target_box

def both_arm_mode_controll( X, Y, Z, pick_mode, angle=90):
    update_Arm_state() 

    POS, ori, ptype = Nova_Robot.check_position_workspace(np.array([X, Y, Z]), np.array([-180, 0, 0]),"side", "left")

    POS[1] = 0
    target_left_orientation = np.array([-180, 0, 0])
    target_right_orientation = np.array([0, 180, 0])

    left_target_position_world = np.array([POS[0], POS[1], POS[2] + Nova_Robot.z_offset])
    right_target_position_world = np.array([POS[0], POS[1], POS[2] ])

    left_target_base_position = Nova_Robot.left_arm.transform_to_base_position(left_target_position_world)
    right_target_base_position = Nova_Robot.right_arm.transform_to_base_position(right_target_position_world)

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
    }
    
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
    }

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
    print("reach point !")
    Nova_Robot.get_arm_information()
    print("Target  Position = {}".format(np.around(POS, 1)))

    left_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=left_box_id,
                                basePosition=[left_target_position_world[0]/1000, left_target_position_world[1]/1000, left_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])

    right_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=right_box_id,
                                basePosition=[right_target_position_world[0]/1000, right_target_position_world[1]/1000, right_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])
    return left_target_box, right_target_box


def separateMove( LX, LY, LZ,  lpick_mode,RX, RY, RZ, rpick_mode, L_angle=0, R_angle=0):
    left_target_position_world = Nova_Robot.left_world_pos
    right_target_position_world = Nova_Robot.right_world_pos

    Lori, lpick_mode = Nova_Robot.get_picktype_pose( arm="left", ptype=lpick_mode,angle=L_angle)
    Rori, rpick_mode = Nova_Robot.get_picktype_pose( arm="right", ptype=rpick_mode,angle=R_angle)

    left_target_position_world, Lori, lpick_mode = Nova_Robot.check_position_workspace(np.array([LX, LY, LZ]), np.array([Lori[0], Lori[1], Lori[2]]),"side", "left")
    right_target_position_world, Rori, rpick_mode = Nova_Robot.check_position_workspace(np.array([RX, RY, RZ]), np.array([Rori[0], Rori[1], Rori[2]]),"side", "right")


    left_target_base_position = Nova_Robot.left_arm.transform_to_base_position(left_target_position_world)
    right_target_base_position = Nova_Robot.right_arm.transform_to_base_position(right_target_position_world)


    left_args = (
        Lori[0],
        Lori[1],
        Lori[2],
        left_target_base_position[0],
        left_target_base_position[1],
        left_target_base_position[2],
    )
    left_kwargs = {
        'mode': 'left',
    }
    
    #  right
    right_args = (
        Rori[0],
        Rori[1],
        Rori[2],
        right_target_base_position[0],
        right_target_base_position[1],
        right_target_base_position[2],
    )
    right_kwargs = {
        'mode': 'right',
    }

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
    print("reach point !")
    Nova_Robot.get_arm_information()
    # print("Target  Position = {}".format(np.around(POS, 1)))

    left_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=left_box_id,
                                basePosition=[left_target_position_world[0]/1000, left_target_position_world[1]/1000, left_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])

    right_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=right_box_id,
                                basePosition=[right_target_position_world[0]/1000, right_target_position_world[1]/1000, right_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])
    

    return left_target_box, right_target_box

def synchronizeMove( X, Y, Z, pick_mode, angle=0, object_width=150):
    update_Arm_state()
    if abs(X) < 530 and abs(X) > 200:
            if Y < 260 and Y > -260:
                print("in table workspace")  
                if Z <= -340:
                    Z = -330
            else:
                print("out of table workspace")
                exit(1)
    else:
        print("out of workspace")
        exit(1)

    left_target_position_world = np.array([X, Y + object_width/2, Z])
    right_target_position_world = np.array([X, Y - object_width/2, Z])

    left_target_base_position = Nova_Robot.left_arm.transform_to_base_position(left_target_position_world)
    right_target_base_position = Nova_Robot.right_arm.transform_to_base_position(right_target_position_world)

    # target_left_orientation = np.array([-180, 0, 0])
    # target_right_orientation = np.array([0, 180, 0])
    target_left_orientation, lpick_mode = Nova_Robot.get_picktype_pose( arm="left", ptype=pick_mode,angle=angle)
    target_right_orientation, rpick_mode = Nova_Robot.get_picktype_pose( arm="right", ptype=pick_mode,angle=angle)


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
    }
    
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
    }

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
    print("reach point !")
    Nova_Robot.get_arm_information()
    print("Target  Position = {}".format(np.around([X, Y, Z], 1)))
    left_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=left_box_id,
                                basePosition=[left_target_position_world[0]/1000, left_target_position_world[1]/1000, left_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])

    right_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=right_box_id,
                                basePosition=[right_target_position_world[0]/1000, right_target_position_world[1]/1000, right_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])
    return left_target_box, right_target_box

def singleArmMove(X, Y, Z, arm, pick_mode="side", angle=0):
    update_Arm_state()
    orientation, pick_mode = Nova_Robot.get_picktype_pose( arm=arm, ptype=pick_mode,angle=angle)

    left_target_position_world = Nova_Robot.left_world_pos
    right_target_position_world = Nova_Robot.right_world_pos
    print("User Mode:", arm)
    if arm == "left":
        single_arm = Nova_Robot.left_arm
    
    elif arm == "right":
        single_arm = Nova_Robot.right_arm
    else:
        print("Error: mode should be 'left' or 'right'")
        exit(1)
    POS, ori, pick_mode = Nova_Robot.check_position_workspace(np.array([X, Y, Z]), np.array([orientation[0], orientation[1], orientation[2]]),pick_mode, arm)
    
    target_base_position = single_arm.transform_to_base_position([POS[0], POS[1], POS[2]])
    trajectory_planning(ori[0], ori[1], ori[2], target_base_position[0], target_base_position[1], target_base_position[2], mode=arm)
    print("reach point !")
    Nova_Robot.get_arm_information()
    print("Target  Position = {}".format(np.around(POS, 1)))
    print("Target  Orientation = {}".format(np.around(ori, 1)))

    
    if arm == "left":
        left_target_position_world = POS
    elif arm == "right":
        right_target_position_world = POS
    left_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=left_box_id,
                                basePosition=[left_target_position_world[0]/1000, left_target_position_world[1]/1000, left_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])

    right_target_box = pb.createMultiBody(baseMass=0,
                                baseVisualShapeIndex=right_box_id,
                                basePosition=[right_target_position_world[0]/1000, right_target_position_world[1]/1000, right_target_position_world[2]/1000 + chest_position],
                                baseOrientation=[0, 0, 0, 1])
    return left_target_box, right_target_box

def mode_controll( X, Y, Z, mode, pick_mode, angle=90, object_width=150):

    if pick_mode not in ["side", "down", "forward", "reversal"]:
        print("Error: pick_mode should be 'side', 'down' or 'forward' or 'reversal'")
        exit(1)

    
    if (left_initial is not None) and (right_initial is not None):
        try:
            
            pb.removeBody(left_initial)
            pb.removeBody(right_initial)

        except pb.error:
            # 物體已經不存在，忽略錯誤
            pass
    if mode == "left" or mode == "right":
        orientation, pick_mode = Nova_Robot.get_picktype_pose( arm=mode, ptype=pick_mode,angle=angle)
        left_target_box, right_target_box = single_arm_controll(orientation[0], orientation[1], orientation[2], X, Y, Z, mode, pick_mode, angle)
    elif mode == "both":
        left_target_box, right_target_box = both_arm_mode_controll( X, Y, Z, pick_mode, angle)
    elif mode == "sync_move":
        left_target_box, right_target_box = synchronizeMove( X, Y, Z, pick_mode, angle, object_width)
    else:
        print("Error: mode should be 'left', 'right', 'both' or 'sync_move'")
        exit(1)
    return left_target_box, right_target_box




def add_object(X, Y, Z, size_x=20, size_y=20, size_z=20, angle=0):
    '''
    X, Y, Z: position in mm(in robot world coordinate)
    size_x, size_y, size_z: size in mm
    angle: rotation angle in degree (form camera view)
    0 degree: long side parallel to Y axis
    90 degree: long side parallel to X axis
    
    '''
    global object_id_list
    length = len(object_id_list)

    object_box_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[size_x/1000, size_y/1000, size_z/1000], rgbaColor=[0, 1, 0, 1])  # 綠色，最後一個是透明度)
    
    object_position = [X/1000, Y/1000, Z/1000 + chest_position]
    object_orientation = pb.getQuaternionFromEuler([0, 0, (180-angle)/180*pi])
    object_id = pb.createMultiBody(baseMass=0.1,
                            baseVisualShapeIndex=object_box_id,
                            basePosition=object_position,
                            baseOrientation=object_orientation)
    object_id_list.append(object_id)
    




def remove_object(idex):
    pb.removeBody(object_id_list[idex])
    object_id_list.pop(idex)




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


# def single_test():
#     left_initial, right_initial = initialize()
#     left_target_box = None
#     right_target_box = None
#     angle = 0
#     try:
#         while True:
#             # m = input("請選擇模式left, right, both, same, exit（輸入 l, r, b, s, e ）：").strip().lower()
            
#             m = input("請選擇模式left, right, exit（輸入 l, r, b, s, e ）：").strip().lower()
#             if m == 'l':
#                 mode = 'left'
#                 single_arm = Nova_Robot.left_arm
#             elif m == 'r':
#                 mode = 'right'
#                 single_arm = Nova_Robot.right_arm
#             if m == 'e':
#                 print("程式結束")
#                 pb.disconnect(physicsClientId=physicsClient)
#                 break

#             s = input("請輸入文字或 ox,oy,oz 座標（輸入 e 離開）：").strip()
#             if s.lower() == 'e':
#                 print("程式結束")
#                 pb.disconnect(physicsClientId=physicsClient)
#                 break
        
#             if is_coordinate_input(s):
#                 ox, oy, oz = parse_coordinates(s)
#                 print(f"偵測到座標輸入：ox={ox}, oy={oy}, oz={oz}")
#                 p = input("請輸入文字或 x,y,z 座標（輸入 e 離開）：").strip()
#                 if p.lower() == 'e':
#                     print("程式結束")
#                     pb.disconnect(physicsClientId=physicsClient)
#                     break
            
#                 if is_coordinate_input(p):
#                     X, Y, Z = parse_coordinates(p)
#                     print(f"偵測到座標輸入：x={X}, y={Y}, z={Z}")
#                     pos = np.array([X, Y, Z])
#                     pos, ori, ptype = Nova_Robot.check_position_workspace(np.array([X, Y, Z]), np.array([ox, oy, oz]),"side", mode)
#                     target_base_position = single_arm.transform_to_base_position([pos[0], pos[1], pos[2]]) # in base coordinate, in mm
#                 time.sleep(3)
                
#                 trajectory_planning(ori[0], ori[1], ori[2], target_base_position[0], target_base_position[1], target_base_position[2], mode=mode)
#                 time.sleep(3)
                
#             else:
#                 print(f"偵測到文字輸入：\"{s}\"")
#                 # 在此處可呼叫文字處理函式，例如對話或指令解析
#                 # handle_text_command(s)
#     except KeyboardInterrupt:
#         print("捕捉到 KeyboardInterrupt，結束程式")
# if __name__ == "__main__":
#     left_initial, right_initial = initialize()
#     left_target_box = None
#     right_target_box = None
#     angle = 0
#     # rospy.init_node('delta_angle_publisher')
#     # rate = rospy.Rate(10)  # 10Hz
    
#     try:
#         while True:
#             # m = input("請選擇模式left, right, both, same, exit（輸入 l, r, b, s, e ）：").strip().lower()
            
#             m = input("請選擇模式left, right, both, same, exit（輸入 l, r, b, s, e ）：").strip().lower()
#             if m == 'l':
#                 mode = 'left'
#             elif m == 'r':
#                 mode = 'right'
#             elif m == 'b':
#                 mode = 'both'
#             elif m == 's':
#                 mode = 'same_move'
#             if m == 'e':
#                 print("程式結束")
#                 pb.disconnect(physicsClientId=physicsClient)
#                 break

#             s = input("請輸入pick_mode 輸入s,d,f, r（side, down, forward, reversal）（輸入 e 離開）：").strip()
#             if s.lower() == 'e':
#                 print("程式結束")
#                 pb.disconnect(physicsClientId=physicsClient)
#                 break

#             if s not in ["s", "d", "f"]:
#                 print("Error: pick_mode should be 's', 'd' or 'f' or 'r'")
#                 continue

#             pick_mode = {"s": "side", "d": "down", "f": "forward", "r": "reversal"}[s]
#             print(f"選擇的 pick_mode 為: {pick_mode}")

#             s = input("請輸入偏移量（輸入 e 離開）：").strip()
#             if s.lower() == 'e':
#                 print("程式結束")
#                 pb.disconnect(physicsClientId=physicsClient)
#                 break
#             if s.isdigit():
#                 angle = int(s)
#                 if angle < 0 or angle > 180:
#                     print("Error: angle should be between 0 and 180")
#                     continue
#                 print(f"選擇的 偏移量 為: {angle}")
        
#             p = input("請輸入文字或 x,y,z 座標（輸入 e 離開）：").strip()
#             if p.lower() == 'e':
#                 print("程式結束")
#                 pb.disconnect(physicsClientId=physicsClient)
#                 break
            
#             if is_coordinate_input(p):
#                 X, Y, Z = parse_coordinates(p)
#                 print(f"偵測到座標輸入：x={X}, y={Y}, z={Z}")
                    
#                 # left_target_base_position = left_arm.transform_to_base_position([x, y, z ]) # in base coordinate, in mm
#                 # right_target_base_position = right_arm.transform_to_base_position([300, 150, -200]) # in base coordinate, in mm
#                 # print("left_target_base_position:", left_target_base_position)
#                 # print("right_target_base_position:", right_target_base_position)
#                 time.sleep(3)
#                 # add_object(300, 0, -330, size_x=10, size_y=60, size_z=60, angle=30)
#                 # add_object(X, Y, -330, size_x=10, size_y=60, size_z=60, angle=angle)
#                 # Nova_Robot.step_IK(3.0622, 0.558899, 2.16909, 546.599, 127.689, 167.076, Nova_Robot.left_arm)
#                 left_target_box, right_target_box = mode_controll(X, Y, Z, mode, pick_mode, angle=angle, object_width=150)
#                 # target_base_position = Nova_Robot.left_arm.transform_to_base_position([POS[0], POS[1], POS[2]])
#             # trajectory_planning(167.4, 8.65711, 129.632, 525.907, 43.3473, -40.9267, mode=mode)
            
#                 # print("mode", mode)
#                 # time.sleep(1)
#                 # # left_target_box, right_target_box = mode_controll(46.599, 127.689, 167.076, mode, pick_mode, angle=angle, object_width=150)
#                 # time.sleep(3)
                
#             else:
#                 print(f"偵測到文字輸入：\"{s}\"")
#                 # 在此處可呼叫文字處理函式，例如對話或指令解析
#                 handle_text_command(s)
#     except KeyboardInterrupt:
#         print("捕捉到 KeyboardInterrupt，結束程式")

import rospy
from std_msgs.msg import String
import json

class RobotCommandSubscriber:
    def __init__(self):
        
        
        """初始化 subscriber"""
        rospy.init_node('robot_command_subscriber', anonymous=True)
        
        # 訂閱 robot_command topic
        self.sub = rospy.Subscriber('robot_command', String, self.command_callback)
        self.left_target_box = None
        self.right_target_box = None
        rospy.loginfo("Robot Command Subscriber started, waiting for commands...")
    
    def command_callback(self, msg):
        """接收命令的 callback 函式"""
        try:
            # 解析 JSON 資料
            data = json.loads(msg.data)
            
            # 檢查 action 欄位
            action = data.get("action", "")
            if self.left_target_box is not None:
                try:
                    pb.removeBody(self.left_target_box)
                    rospy.loginfo("Removed old left_target_box")
                except pb.error:
                    pass
            
            # 刪除舊的 right_target_box（如果存在）
            if self.right_target_box is not None:
                try:
                    pb.removeBody(self.right_target_box)
                    rospy.loginfo("Removed old right_target_box")
                except pb.error:
                    pass
            if action == "synchronize_move":
                time.sleep(1)  # 等待 1 秒鐘以確保前一個動作完成
                self.handle_synchronize_move(data)

            elif action == "single_move":
                time.sleep(1)  # 等待 1 秒鐘以確保前一個動作完成
                self.handle_single_move(data)

            elif action == "dual_move":
                time.sleep(1)  # 等待 1 秒鐘以確保前一個動作完成
                self.handle_dual_move(data)

            else:
                rospy.logwarn("Unknown action: {}".format(action))
            
            if (left_initial is not None) and (right_initial is not None):
                try:
                    
                    pb.removeBody(left_initial)
                    pb.removeBody(right_initial)

                except pb.error:
                    # 物體已經不存在，忽略錯誤
                    pass    
        except json.JSONDecodeError as e:
            rospy.logerr("Failed to parse JSON: {}".format(e))
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
    
    def handle_synchronize_move(self, data):
        # global left_target_box, right_target_box
        """處理同步移動命令"""
        x = data.get("x", 0.0)
        y = data.get("y", 0.0)
        z = data.get("z", 0.0)
        pick_mode = data.get("pick_mode", "")
        object_width = data.get("object_width", 0.0)
        
        rospy.loginfo("synchronize_move: x=%.2f, y=%.2f, z=%.2f, mode=%s, width=%.2f",
                      x, y, z, pick_mode, object_width)
        
        # 在這裡呼叫你的模擬器函式
        self.left_target_box, self.right_target_box =  synchronizeMove( x, y, z, pick_mode, angle=0, object_width=object_width)
        # left_target_box, right_target_box = mode_controll(x, y, z, "both", pick_mode, angle=angle, object_width=150)
        # self.robot.synchronize_move(x, y, z, pick_mode, object_width)
        
    def handle_single_move(self, data):
        # global left_target_box, right_target_box
        """處理單一手臂移動命令"""
        arm = data.get("arm", "")
        x = data.get("x", 0.0)
        y = data.get("y", 0.0)
        z = data.get("z", 0.0)
        pick_mode = data.get("pick_mode", "")
        angle = data.get("angle", 0.0)
        
        rospy.loginfo("single_move: arm=%s, x=%.2f, y=%.2f, z=%.2f, mode=%s, angle=%.2f",
                      arm, x, y, z, pick_mode, angle)
        self.left_target_box, self.right_target_box = singleArmMove( x, y, z, arm, pick_mode, angle)
        # 在這裡呼叫你的模擬器函式
        # self.robot.single_move(arm, x, y, z, pick_mode, angle)
        
    def handle_dual_move(self, data):
        # global left_target_box, right_target_box
        """處理雙手移動命令"""
        # 左手資料
        left = data.get("left", {})
        LX = left.get("x", 0.0)
        LY = left.get("y", 0.0)
        LZ = left.get("z", 0.0)
        Lpick_mode = left.get("pick_mode", "")
        Langle = left.get("angle", 0.0)
        
        # 右手資料
        right = data.get("right", {})
        RX = right.get("x", 0.0)
        RY = right.get("y", 0.0)
        RZ = right.get("z", 0.0)
        Rpick_mode = right.get("pick_mode", "")
        Rangle = right.get("angle", 0.0)
        
        rospy.loginfo("dual_move: Left(%.2f, %.2f, %.2f) pick_mode=%s, angle=%.2f",
                      LX, LY, LZ, Lpick_mode, Langle)
        rospy.loginfo("dual_move: Right(%.2f, %.2f, %.2f) pick_mode=%s, angle=%.2f",
                      RX, RY, RZ, Rpick_mode, Rangle)
        # 執行雙手不同目標點同時移動
        self.left_target_box, self.right_target_box = separateMove( LX, LY, LZ,  Lpick_mode,RX, RY, RZ, Rpick_mode, Langle, Rangle)
        
    def spin(self):
        """保持節點運行"""
        rospy.spin()

if __name__ == '__main__':
    left_initial, right_initial = initialize()
    left_target_box = None
    right_target_box = None
    try:
        subscriber = RobotCommandSubscriber()
        subscriber.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Subscriber stopped")
        pb.disconnect(physicsClientId=physicsClient)






