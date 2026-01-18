import pybullet as pb
import numpy as np
import ArmInfo
import time
from math import *
import pybullet_data
import rospy
from std_msgs.msg import Int16MultiArray
import threading

# import scipy.linalg


# np.set_printoptions(precision= 2)

physicsClient = pb.connect(pb.GUI,  options='--background_color_red=0.0 --background_color_green=0.66 --background_color_blue=0.66')
# joe = pb.loadURDF("Joe_URDF/robot_2arm/urdf/robot244.urdf", useFixedBase= 1)
StartPos = [0, 0, 1]
StartOrientation = pb.getQuaternionFromEuler([pi/2, 0, 0])
# RobotId = pb.loadURDF('/home/aiRobots/pybullet/open_manipulator_p_description/urdf/open_manipulator_p_with_gripper.urdf', StartPos, StartOrientation,useFixedBase=True)

# RobotId = pb.loadURDF('/home/aiRobots/pybullet/單手URDF_第二張圖_第一版20250921.SLDASM/urdf/單手URDF_第二張圖_第一版20250921.SLDASM.urdf', StartPos, StartOrientation,useFixedBase=True)
RobotId = pb.loadURDF('/home/aiRobots/pybullet/單手URDF_第二張圖_第二版20250922.SLDASM/urdf/單手URDF_第二張圖_第二版20250922.SLDASM.urdf', StartPos, StartOrientation,useFixedBase=True)


pb.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = pb.loadURDF("plane.urdf")




pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)



# theta, alpha, a ,d
# single_arm_DH_table = np.array([[0,     0,      0,   159],
#                                 [pi/2, -pi/2,   0,   0],
#                                 [0,     0, 265.69,   0],
#                                 [0, -pi/2,     30,   0],
#                                 [0,  pi/2,      0, 258],
#                                 [0, -pi/2,      0,   0],
#                                 [0,     0,      0,   245.3]
#                                 ])
single_arm_DH_table = np.array([[0,                     0,       0,     159],
                                [0,                  -pi/2,       0,      0],
                                [0,                     0,     265.7,   0],
                                [0,                 -pi/2,     30,      0],
                                [0,                  pi/2,       0,      264],
                                [0,                 -pi/2,       0,      0],
                                [0,                     0,       0,     235.27]
                               
                                ])

single_arm = ArmInfo.ArmInfo(single_arm_DH_table)

# right_w_to_b = single_arm.rotation_matrix(0, -pi/2)
# single_arm.set_world_to_base_transform(0, pi/2)# 座標點不動，逆旋轉矩陣來轉換點座標
single_arm.set_world_to_base_transform(pi/2,0)# 座標點不動，逆旋轉矩陣來轉換點座標
print("world_to_base_rotation:\n", single_arm.world_to_base_transform)

# left_arm = ArmInfo.ArmInfo(left_arm_DH_table)
# right_arm = ArmInfo.ArmInfo(right_arm_DH_table)

def initialize():    
    initial_angle = np.array([0, 0, -pi/2,  0, 0, 0])
    for i in range(0, 6):
        pb.setJointMotorControl2(RobotId, jointIndex=i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition =initial_angle[i],
                                    force = 50000)
        single_arm.motor_angles[i] = pb.getJointState(RobotId, i)[0]
        # print(pb.getJointState(RobotId, i)[0])  
    time.sleep(1)    
    single_arm.set_delta_angle(np.array([0, 0, -pi/2, 0, 0, 0]) )
    
    for _ in range(1000):
        pb.stepSimulation()
    
    print("Pb1:{}".format(single_arm.Pb1))
    print("Pb2:{}".format(single_arm.Pb2))
    print("Pb3:{}".format(single_arm.Pb3))
    # print("Tb2:{}".format(single_arm.Tb1))
    # print("T34:{}".format(single_arm.T34))
    print("Pb4:{}".format(single_arm.Pb4))
    single_arm.update()
    
    link_index=5
    state = pb.getLinkState(RobotId, link_index)
    pos = state[4]
    print("End-effector position:", [pos[0]*1000, pos[1]*1000, pos[2]*1000-1000])
    # TESTING
    print("===== Testing Jacobian Matrix =====")
    # single_arm.delta_angle = np.array([0, -pi/2, 0, 0, 0, 0]) 
    # print("single_arm.delta_angle",single_arm.delta_angle)
    for i in range(0, 6):
        pb.setJointMotorControl2(RobotId, jointIndex=i,
                                    controlMode=pb.POSITION_CONTROL,
                                    targetPosition =initial_angle[i]+[0, 0, 0, 0, 0, 0][i],
                                    force = 50000)
    for _ in range(1000):
        pb.stepSimulation()
    time.sleep(1)
    for i in range(0, 6):
        # print(pb.getJointState(RobotId, i)[0])        
        single_arm.set_motor_angles(i, pb.getJointState(RobotId, i)[0] )
        # single_arm.delta_angle[i] = single_arm.motor_angles[i]                              
    single_arm.set_delta_angle( single_arm.motor_angles )
    single_arm.update()
    print("self.jocobian_matrix = {}".format(np.around(single_arm.jacobian_matrix, 2)))
    # zero = [0] * len(single_arm.motor_angles)
    
    # jac_t, jac_r = pb.calculateJacobian(RobotId, 5, [0, 0, 0],
    #                                     single_arm.motor_angles.tolist(),
    #                                     zero,
    #                                     zero)
    # print("jac_t = {}".format(np.around(jac_t, 2)))
    # print("jac_r = {}".format(np.around(jac_r, 2)))
    print("Pb1:{}".format(single_arm.Pb1))
    print("Pb2:{}".format(single_arm.Pb2))
    print("Pb3:{}".format(single_arm.Pb3))
    # print("Tb2:{}".format(single_arm.Tb1))
    # print("T34:{}".format(single_arm.T34))
    print("Pb4:{}".format(single_arm.Pb4))
    print("Current Position = {}".format(np.around(single_arm.current_position, 0)))
    print("Current Orientation = {}".format(np.around(single_arm.get_current_orientation_in_degree(), 0)))
    print("==============================")
    link_index=5
    state = pb.getLinkState(RobotId, link_index)
    pos = state[4]
    orn = state[5]

    print("End-effector position:", [pos[0]*1000, pos[1]*1000, pos[2]*1000-1000])
    print("End-effector orientation (quaternion):", orn)
    # print(np.around(single_arm.current_position[1], 0))
    
    # print("box position:", [pos[0], pos[1], pos[2]])
    visual_shape_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    # print(f"right_w_to_b:{right_w_to_b}")
    # print(single_arm.current_position)
    # matrix= np.matmul(right_w_to_b,[single_arm.current_position[0], single_arm.current_position[1], single_arm.current_position[2],1])
    box_position = single_arm.transform_to_world_position(single_arm.current_position)

    box_id = pb.createMultiBody(baseMass=0,
                            baseVisualShapeIndex=visual_shape_id,
                            basePosition=[box_position[0]/1000, box_position[1]/1000, box_position[2]/1000+1],
                            baseOrientation=[0, 0, 0, 1])
    print("box position:", [box_position[0], box_position[1], box_position[2]])
    time.sleep(5)
    # box_id = pb.createMultiBody(baseMass=1,
    #                        baseCollisionShapeIndex=box_collision,
    #                        basePosition=[0,0,0.5],
    #                        baseOrientation=[0, 0, 0])
    # for i in range(1, 7):
    #     pb.setJointMotorControl2(RobotId, jointIndex=i,
    #                             controlMode = pb.VELOCITY_CONTROL,
    #                             targetVelocity = 0,
    #                             force=50000)
    

def trajectory_planning(ox, oy, oz, px, py, pz):
    velocity_factor = 1.2
    
    angular_threshold = pi/180
    linear_threshold = 1 # in mm
    stop_counter = 0
    acceleration_factor = 0.0
    acceleration_counter = 0
    
    single_arm.set_target_orientation(ox, oy, oz)# single_arm.target_orientation in radian

    print("Target Orientation = {}".format(np.around(single_arm.target_orientation )))
    
    print("single_arm.current_orientation = {}".format(np.around(single_arm.current_orientation, 2)))
    angular_error = sqrt(pow(single_arm.target_orientation[0] - single_arm.current_orientation[0], 2) +
                        pow(single_arm.target_orientation[1] - single_arm.current_orientation[1], 2) +
                        pow(single_arm.target_orientation[2] - single_arm.current_orientation[2], 2)) / 3
    print("angular error = {}".format(angular_error))
    print("Target Position = [{}, {}, {}]".format(px, py, pz))
    print("single_arm.current_position = {}".format(np.around(single_arm.current_position, 2)))
    linear_error = sqrt(pow(px - single_arm.current_position[0], 2) +
                        pow(py - single_arm.current_position[1], 2) +
                        pow(pz - single_arm.current_position[2], 2)) /3
    print("linear error = {}".format(linear_error))
    
    while angular_error > angular_threshold or linear_error > linear_threshold :
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        
        # update error
        angular_error = sqrt(pow(single_arm.target_orientation[0] - single_arm.current_orientation[0], 2) +
                            pow(single_arm.target_orientation[1] - single_arm.current_orientation[1], 2) +
                             pow(single_arm.target_orientation[2] - single_arm.current_orientation[2], 2)) / 3
    
        linear_error = sqrt(pow(px - single_arm.current_position[0], 2) +
                             pow(py - single_arm.current_position[1], 2) +
                            pow(pz - single_arm.current_position[2], 2)) / 3
        print("=====================================================================")
        print("single_arm.current_orientation = {}".format(np.around(single_arm.current_orientation, 2)))
        print("Target Orientation = {}".format(np.around(single_arm.target_orientation )))
        print("single_arm.current_position = {}".format(np.around(single_arm.current_position, 2)))
        print("Target Position = [{}, {}, {}]".format(px, py, pz))
        print("angular error = {}".format(angular_error))
        print("linear error = {}".format(linear_error))
        print("=====================================================================")
        
        if (acceleration_factor < 1.0 and acceleration_counter % 20 == 0):
            acceleration_factor += 0.1
            

        acceleration_counter+=1
        # if acceleration_counter == 2:
        #     break
        # calculate linear velocity
        vel_ox = (single_arm.target_orientation[0] - single_arm.current_orientation[0]) * velocity_factor 
        vel_oy = (single_arm.target_orientation[1] - single_arm.current_orientation[1]) * velocity_factor 
        vel_oz = (single_arm.target_orientation[2] - single_arm.current_orientation[2]) * velocity_factor 

        vel_px = (px - single_arm.current_position[0]) * 1.5
        vel_py = (py - single_arm.current_position[1]) * 1.5
        vel_pz = (pz - single_arm.current_position[2]) * 1.5
        # print(single_arm.current_orientation[0])
        print("vel_ox={}, vel_oy={}, vel_oz={}".format(vel_ox, vel_oy, vel_oz))
        print("vel_px={}, vel_py={}, vel_pz={}".format(vel_px, vel_py, vel_pz))
        
        linear_vel = np.array((vel_ox, vel_oy, vel_oz, vel_px, vel_py, vel_pz))
        # print("linear velocity = {}".format(np.around(linear_vel, 2)))
        # angular_vel = np.matmul(left_arm.inverse_jacobian, linear_vel)
        
        single_arm.set_joint_velocity(linear_vel)
        
        single_arm.check_motor_limit()
        
        if (abs(single_arm.joint_velocity[0]) <= e-16 and abs(single_arm.joint_velocity[1]) <= e-16 and
            abs(single_arm.joint_velocity[2]) <= e-16 and abs(single_arm.joint_velocity[3]) <= e-16 and
            abs(single_arm.joint_velocity[4]) <= e-16 and abs(single_arm.joint_velocity[5]) <= e-16):


                if (stop_counter < 25):
                        stop_counter+=1
                        print("stop counter = {}".format(stop_counter))
                else:

                    break; 
                                    
        for i in range(0, 6):
            pb.setJointMotorControl2(RobotId, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = single_arm.joint_velocity[i],
                                        force = 1500)
        # for _ in range(1000):
        #     pb.stepSimulation()
            # single_arm.motor_angles[i] = pb.getJointState(RobotId, i)[0] 
            
            # pb.stepSimulation()
        # time.sleep(4)

        time.sleep(1)    
       
        # update arm state
        
        # joint_states = pb.getJointStates(RobotId, [1, 2, 3, 4, 5, 6])
        
        # for _ in range(1000):
        #     pb.stepSimulation()
        
        for i in range(0, 6):
            
            # print("Joint {} angle = {}".format(i, pb.getJointState(RobotId, i)[0]))
            single_arm.set_motor_angles(i, pb.getJointState(RobotId, i)[0])              
        
        single_arm.set_delta_angle( single_arm.motor_angles )
        # print("motor angles = {}".format(np.around(single_arm.motor_angles, 2)))    
        # print("delta angle = {}".format(np.around(single_arm.delta_angle, 2)))
        single_arm.update()
        print("current Position = {}".format(single_arm.current_position))
        stop_all_motors()


   
    # print(np.around(single_arm.joint_velocity, 2))
def stop_all_motors():
    for i in range(0, 6):
            pb.setJointMotorControl2(RobotId, jointIndex=i,
                                        controlMode = pb.VELOCITY_CONTROL,
                                        targetVelocity = 0,
                                        force = 50000)
def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])

def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)
def stepToward( ox_deg, oy_deg, oz_deg, px, py, pz):
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

    print("Target Position = [{}, {}, {}]".format(px, py, pz))
    print("single_arm.current_position = {}".format(np.around(single_arm.current_position, 2)))
    last_position = single_arm.current_position
    single_arm.update()
    # target position
    # target_p = np.array([, py, pz], dtype=np.float32)
    # set up target position
   
    ## workspace limit
    target_distance = np.linalg.norm(single_arm.target_position)
    max_reach = 755.0

    if target_distance > max_reach:
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(f"Warning: Target ({px:.1f}, {py:.1f}, {pz:.1f}) may be out of reach (distance: {target_distance:.1f}mm > {max_reach}mm)")
        scale = (max_reach - 10) / target_distance
        # target_p = target_p * scale

        single_arm.set_target_position(single_arm.target_position[0]*scale, single_arm.target_position[1]*scale, single_arm.target_position[2]*scale)
        print(f"Adjusted target to: ({single_arm.target_position[0]:.1f}, {single_arm.target_position[1]:.1f}, {single_arm.target_position[2]:.1f})")
    
    # # set up angular error 
    # q_target = pb.getQuaternionFromEuler([float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])])
    # q_current = pb.getQuaternionFromEuler([float(single_arm.current_orientation[0]),
    #                                         float(single_arm.current_orientation[1]),
    #                                         float(single_arm.current_orientation[2])])

    # q_err = quat_mul(q_target, quat_conjugate(q_current))
    # qw = max(min(q_err[3], 1.0), -1.0)
    # angle = 2.0 * acos(qw)
    # if abs(angle) < 1e-6:
    #     axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    # else:
    #     s = sqrt(1.0 - qw*qw)
    #     if s < 1e-8:
    #         axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    #     else:
    #         axis = np.array([q_err[0]/s, q_err[1]/s, q_err[2]/s], dtype=np.float32)
    # error_ang_vec = axis * angle
    error_ang_vec = single_arm.target_orientation - single_arm.current_orientation
    angular_error_norm = np.linalg.norm(error_ang_vec)
    # ang_err_norm = np.linalg.norm(error_ang_vec)
    # set up linear error
    error_pos = single_arm.target_position - single_arm.current_position
    linear_error_norm = np.linalg.norm(error_pos)

    # check if reached
    if linear_error_norm < linear_threshold and angular_error_norm < angular_threshold:
        stop_all_motors()
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
    for i in range(0, 6):
        pb.setJointMotorControl2(RobotId, jointIndex=i,
                                    controlMode = pb.VELOCITY_CONTROL,
                                    targetVelocity = float(single_arm.joint_velocity[i]),
                                    force = 1500)
    for i in range(0, 6):
        single_arm.set_motor_angles(i, pb.getJointState(RobotId, i)[0])              
    
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

def trajectory_planning_step(ox, oy, oz, px, py, pz):
    start_time = time.time()
    timeout = 30  # seconds
    iteration = 0
    # trajectory_planning(ox, oy, oz, px, py, pz)
    while time.time() - start_time < timeout:
            iteration += 1
            
            if stepToward(ox, oy, oz, px, py, pz):
                single_arm.update()
                error = np.linalg.norm(np.array([px, py, pz]) - single_arm.current_position)
                print(f"Target reached! distance error: {error:.2f} mm (iter: {iteration})")
                print(f"Final position: ({single_arm.current_position[0]:.1f}, {single_arm.current_position[1]:.1f}, {single_arm.current_position[2]:.1f}) mm")
                stop_all_motors()
                return True
            time.sleep(single_arm.dt_)
# def adaptive():
#     while True:
#         # print("{}\t{}\t{}".format(fx, fy, fz))
#         # update arm state
#         single_arm.delta_angle = np.array(pb.getJointStates(RobotId, [1, 2, 3, 4, 5, 6]))[0:6, 0]
#         single_arm.update()

#         vx = 0
#         vy = 0
#         vz = 0
        
#         if abs(fx) < 50:
#             vz = 0
#         else:
#             vz = -fx / 5

#         if abs(fy) < 50:
#             vy = 0
#         else:
#             vy = fy / 5

#         if abs(fz) < 50:
#             vx = 0
#         else:
#             vx = fz / 5


#         linear_vel = np.array((0, 0, 0, vx, vy, vz))
#         single_arm.set_angular_velocity(linear_vel)
#         # angular_vel = np.matmul(single_arm.inverse_jacobian, linear_vel)

#         print(np.around(single_arm.angular_velocity, 2))

#         # check velocity limit
#         single_arm.check_motor_limit()
#         # max_index = np.argmax(angular_vel)
#         # max_value = np.max(angular_vel)

#         # if abs(max_value) > pi:
#         #     print("\tDangerous ! Stop moving.")
#         #     break
#         # elif abs(max_value) > pi/6:
#         #     angular_vel *= (pi / 6 / abs(max_value))
        
#         # render in simulation
#         for i in range(1, 7):
#             pb.setJointMotorControl2(RobotId, jointIndex=i,
#                                         controlMode = pb.VELOCITY_CONTROL,
#                                         targetVelocity = single_arm.angular_velocity[i-1],
#                                         force = 500)
        # for i in range(16):
        #     if i == 0 or i == 7 or i == 15:
        #         continue

        #     elif i >= 9:
        #         pb.setJointMotorControl2(RobotId, jointIndex=i,
        #                                     controlMode = pb.VELOCITY_CONTROL,
        #                                     targetVelocity = angular_vel[i-9],
        #                                     force = 50000)

        # time.sleep(0.8)

    # for i in range(16):
    #     pb.setJointMotorControl2(RobotId, jointIndex=i,
    #                                     controlMode = pb.VELOCITY_CONTROL,
    #                                     targetVelocity = 0,
    #                                     force = 500)

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

def listener():    
    rospy.Subscriber("ForseSensor_C2Python", Int16MultiArray, callback)
    rospy.spin()

rospy.init_node("aiRobots_Python", anonymous=True)
sub_td = threading.Thread(target=listener)
sub_td.start()


# worspace_data=collect_workspace_data()
# print("Workspace data collected:", worspace_data[1])
initialize()
time.sleep(2)
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
# target_position_world = np.array([390, 250, 60]) # in world coordinate, in mm
target_position_world = np.array([450, -100, -100])
target_base_position = single_arm.transform_to_base_position(target_position_world) # in base coordinate, in mm
print

# trajectory_planning(-180, 0, 0, target_place_position[0], target_place_position[1], target_place_position[2])
trajectory_planning_step(-170, 0, 0, target_base_position[0], target_base_position[1], target_base_position[2])
print("reach point 1")
visual_shape_id2 = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
box_id = pb.createMultiBody(baseMass=0,
                            baseVisualShapeIndex=visual_shape_id2,
                            basePosition=[target_position_world[0]/1000, target_position_world[1]/1000, target_position_world[2]/1000+1],
                            baseOrientation=[0, 0, 0, 1])
print("Current Position = {}".format(np.around(single_arm.current_position, 1)))
print("Current Orientation = {}".format(np.around(single_arm.get_current_orientation_in_degree(), 1)))
# trajectory_planning(-180, 0, 0, target_place_position[0], target_place_position[1], target_place_position[2])
