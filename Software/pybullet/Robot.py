import numpy as np
import ArmInfo
import time
from math import *
import threading
from scipy.spatial.transform import Rotation as R
from math import *


class Robot(object):
    def __init__(self):
        # in terms of theta, alpha, a, d
        self.left_arm_DH_table = np.array([[0,                     0,       0,     233], #159
                                [0,                  -pi/2,       0,      0],
                                [0,                     0,     265.7,   0],
                                [0,                 -pi/2,     30,      0],
                                [0,                  pi/2,       0,      264],
                                [0,                 -pi/2,       0,      0],
                                [0,                     0,       0,     235.27]
                                ])
        self.right_arm_DH_table = np.array([[0,                     0,       0,     233],
                                [0,                  -pi/2,       0,      0],
                                [0,                     0,     265.7,   0],
                                [0,                 -pi/2,     30,      0],
                                [0,                  pi/2,       0,      264],
                                [0,                 -pi/2,       0,      0],
                                [0,                     0,       0,     235.27]
                                ])
        ## left id 0~6
        self.left_arm = ArmInfo.ArmInfo(self.left_arm_DH_table,start_id=0,end_id=6)
        ## right id 7~12
        self.right_arm = ArmInfo.ArmInfo(self.right_arm_DH_table,start_id=6,end_id=12)
        self.left_arm.set_world_to_base_transform(-pi/2, 0)# 座標點不動，逆旋轉矩陣來轉換點座標
        self.right_arm.set_world_to_base_transform(pi/2, 0)# 座標點不動，逆旋轉矩陣來轉換點座標
        self.gripper_offset = 100 # in mm
        self.object_offset = 100 # in mm
        self.z_offset = 50 # in mm

        # self.initial_angle = np.array([0, -pi/2, 0,  0, -pi/2, -pi/2])# -pi*2/3
        # self.initial_angle = np.array([1.2/180*pi, -20.3/180*pi, 1.37917/180*pi, -79/180*pi, -14.4716/180*pi, 15.5/180*pi])# -pi*2/3
        # self.initial_angle= np.array([0.98943, -0.557161, 0.0187393, -1.28869, 0.638063, 0.69464])# -pi*2/3
        # self.initial_angle = np.array([0, -pi/2, 0,  0, 0, 0])# -pi*2/3
        # self.initial_angle= np.array([0.189335, -0.43748, -0.791364, 2.87292, -1.13343, -0.603484])# -pi*2/3
        self.initial_angle = np.array([0.33627088, -0.95706291, -0.05231596,  3.14100796, -1.00821, -2.79714295])# -pi*2/3
        self.left_world_pos = np.zeros(3)
        self.right_world_pos = np.zeros(3)
        self.left_orientation_deg = np.zeros(3)
        self.right_orientation_deg = np.zeros(3)

    def check_position_workspace(self,pos,ori,type, arm): ## not finished
        self.change_pos = False
        self.change_ori = False
        self.max_z = 400
        self.min_z = -330
        self.min_x = 200
        self.max_x = 700
        self.max_y = 760
        self.min_y = -760
        ptype = type  # 使用預設值初始化
        if  pos[2] > self.max_z:
            print("z out of range")
            print("adjust position!!")
            pos[2] = self.max_z
            self.change_pos = True
        elif pos[2] < self.min_z:
            print("z out of range")
            print("adjust position!!")
            pos[2] = self.min_z
            self.change_pos = True
        if pos[0] < self.min_x:
            print("x out of range")
            print("adjust position!!")
            pos[0] = self.min_x
            self.change_pos = True
        elif pos[0] > self.max_x:
            print("x out of range")
            print("adjust position!!")
            pos[0] = self.max_x
            self.change_pos = True
        if  pos[1] > self.max_y:
            print("y out of range")
            print("adjust position!!")
            pos[1] = self.max_y
            self.change_pos = True
        elif pos[1] < self.min_y:
            print("y out of range")
            print("adjust position!!")
            pos[1] = self.min_y
            self.change_pos = True
        
        if pos[2] > 170 and pos[2] <= 400:
            if type == "down":
                print("adjust orientation!! side")
                ori, ptype = self.get_picktype_pose(arm, "side", angle=90)
                self.change_ori = True
        if abs(pos[0]) < 530 and abs(pos[0]) > 200 and (pos[1] < 260 and pos[1] > -260):
            print("5:In table workspace")
            if type == "forward" and pos[0] < 400:
                ori, ptype = self.get_picktype_pose(arm, "side")
                print("adjust orientation!! side")
                self.change_ori = True
        if abs(pos[0]) < 530 and abs(pos[0]) > 200 and (abs(pos[1]) < 360 and abs(pos[1]) >= 260):
            print("5+:around table workspace")
        if abs(pos[0]) < 530 and abs(pos[0]) > 200 and (abs(pos[1]) >= 360 ):
             print("4,6: left or right area")
             if type != "forward":
                    ori, ptype = self.get_picktype_pose( arm, "forward")
                    print("adjust orientation!! forward")
                    self.change_ori = True
        if abs(pos[0]) >= 530 and abs(pos[0]) <= 700 and (abs(pos[1]) < 360 ):
            print("2: forward area")
        if abs(pos[0]) >= 530 and abs(pos[0]) <= 700 and (abs(pos[1]) >= 360 ):
            print("1,3: far forward area")
            if type != "forward":
                ori, ptype = self.get_picktype_pose(arm, "forward")
                print("adjust orientation!! forward")
                self.change_ori = True
        if self.change_pos:
            print("Warning: position out of workspace!!!!!!")
        if self.change_ori:
            print("Warning: orientation changed!!!!!!!!")
        return pos, ori, ptype

    def get_picktype_pose(self, arm, ptype,angle=0): 
        orientation = np.array([0, 0, 0])
        p_type = ptype
        if arm == "left":
            if ptype == "side":
                orientation = np.array([-180, 0, 0])
                if angle >= 90:
                    supplement_angle = 180 - angle
                    orientation[1] = supplement_angle # 90 => forward
                    if angle == 90:
                        p_type = "forward"
                else:
                    orientation[1] = -(angle)
            elif ptype == "down":
                orientation = np.array([-90, 0, -90])
                if angle > 90:
                    supplement_angle = 180 - angle
                    orientation[2] = -supplement_angle
                else:
                    orientation[2] =  angle
            elif ptype == "forward":
                orientation = np.array([-180, 90, 0]) 
            elif ptype == "reversal":
                orientation = np.array([-170, 0, 178]) 
               
        elif arm == "right":
            if ptype == "side":
                orientation = np.array([0,180,0])
                if angle > 90:
                    orientation[1] = -(angle)
                else:
                    supplement_angle = 180 - angle
                    orientation[1] = supplement_angle
                    
            elif ptype == "down":
                orientation = np.array([90, 0, -90])
                if angle > 90:
                   
                    orientation[2] = angle
                else:
                    supplement_angle = 180 - angle
                    orientation[2] = -supplement_angle
            elif ptype == "forward":
                orientation = np.array([0, 90, 0])
            elif ptype == "reversal":
                orientation = np.array([170, 0, -178])
                ## 水平 0,180,0 (-170 in; 170 out) 前(0,90,0) 垂直 90,0,-90(z:-95)
        else:
            print("Unknown arm")
        return orientation, p_type

    def quat_conjugate(self, q):
        return (-q[0], -q[1], -q[2], q[3])

    def quat_mul(self, a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw*bx + ax*bw + ay*bz - az*by,
                aw*by - ax*bz + ay*bw + az*bx,
                aw*bz + ax*by - ay*bx + az*bw,
                aw*bw - ax*bx - ay*by - az*bz)
    def step_IK_acc(self, ox_deg, oy_deg, oz_deg, px, py, pz, single_arm, acceleration_factor): ## not finished
        '''
        px, py, pz (mm ) in base coordinate
        ox_deg, oy_deg, oz_deg (deg) in base coordinate
        mode: "left", "right", "both"
        single_arm: left_arm or right_arm
        return: True if reached, False if not reached
        '''

        # self.vel_gain_ = 1.5
        self.vel_gain_ = 1.2
        # self.vel_gain_angular_ = 1.2
        self.vel_gain_angular_ = 1.0
        self.ki_gain_ = 0.1
        self.angular_threshold = pi/180
        self.linear_threshold = 1 # in mm
        self.stop_counter = 0
        # acceleration_factor = 0.0
        # acceleration_counter = 0
        pos = np.array([px, py, pz])
        ori = np.array([ox_deg, oy_deg, oz_deg])

        # pos, ori = self.position_workspace(pos, ori)


        single_arm.set_target_orientation(ori[0], ori[1], ori[2])# single_arm.target_orientation in radian
        single_arm.set_target_position(pos[0], pos[1], pos[2])

        print("Target Orientation = {}".format(np.around(single_arm.target_orientation )))
        print("single_arm.current_orientation = {}".format(np.around(single_arm.current_orientation, 2)))

        print("Target  in base = [{}, {}, {}]".format(single_arm.target_position[0], single_arm.target_position[1], single_arm.target_position[2]))
        print("single_arm.current_position = {}".format(np.around(single_arm.current_position, 2)))
        self.last_position = single_arm.current_position
        self.update_robot_pose()
        # set up angular error 
        # q_target = pb.getQuaternionFromEuler([float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])])
        # r_target = R.from_euler('XYZ', [float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])], degrees=False)
        r_target = R.from_euler('XYZ', [float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])], degrees=False)

        self.q_target = r_target.as_quat()  # (x, y, z, w)
        r_current = R.from_euler('XYZ', [float(single_arm.current_orientation[0]), float(single_arm.current_orientation[1]), float(single_arm.current_orientation[2])], degrees=False)
        self.q_current = r_current.as_quat()  # (x, y, z, w)
        # print("q_target  =", self.q_target)
        # print("q_current =", self.q_current)
        

        dot = self.q_current[0]*self.q_target[0] + self.q_current[1]*self.q_target[1] + self.q_current[2]*self.q_target[2] + self.q_current[3]*self.q_target[3]
        # 若內積為負，翻轉目標四元數
        if dot < 0.0:
            self.q_target = (-self.q_target[0], -self.q_target[1], -self.q_target[2], -self.q_target[3])
        self.q_err = self.quat_mul(self.q_target, self.quat_conjugate(self.q_current))
        # print("q_err     =", self.q_err)

        qw = max(min((self.q_err[3]), 1.0), -1.0)  # 取絕對值
        angle = 2.0 * acos(qw)
        # print("angle rad =", angle)
        
        # 檢查是否接近零旋轉
        if abs(angle) < 1e-6:
            axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        else:
            # 計算旋轉軸
            s = sqrt(1.0 - qw*qw)
            if s < 1e-8:
                axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            else:
                axis = np.array([self.q_err[0]/s, self.q_err[1]/s, self.q_err[2]/s], dtype=np.float32)
        # 確保選擇最短路徑（角度小於π）
        # print("angle rad =", angle)
        # print("axis      =", axis)
        if angle > np.pi:
            angle = 2*np.pi - angle
            axis = -axis
        error_ang_vec = axis * angle
        # print("angle rad =", angle)
        # print("axis      =", axis)
        # print("error_ang_vec = {}".format(np.around(error_ang_vec, 2)))
        angular_error_norm = np.linalg.norm(error_ang_vec)
        # print("angular_error_norm  = {}".format(np.around(angular_error_norm, 4)))
        # set up linear error
        error_pos = single_arm.target_position - single_arm.current_position
        linear_error_norm = np.linalg.norm(error_pos)
        # print("error_pos = {}".format(np.around(error_pos, 2)))
        # print("linear_error_norm  = {}".format(np.around(linear_error_norm, 2)))
        
        # check if reached

        if linear_error_norm < self.linear_threshold and angular_error_norm < self.angular_threshold:
            
            single_arm.set_error_integral(np.zeros(6)) 
            return True

        
        # P
        w = error_ang_vec * self.vel_gain_angular_ * acceleration_factor
        v = error_pos * self.vel_gain_ * acceleration_factor
        # w = error_ang_vec * self.vel_gain_angular_ 
        # v = error_pos * self.vel_gain_ 
        
        # 距離自適應
        if linear_error_norm > 150:
            print("Speed up")
            v *= 1.5
        elif linear_error_norm < 50:
            print("Slow down")
            v *= 0.7

        # [角, 線] -> xdot
        self.xdot = np.concatenate([w, v]).astype(np.float32)
    
        # q'=inverse(J) * xdot
        # print(  "xdot = {}".format(np.around(self.xdot, 2)))

        single_arm.set_joint_velocity(self.xdot)
        
        # qdot = solveDLS(self.J_, xdot, self.lambda_dls_)

        # 限速
        single_arm.check_motor_limit()

        # deadzone
        deadzone = 5e-4
        for i in range(6):
            if abs(single_arm.joint_velocity[i]) < deadzone:
                single_arm.set_joint_idx_velocity(0.0, i)
        print("joint_velocity = {}".format(np.around(single_arm.joint_velocity, 4)))
               

        # # 卡住檢測
        if self.last_position is not None:
            position_change = np.linalg.norm(single_arm.current_position - self.last_position)
            if position_change < 0.1:
                self.stop_counter += 1
                if self.stop_counter > 50:
                    print("not moving")
                    self.stop_counter = 0
                    # print("Stall detected, reduce integral")
                    # single_arm.set_error_integral(single_arm.error_integral_ * 0.5)
                    return True    
            else:
                self.stop_counter = 0
        return False
    def step_IK(self, ox_deg, oy_deg, oz_deg, px, py, pz, single_arm): ## not finished
        '''
        px, py, pz (mm ) in base coordinate
        ox_deg, oy_deg, oz_deg (deg) in base coordinate
        mode: "left", "right", "both"
        single_arm: left_arm or right_arm
        return: True if reached, False if not reached
        '''

        # self.vel_gain_ = 1.5
        self.vel_gain_ = 1.2
        # self.vel_gain_angular_ = 1.2
        self.vel_gain_angular_ = 1.0
        self.ki_gain_ = 0.1
        self.angular_threshold = pi/180
        self.linear_threshold = 1 # in mm
        self.stop_counter = 0
        # acceleration_factor = 0.0
        # acceleration_counter = 0
        pos = np.array([px, py, pz])
        ori = np.array([ox_deg, oy_deg, oz_deg])

        # pos, ori = self.position_workspace(pos, ori)


        single_arm.set_target_orientation(ori[0], ori[1], ori[2])# single_arm.target_orientation in radian
        single_arm.set_target_position(pos[0], pos[1], pos[2])

        print("Target Orientation = {}".format(np.around(single_arm.target_orientation )))
        print("single_arm.current_orientation = {}".format(np.around(single_arm.current_orientation, 2)))

        print("Target  in base = [{}, {}, {}]".format(single_arm.target_position[0], single_arm.target_position[1], single_arm.target_position[2]))
        print("single_arm.current_position = {}".format(np.around(single_arm.current_position, 2)))
        self.last_position = single_arm.current_position
        self.update_robot_pose()
        # set up angular error 
        # q_target = pb.getQuaternionFromEuler([float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])])
        # r_target = R.from_euler('XYZ', [float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])], degrees=False)
        r_target = R.from_euler('XYZ', [float(single_arm.target_orientation[0]), float(single_arm.target_orientation[1]), float(single_arm.target_orientation[2])], degrees=False)

        self.q_target = r_target.as_quat()  # (x, y, z, w)
        r_current = R.from_euler('XYZ', [float(single_arm.current_orientation[0]), float(single_arm.current_orientation[1]), float(single_arm.current_orientation[2])], degrees=False)
        self.q_current = r_current.as_quat()  # (x, y, z, w)
        # print("q_target  =", self.q_target)
        # print("q_current =", self.q_current)
        

        dot = self.q_current[0]*self.q_target[0] + self.q_current[1]*self.q_target[1] + self.q_current[2]*self.q_target[2] + self.q_current[3]*self.q_target[3]
        # 若內積為負，翻轉目標四元數
        if dot < 0.0:
            self.q_target = (-self.q_target[0], -self.q_target[1], -self.q_target[2], -self.q_target[3])
        self.q_err = self.quat_mul(self.q_target, self.quat_conjugate(self.q_current))
        # print("q_err     =", self.q_err)

        qw = max(min((self.q_err[3]), 1.0), -1.0)  # 取絕對值
        angle = 2.0 * acos(qw)
        # print("angle rad =", angle)
        
        # 檢查是否接近零旋轉
        if abs(angle) < 1e-6:
            axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        else:
            # 計算旋轉軸
            s = sqrt(1.0 - qw*qw)
            if s < 1e-8:
                axis = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            else:
                axis = np.array([self.q_err[0]/s, self.q_err[1]/s, self.q_err[2]/s], dtype=np.float32)
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
        angular_error_norm = np.linalg.norm(error_ang_vec)
        print("angular_error_norm  = {}".format(np.around(angular_error_norm, 4)))
        # set up linear error
        error_pos = single_arm.target_position - single_arm.current_position
        linear_error_norm = np.linalg.norm(error_pos)
        print("error_pos = {}".format(np.around(error_pos, 2)))
        print("linear_error_norm  = {}".format(np.around(linear_error_norm, 2)))
        
        # check if reached

        if linear_error_norm < self.linear_threshold and angular_error_norm < self.angular_threshold:
            
            single_arm.set_error_integral(np.zeros(6)) 
            return True

        
        # P
        
        w = error_ang_vec * self.vel_gain_angular_ 
        v = error_pos * self.vel_gain_ 
        
        # 距離自適應
        if linear_error_norm > 150:
            print("Speed up")
            v *= 1.5
        elif linear_error_norm < 50:
            print("Slow down")
            v *= 0.7

        # [角, 線] -> xdot
        self.xdot = np.concatenate([w, v]).astype(np.float32)
    
        # q'=inverse(J) * xdot
        print(  "xdot = {}".format(np.around(self.xdot, 2)))

        single_arm.set_joint_velocity(self.xdot)
        
        # qdot = solveDLS(self.J_, xdot, self.lambda_dls_)

        # 限速
        single_arm.check_motor_limit()

        # deadzone
        deadzone = 5e-4
        for i in range(6):
            if abs(single_arm.joint_velocity[i]) < deadzone:
                single_arm.set_joint_idx_velocity(0.0, i)
        print("joint_velocity = {}".format(np.around(single_arm.joint_velocity, 4)))
               

        # # 卡住檢測
        if self.last_position is not None:
            position_change = np.linalg.norm(single_arm.current_position - self.last_position)
            if position_change < 0.1:
                self.stop_counter += 1
                if self.stop_counter > 50:
                    print("not moving")
                    self.stop_counter = 0
                    # print("Stall detected, reduce integral")
                    # single_arm.set_error_integral(single_arm.error_integral_ * 0.5)
                    return True    
            else:
                self.stop_counter = 0
        return False

    def get_orientation_mode(self, mode, shift = 0): ## not finished
        ori = np.array([-180, 0, 0])
        if mode == "down":
            ori = np.array([-90, 0, 0])
            # turn in : z > 0 Counterclockwise
            # turn out: z < 0 Clockwise
            if shift > 90:
                ori[2] = 180 - shift
            else:
                ori[2] = -shift
        elif mode == "side":
            ori = np.array([-180, 0, 0])
            # turn out : y > 0 Counterclockwise
            # turn in  : y < 0 Clockwise
            if shift > 90:
                ori[1] = 180 - shift
            else:
                ori[1] = -shift

        elif mode == "forward":

            ori = np.array([-180, 90, 0])
        else:
            print("Unknown mode, use side")
            ori = np.array([-180, 0, 0])

        return ori


    def set_initial_pose(self, angle_array):
        self.initial_angle = np.array([angle_array[0], angle_array[1], angle_array[2], angle_array[3], angle_array[4], angle_array[5]])

    def update_robot_pose(self):
        self.left_arm.update()
        self.right_arm.update()
        self.left_world_pos = self.left_arm.transform_to_world_position(self.left_arm.current_position)
        self.right_world_pos = self.right_arm.transform_to_world_position(self.right_arm.current_position)
        self.left_orientation_deg = self.left_arm.get_current_orientation_in_degree()
        self.right_orientation_deg = self.right_arm.get_current_orientation_in_degree()
        self.get_arm_information()
    def get_arm_information(self):
        print("===================================")
        print("Current left world Position = {}".format(np.around(self.left_world_pos, 0)))
        print("Current left Orientation(deg) = {}".format(np.around(self.left_orientation_deg, 0)))
        print("current left joint angle  = {}".format(np.around(self.left_arm.delta_angle , 1)))
        print("-----------------------------------")
        print("Current right world  Position = {}".format(np.around(self.right_world_pos, 0)))
        print("Current right Orientation(deg) = {}".format(np.around(self.right_orientation_deg, 0)))
        print("current right joint angle  = {}".format(np.around(self.right_arm.delta_angle , 1)))
        print("===================================")

    
                        