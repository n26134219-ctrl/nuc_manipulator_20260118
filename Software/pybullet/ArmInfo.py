from math import *
import numpy as np
from scipy.spatial.transform import Rotation as R
class ArmInfo(object):
    def __init__(self, DH_table, start_id, end_id):
        # in terms of theta, alpha, a, d
        self.DH_table = DH_table
        
        self.start_id = start_id
        self.end_id = end_id
        # these two is currently based on 1st coordination
        self.current_position = np.zeros(shape=(3))
        self.current_orientation = np.zeros(shape=(3))

        self.target_position = np.zeros(shape=(3))
        # used to adjust the scale of linear velocity
        self.velocity_factor = 1.0

        # current delta angles of 6 motors
        # self.delta_angle = np.array([0, pi/2-arctan(30/264), pi/2+arctan(30/264), 0, 0, 0])
        self.delta_angle = np.zeros(shape=(6))
        self.motor_angles = np.zeros(shape=(6)) # actual motor angles
        self.acceleration_factor = 0.1
        # check if the motor is out of speed limit
        self.is_out_of_limit_ = False    
        self.joint_velocity = np.zeros(shape=(6)) # velocity of 6 joints
        self.update()

        self.camera_to_endeffector_transform = np.linalg.inv(self.rotation_matrix(2, pi))
        
        self.max_integral_=10.0
        self.error_integral_ = np.zeros(6, dtype=np.float32)
        self.dt_ = 0.02
        self.stall_counter=0
        self.distance_to_base = 0

    def transform_matrix(self, theta, alpha, a ,d):
        matrix = np.array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                           [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                           [0, sin(alpha), cos(alpha), d],
                           [0, 0, 0, 1]])
        
        return matrix


    def rotation_matrix(self, angle_type, theta):
        if angle_type == 0: # x
            return np.array([[1,    0,              0,              0],
                             [0,    cos(theta),     -sin(theta),    0], 
                             [0,    sin(theta),     cos(theta),     0],
                             [0,    0,              0,              1]])

        elif angle_type == 1: # y
            return np.array([[cos(theta),   0,      sin(theta),         0],
                             [0,            1,      0,                  0], 
                             [-sin(theta),  0,      cos(theta),         0],
                             [0,            0,      0,                  1]])

        elif angle_type == 2: # z
            return np.array([[cos(theta),   -sin(theta),    0,         0],
                             [sin(theta),   cos(theta),     0,         0], 
                             [0,            0,              1,         0],
                             [0,            0,              0,         1]])

    def transform_to_endeffector_position(self, position_in_camera):
        coord_camera = np.array([position_in_camera[0], position_in_camera[1], position_in_camera[2], 1])
        
        # ee坐標系下的相對位置
        coord_new = np.matmul(self.camera_to_endeffector_transform,(coord_camera))
        # print("coord_new:", coord_new)
        # print("TbE:", self.TbE)
        self.object_in_base_position = np.matmul(self.TbE, coord_new)
        # print("object_in_base_position:", self.object_in_base_position)
        return coord_new
    def set_world_to_base_transform(self, alpha, d):
        self.distance_to_base = d
        
        self.world_to_base_transform = np.linalg.inv(self.transform_matrix(0, alpha, 0, 0))

        self.base_to_world_transform = self.transform_matrix(0, alpha, 0, 0)
       
    def transform_to_base_position(self, position_in_world):
        coord_world = np.array([position_in_world[0], position_in_world[1] - self.distance_to_base, position_in_world[2], 1])
        # 新坐標系下的相對位置
        # coord_new = self.world_to_base_transform.dot(coord_world)
        coord_new = np.matmul(self.world_to_base_transform,(coord_world))
        return coord_new
    def transform_to_world_position(self, position_in_base):
        coord_world = np.array([position_in_base[0], position_in_base[1], position_in_base[2], 1])
        # print("coord_world:", coord_world)
        # print(self.base_to_world_transform)
        # 新坐標系下的相對位置
        coord_new = np.matmul(self.base_to_world_transform,(coord_world))
        coord_new[1] += self.distance_to_base
        # print("Distance to base:", self.distance_to_base)
        # print("coord_new:", coord_new)
        return coord_new
    def wrap_to_pi(self,angle):
        return (angle + np.pi) % (2*np.pi) - np.pi
    def forward_kinematics(self):

     
        # print("Delta angles (in degree):", np.around(self.delta_angle / pi * 180, 1))
        self.Tb0 = self.transform_matrix(self.DH_table[0, 0] , self.DH_table[0, 1], self.DH_table[0, 2], self.DH_table[0, 3])
        self.T01 = self.transform_matrix(self.DH_table[1, 0] + self.delta_angle[0], self.DH_table[1, 1], self.DH_table[1, 2], self.DH_table[1, 3])
        self.T12 = self.transform_matrix(self.DH_table[2, 0] + self.delta_angle[1], self.DH_table[2, 1], self.DH_table[2, 2], self.DH_table[2, 3])
        self.T23 = self.transform_matrix(self.DH_table[3, 0] + self.delta_angle[2], self.DH_table[3, 1], self.DH_table[3, 2], self.DH_table[3, 3])
        self.T34 = self.transform_matrix(self.DH_table[4, 0] + self.delta_angle[3], self.DH_table[4, 1], self.DH_table[4, 2], self.DH_table[4, 3])
        self.T45 = self.transform_matrix(self.DH_table[5, 0] + self.delta_angle[4], self.DH_table[5, 1], self.DH_table[5, 2], self.DH_table[5, 3])
        self.T5E = self.transform_matrix(self.DH_table[6, 0] + self.delta_angle[5], self.DH_table[6, 1], self.DH_table[6, 2], self.DH_table[6, 3])
        # print("Tb0:", self.Tb0)
        # print("T01:", self.T01)
        # print("T12:", self.T12)
        # print("T23:", self.T23)
        self.Tb1 = np.dot(self.Tb0, self.T01)

        self.T02 = np.dot(self.T01, self.T12)
        self.T03 = np.dot(self.T02, self.T23)
        self.T04 = np.dot(self.T03, self.T34)
        self.T05 = np.dot(self.T04, self.T45)
        self.T0E = np.dot(self.T05, self.T5E)
        # print("Tb1:", self.Tb1)
        self.Tb2 = np.dot(self.Tb1, self.T12)
        # print("Tb2:", self.Tb2)
        self.Tb3 = np.dot(self.Tb2, self.T23)
        # print("Tb3:", self.Tb3)
        self.Tb4 = np.dot(self.Tb3, self.T34)
        self.Tb5 = np.dot(self.Tb4, self.T45)
        self.TbE = np.dot(self.Tb5, self.T5E)
        # print("TbE:", self.TbE)
        # Position of each joint
        self.Pbb = np.array([0, 0, 0])
        self.Pb0 = np.array(self.Tb0[0:3, 3])
        self.Pb1 = np.array(self.Tb1[0:3, 3])
        self.Pb2 = np.array(self.Tb2[0:3, 3])
        self.Pb3 = np.array(self.Tb3[0:3, 3])
        self.Pb4 = np.array(self.Tb4[0:3, 3])
        self.Pb5 = np.array(self.Tb5[0:3, 3])
        self.PbE = np.array(self.TbE[0:3, 3])
        # print("PbE:", self.PbE)
        self.P01 = np.array(self.T01[0:3, 3])
        self.P02 = np.array(self.T02[0:3, 3])
        self.P03 = np.array(self.T03[0:3, 3])
        self.P04 = np.array(self.T04[0:3, 3])
        self.P05 = np.array(self.T05[0:3, 3])
        self.P0E = np.array(self.T0E[0:3, 3])


        # End effector orientation
        # self.oz = atan2(self.TbE[1, 0], self.TbE[0, 0])
        # self.oy = atan2(-self.TbE[2, 0], (self.TbE[0, 0] * cos(self.oz) + self.TbE[1, 0] * sin(self.oz)))
        # self.ox = atan2((self.TbE[0, 2] * sin(self.oz) - self.TbE[1, 2] * cos(self.oz)), (self.TbE[1, 1] * cos(self.oz) - self.TbE[0, 1] * sin(self.oz)))
        # R_matrix = self.TbE[0:3, 0:3]  # 提取3x3旋转矩阵
        # r = R.from_matrix(R_matrix)
        # self.ox, self.oy, self.oz = r.as_euler('xyz', degrees=False)  # 返回弧度
    
        # self.oz = atan2(self.Tb5[1, 0], self.Tb5[0, 0])
        # self.oy = atan2(-self.Tb5[2, 0], (self.Tb5[0, 0] * cos(self.oz) + self.Tb5[1, 0] * sin(self.oz)))
        # self.ox = atan2((self.Tb5[0, 2] * sin(self.oz) - self.Tb5[1, 2] * cos(self.oz)), (self.Tb5[1, 1] * cos(self.oz) - self.Tb5[0, 1] * sin(self.oz)))

        R_matrix = self.TbE[0:3, 0:3]

        r = R.from_matrix(R_matrix)
        # print("R_matrix:", R_matrix)
        # print("r.as_matrix():", r.as_matrix())
        # 按固定 X→Y→Z （extrinsic）顺序提取欧拉角
        ox, oy, oz = r.as_euler('XYZ', degrees=False)
        # ox, oy, oz = r.as_euler('xyz', degrees=False)
        
        # 归一化到 [–π,π]
        self.ox = self.wrap_to_pi(ox)
        self.oy = self.wrap_to_pi(oy)
        self.oz = self.wrap_to_pi(oz)
        print("Euler angles (in radian):", ox, oy, oz)
        self.current_orientation = np.array((self.ox, self.oy, self.oz)) # in radian

        # End effector position
        # self.current_position = np.array((self.Pb5[0], self.Pb5[1], self.Pb5[2]))
        # print("pbe")
        # print((self.PbE[0], self.PbE[1], self.PbE[2]))
        
        self.current_position = np.array((self.PbE[0], self.PbE[1], self.PbE[2]))

    def set_target_orientation(self, ox, oy, oz):
        self.target_orientation = np.array((ox, oy, oz))  / 180 * pi # in radian
    def set_delta_angle(self, delta_angle):
        self.delta_angle = delta_angle
        print("Set delta angle to (in degree):", np.around(self.delta_angle / pi * 180, 1))
    def get_current_orientation_in_degree(self):
        return self.current_orientation / pi * 180  # in degree

    def jacobian(self):
       

        Z00 = np.array([0, 0, 1])
        Z01 = np.array(self.T01[0:3, 2])
        Z02 = np.array(self.T02[0:3, 2])
        Z03 = np.array(self.T03[0:3, 2])
        Z04 = np.array(self.T04[0:3, 2])
        Z05 = np.array(self.T05[0:3, 2])

        Jv00 = np.cross(Z00, (self.P0E - np.array([0,0,0])))
        Jv01 = np.cross(Z01, (self.P0E - self.P01))
        Jv02 = np.cross(Z02, (self.P0E - self.P02))
        Jv03 = np.cross(Z03, (self.P0E - self.P03))
        Jv04 = np.cross(Z04, (self.P0E - self.P04))
        Jv05 = np.cross(Z05, (self.P0E - self.P05))
        # Jvb5 = np.cross(Zb5, (self.PbE - self.Pb4))
        self.jacobian_matrix = np.array([
            [Z00[0], Z01[0], Z02[0], Z03[0], Z04[0], Z05[0]],
            [Z00[1], Z01[1], Z02[1], Z03[1], Z04[1], Z05[1]],
            [Z00[2], Z01[2], Z02[2], Z03[2], Z04[2], Z05[2]],
            [Jv00[0], Jv01[0], Jv02[0], Jv03[0], Jv04[0], Jv05[0]],
            [Jv00[1], Jv01[1], Jv02[1], Jv03[1], Jv04[1], Jv05[1]],
            [Jv00[2], Jv01[2], Jv02[2], Jv03[2], Jv04[2], Jv05[2]],
        ])
        # print("Jacobian matrix:\n", np.around(self.jacobian_matrix, 3))
        self.inverse_jacobian = np.linalg.pinv(self.jacobian_matrix)


    def set_stall_counter(self, count):
        self.stall_counter = count

    def set_error_integral(self, error_integral):
        self.error_integral_ = error_integral
    def set_target_position(self, x, y, z):
        self.target_position = np.array((x, y, z)) # in mm
    def get_joint_velocity(self):
        return self.joint_velocity

    def set_motor_angles(self, index,motor_angle):
        self.motor_angles[index] = motor_angle
        # print("Set motor {} angle to (in degree): {}".format(index, np.around(motor_angle / pi * 180, 1)))
    def solveDLS(self, J, xdot, damping_factor=0.08):
        # print("J:", J)
        # print("xdot:", xdot)
        JtJ = np.dot(J.T, J)
        # print("JtJ:", JtJ)
        damped_inv = np.linalg.inv(JtJ + damping_factor**2 * np.eye(J.shape[1]))
        # print("damped_inv:", damped_inv)

        return np.dot(damped_inv, np.dot(J.T, xdot))
   
    def set_joint_velocity(self, linear_vel):
        # print("Set linear velocity to:", linear_vel)

        # print("inverse jacobian:", self.inverse_jacobian)
        self.joint_velocity = self.solveDLS( self.jacobian_matrix, linear_vel)
        # print("Calculated joint velocity:", self.joint_velocity)
        # 使用阻尼偽逆避免奇異點
    def set_joint_idx_velocity(self, velocity, idx):
        self.joint_velocity[idx] = velocity

    def update(self):
        print("--- Update ---")
        
        self.forward_kinematics()
        # print("Current Position = {}".format(np.around(self.current_position, 0)))
        # print("Current Orientation = {}".format(np.around(self.get_current_orientation_in_degree(), 0)))
        self.jacobian()

    def check_motor_limit(self):
        # print("Joint velocity:", self.joint_velocity)
        max_speed = 0.0 # max speed in all motor 
        max_index = 0 # index of max speed motor
        for i in range(6):
            if (abs(self.joint_velocity[i]) >= max_speed):
                max_speed = abs(self.joint_velocity[i])
                max_index = i

        if max_speed >= pi/4: # dangerous zone
            scale_factor = pi / 4 / max_speed
            self.joint_velocity *= scale_factor
            self.error_integral_ *= 0.9

            self.is_out_of_limit_ = True 
            print("\tDangerous !")
            # self.joint_velocity = np.zeros(shape=(6))
        # elif max_speed >= (pi / 3):# limited zone
        #         self.joint_velocity *= abs((pi / 3) / self.joint_velocity[max_index])
    