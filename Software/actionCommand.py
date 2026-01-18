#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rospy
import json
import time
from geometry_msgs.msg import Point
import os
import threading
robot_state = False
last_done_count = 0
new_done_count = 0
def wait_for_robot_action_completion():
    global robot_state
    robot_state = False
    while not robot_state or last_done_count == new_done_count:
        time.sleep(0.5)
    robot_state = False

def action_state_callback(msg):
    global robot_state, new_done_count, last_done_count
    """接收並處理動作狀態"""
    state = msg.data
    rospy.loginfo(f"收到動作狀態: {state}")
    if state == "Done":
        rospy.loginfo("機器人動作完成，可以進行下一步操作。")
        robot_state = True 
        last_done_count = new_done_count
        new_done_count += 1


rospy.Subscriber('action_state', String, action_state_callback)



class CommandPublisher:
    def __init__(self):
        rospy.init_node('command_publisher', anonymous=True)
        self.pub = rospy.Publisher('robot_command', String, queue_size=10)
        self.camera_pub = rospy.Publisher('camera_command', String, queue_size=10)
        
        rospy.sleep(1)
    
    def capture_publisher(self, command):       
        # 發送指令示例
        if command == "head":
            commands = ["capture_head"]
        elif command == "arms":
            commands = ["capture_arms"]
        elif command == "left":
            commands = ["capture_left"]
        elif command == "right":
            commands = ["capture_right"]
        else:
            rospy.logwarn(f"未知相機指令: {command}")
            return

        for cmd in commands:
            msg = String()
            msg.data = cmd
            time.sleep(2)  # 等待 2 秒
            self.camera_pub.publish(msg)
            rospy.loginfo(f"發送指令: {cmd}")
            

    def synchronize_move(self, x, y, z, pick_mode, object_width):
        """同步移動命令"""
        command = {
            "action": "synchronize_move",  # 識別欄位
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "pick_mode": pick_mode,
            "object_width": float(object_width)
        }
        self._send(command)
        wait_for_robot_action_completion()
    
    def single_move(self, arm, x, y, z, pick_mode, angle):
        """單一移動命令"""
        command = {
            "action": "single_move",  # 識別欄位
            "arm": arm,
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "pick_mode": pick_mode,
            "angle": float(angle)
        }
        self._send(command)
        wait_for_robot_action_completion()
    
    def dual_move(self, lx, ly, lz, lpick_mode, l_angle, rx, ry, rz, rpick_mode, r_angle):
        """雙臂移動命令"""
        command = {
            "action": "dual_move",  # 識別欄位
            "left": {
                "x": float(lx),
                "y": float(ly),
                "z": float(lz),
                "pick_mode": lpick_mode,
                "angle": float(l_angle)
            },
            "right": {
                "x": float(rx),
                "y": float(ry),
                "z": float(rz),
                "pick_mode": rpick_mode,
                "angle": float(r_angle)
            }
        }
        self._send(command)
        wait_for_robot_action_completion()
    def close_gripper(self, arm, angle=700):
        """關閉夾爪命令"""
        command = {
            "action": "close_gripper",  # 識別欄位
            "arm": arm,
            # "angle": angle
        }
        self._send(command)
        wait_for_robot_action_completion()
    def open_gripper(self, arm):
        """打開夾爪命令"""
        command = {
            "action": "open_gripper",  # 識別欄位
            "arm": arm
        }
        self._send(command)
        wait_for_robot_action_completion()
    def neck_control(self, yaw, pitch):
        """頸部控制命令"""
        command = {
            "action": "neck_control",  # 識別欄位
            "yaw": float(yaw),
            "pitch": float(pitch)
        }
        self._send(command)
        wait_for_robot_action_completion()
    def _send(self, command):
        """統一發送函式"""
        msg = String()
        msg.data = json.dumps(command)
        print("Sending: {}".format(command))
        self.pub.publish(msg)
        rospy.sleep(0.1)
    def camera_body_search(self):
        self.neck_control(0, 76)
        time.sleep(2)
        self.capture_publisher("head")
    def camera_table_search(self):
        self.neck_control(0, 30)
        time.sleep(2)
        self.capture_publisher("head")
    def arms_camera_capture(self, x, y, z, pick_mode, arm):
        camera_to_gripper_offset = 57.5/2
        
        distance = 150
        if arm == "left":
            sign = 1
        elif arm == "right":
            sign = -1
        if pick_mode == "side":

            if arm == "left":
                self.single_move("left", x, y + distance, z - camera_to_gripper_offset, pick_mode, 0)
                # time.sleep(12)
                self.capture_publisher("left")
                time.sleep(12)
            elif arm == "right":
                self.single_move("right", x, y - distance, z - camera_to_gripper_offset, pick_mode, 0)
                # time.sleep(12)
                self.capture_publisher("right")
                time.sleep(12)
        elif pick_mode == "down":
            step1_thread = threading.Thread(
                target=self.single_move, args=(arm, 400, y , -250, pick_mode, sign * -90)
            )
            step1_thread.start()
            step1_thread.join()  # 等待步驟 1 完成
            step2_thread = threading.Thread(
                target=self.single_move, args=(arm, x - camera_to_gripper_offset, y, -200 , pick_mode, sign * -90)
            )
            step2_thread.start()
            step2_thread.join()  # 等待步驟 2 完成
            step3_thread = threading.Thread(
                target=self.single_move, args=(arm, x - camera_to_gripper_offset, y, -140 , pick_mode, sign * -90)
            )
            step3_thread.start()
            step3_thread.join()  # 等待步驟 3 完成
            self.capture_publisher(arm)
            time.sleep(12)
    
    def single_arm_place(self, x, y, z, pick_mode, size, angle, arm):
        if arm == "left":
            sign = 1
        elif arm == "right":
            sign = -1
        if pick_mode == "down":
            self.single_move(arm, x, y + sign * 13, z + 100, pick_mode, sign * -90)
            self.single_move(arm, x, y + sign * 13, z + 100, pick_mode, angle)
            self.single_move(arm, x, y + sign * 13, z + size[2]/2, pick_mode, angle)
            self.open_gripper(arm)
            self.single_move(arm, x, y + sign * 13, z + 100, pick_mode, sign * -90)


        elif pick_mode == "side":
            self.single_move(arm, x, y + sign * 120, z, pick_mode, angle)
            self.single_move(arm, x, y + sign * size[0]/2, z, pick_mode, angle)
            self.open_gripper(arm)
            self.single_move(arm, x, y + sign * size[0]/2, z + 100, pick_mode, angle)
        self.initial_position()
    
    def single_arm_pick(self, x, y, z, pick_mode, size, angle, arm):
        if arm == "left":
            sign = 1
        elif arm == "right":
            sign = -1
        if pick_mode == "down":

            
            self.single_move(arm, x, y + sign * 15, z + 100, pick_mode, angle)
           
            self.single_move(arm, x, y + sign * 15, z + size[2]/2, pick_mode, angle)
            
            self.close_gripper(arm)
            self.single_move(arm, 400, sign *180, -220, pick_mode,sign * -90) # 往前抬起手 並轉正
          
            # self.single_move(arm, 400, y + sign * 15, z + 100, pick_mode, sign * -90) 
            # self.single_move(arm, 400, -150, -250 , "down", 90)

        elif pick_mode == "side":
            self.single_move(arm, x, y + sign * 120, z, pick_mode, angle)
            time.sleep(1)
            self.single_move(arm, x, y + sign * size[0]/2, z, pick_mode, angle)
            time.sleep(1)
            self.close_gripper(arm)
            time.sleep(1)
            self.single_move(arm, x, y + sign * size[0]/2, z + 120, pick_mode, angle)

    def initial_position(self):
    
        self.dual_move( 420, 120, -130, "side", 150, 420, -120, -130, "side", 30)
        self.neck_control(0, 76)
        time.sleep(10)