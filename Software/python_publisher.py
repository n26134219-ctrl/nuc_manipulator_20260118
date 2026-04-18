
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import time
from geometry_msgs.msg import Point
from dotenv import load_dotenv
import os
from openai import OpenAI
import math
from actionCommand import CommandPublisher
from task_planning import *
from plan_validator import *
from typing import List
import threading
robot_control = CommandPublisher()
GPT_planner = GPTPlanner()
total_objects_phrase_pub = rospy.Publisher('assign_object_phase', String, queue_size=10)
left_objects_phrase_pub = rospy.Publisher('assign_left_object_phase', String, queue_size=10)
right_objects_phrase_pub = rospy.Publisher('assign_right_object_phase', String, queue_size=10)

class Robotstatus:
    def __init__(self):
        self.left_arm_status = "empty" # 記錄抓取物品
        self.right_arm_status = "empty" # 記錄抓取物品
robot_status = Robotstatus()

class SharedObject:
    def __init__(self):
        self.total = []  # 總物體列表
        self.left = []   # 左側物體列表
        self.right = []  # 右側物體列表
        self.other = []  # 其他物體列表
        self.head_camera_ready = False
        self.left_camera_ready = False
        self.right_camera_ready = False
    def update_robot_status_prompt(self, arm, object_name, status=None):
        global GPT_planner,robot_status
        # 1. 根據輸入更新對應手的狀態
        if status == "holding":
            status_text = f"holding {object_name}" if object_name else "empty"
            if arm == "left":
                robot_status.left_arm_status = object_name # 更新抓取物品
            elif arm == "right":
                robot_status.right_arm_status = object_name # 更新抓取物品
        elif status == "clear":
            status_text = "empty"
            if arm == "left":
                robot_status.left_arm_status = "empty"
            elif arm == "right":
                robot_status.right_arm_status = "empty"

        if arm == "left":
            GPT_planner.left_arm_status = status_text # 更新 gpt robot status prompt
        elif arm == "right":
            GPT_planner.right_arm_status = status_text  # 更新 gpt robot status prompt
        elif arm == "clear": # 額外功能：清空所有狀態
            GPT_planner.left_arm_status = "empty"
            GPT_planner.right_arm_status = "empty"
        
        # 2. 將兩者的狀態組合進最終的 Prompt
        GPT_planner.robot_status_prompt = (
            "機器人狀態: \n"
            f"left arm: {GPT_planner.left_arm_status}\n"
            f"right arm: {GPT_planner.right_arm_status}\n"
        )
        
        print("更新機器人狀態提示詞:")
        print(GPT_planner.robot_status_prompt) 
    def record_pick(self, object_name, arm):
        for obj in self.total:
            if obj['name'] == object_name:
                obj['status'] = f"{arm} arm is holding {object_name}"
                update_camera_prompt()  # 更新提示詞以反映物品狀態變化
                self.update_robot_status_prompt(arm, object_name, "holding")  # 更新機器人狀態提示詞
                break
    def record_place(self, object_name, arm):
        for obj in self.total:
            if obj['name'] == object_name:
                obj['status'] = f"{arm} arm is empty"
                update_camera_prompt()  # 更新提示詞以反映物品狀態變化
                self.update_robot_status_prompt(arm, object_name, "clear")  # 清空手臂狀態
                break
                     
shared_object = SharedObject()



# ================================ ROS 訂閱回調函式 =============================================
# 接收並更新總物體列表
def total_objects_callback(msg):
    """接收並更新總物體列表"""
    global shared_object
    try:
        raw_data = json.loads(msg.data)
        if isinstance(raw_data, list):
            incoming_list = raw_data
        else:
            incoming_list = [raw_data]
        for new_item in incoming_list:
            # 防呆
            if not isinstance(new_item, dict) or 'name' not in new_item:
                continue
            matched = False    
            # 4. 去舊清單找找看有沒有同名的
            if len(shared_object.total) > 0:
                for idx, old_item in enumerate(shared_object.total):
                    if old_item['name'] == new_item['name']:
                        # 找到相同物品更新它
                        shared_object.total[idx] = new_item
                        rospy.loginfo(f'更新物體資訊: {new_item["name"]}')
                        matched = True
                        break
            # 5. 沒找到，這是新物體，加入清單
            if not matched:
                shared_object.total.append(new_item)

        # show_info(shared_object.total)
    except json.JSONDecodeError as e:
            rospy.logerr(f"JSON 解析失败: {e}")
    except Exception as e:
        rospy.logerr(f"處理失敗: {e}")

# 接收並更新左側物體列表     
def left_objects_callback(msg):
    """接收並更新左側物體列表"""
    global shared_object
    try:
        data = json.loads(msg.data)
        shared_object.left = data
        show_info(shared_object.left)  

        angle_fine_tune("left")
    except json.JSONDecodeError as e:
            rospy.logerr(f"JSON 解析失败: {e}")
    except Exception as e:
        rospy.logerr(f"處理失敗: {e}")

# 接收並更新右側物體列表
def right_objects_callback(msg):
    """接收並更新右側物體列表"""
    global shared_object
    try:
        data = json.loads(msg.data)
        shared_object.right = data
        show_info(shared_object.right)
        angle_fine_tune("right")
       
    except json.JSONDecodeError as e:
            rospy.logerr(f"JSON 解析失败: {e}")
    except Exception as e:
        rospy.logerr(f"處理失敗: {e}")

# 接收並處理任務說明
def task_explanation_callback(msg):
    global GPT_planner
    """接收並處理任務說明"""
    GPT_planner.task_description_prompt = msg.data
    rospy.loginfo(f"收到任務說明: {GPT_planner.task_description_prompt}")

def camera_ready_callback(msg): # camera propmt更新會觸發
    global shared_object
    """接收並處理相機準備狀態"""
    state = msg.data
    rospy.loginfo(f"收到相機準備狀態: {state}")
    if state == "head_ready":
        shared_object.head_camera_ready = True
    elif state == "left_ready":
        shared_object.left_camera_ready = True
    elif state == "right_ready":
        shared_object.right_camera_ready = True

def task_type_callback(msg):
    global GPT_planner
    category = msg.data
    if category == "cleaning":
        get_env_info()
        update_camera_prompt()
    elif category == "pick_pepper":
        GPT_planner.task_description_prompt = "請規劃一個機器人抓取胡椒粉罐動作"
        get_pepper_info()
        update_camera_prompt()
    elif category == "pick_toast":
        GPT_planner.task_description_prompt = "請規劃一個機器人完成夾取土司動作"
        get_toast_info()
        update_camera_prompt()
    elif category == "prepare_sandwich":
        GPT_planner.task_description_prompt =  "請規劃一個機器人完成準備三明治動作"
        get_sandwich()
        update_camera_prompt()

    elif category == "pick_pepper_and_toast":
        GPT_planner.task_description_prompt = "請規劃一個機器人完成夾取胡椒粉與土司動作"
        get_pepper_toast_info()
        update_camera_prompt()
    elif category == "place_pepper":
        GPT_planner.task_description_prompt = "請規劃一個機器人完成放胡椒粉到原位動作。"
        GPT_planner.environment_context_prompt = ("")


    robot_plan = generate_task_plan()
    
    while True:
            user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            if user_input == "1":
                print("✓ 繼續執行...")
                
                break 
            elif user_input.lower() == "q":
                print("✗ 取消動作")
                exit()
            else:
                print("⚠ 請輸入 1 或 q")
    if robot_plan:
        executor.execute_plan_together(robot_plan)


def ros_sub_init():
    rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    rospy.Subscriber('task_explanation', String, task_explanation_callback)
    rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    rospy.Subscriber('/camera/left_objects', String, left_objects_callback)
    rospy.Subscriber('/camera/right_objects', String, right_objects_callback)
    rospy.Subscriber('/camera/camera_ready', String, camera_ready_callback)
    rospy.Subscriber('task_type', String, task_type_callback)

# ================================= 輔助顯示資訊函式 =============================================
def show_info(object):
    for idx, obj_info in enumerate(object):
                rospy.loginfo(f"\n對象 {idx}:")
                rospy.loginfo(f"  名稱: {obj_info['name']}")
                rospy.loginfo(f"  基座標: {obj_info.get('base_center_pos', 'N/A')}")
                rospy.loginfo(f"  左端點: {obj_info.get('left_base_pos', 'N/A')}")
                rospy.loginfo(f"  右端點: {obj_info.get('right_base_pos', 'N/A')}")
                rospy.loginfo(f"  角度: {obj_info['angle']:.1f} deg")
                rospy.loginfo(f"  抓取模式: {obj_info['pick_mode']}")
                rospy.loginfo(f"  中心向量: {obj_info.get('center_vector', 'N/A')}")
                rospy.loginfo(f"  尺寸: {obj_info.get('3d_size', 'N/A')}")
                rospy.loginfo(f"====================================================")
                rospy.loginfo(f"  相機ee座標: {obj_info.get('center_pos', 'N/A')}")
                rospy.loginfo(f"====================================================")
#=================================== update prompt functions ====================================
def update_camera_prompt():
    global GPT_planner
    GPT_planner.camera_information_prompt = "[object] info: \n"
    for idx, obj in enumerate(shared_object.total):
        name = obj['name']
        angle = obj['angle']
        pos = obj.get('base_center_pos', None)
        pick_mode = obj.get('pick_mode', "down")
        status = obj.get('status', None)
        if pos:
            GPT_planner.camera_information_prompt += f"object_name: {name} \n"
            GPT_planner.camera_information_prompt += f"object_index: {idx} \n"
            GPT_planner.camera_information_prompt += f"object_position: px={pos[0]:.1f}mm, py={pos[1]:.1f}mm, pz={pos[2]:.1f}mm \n"
            GPT_planner.camera_information_prompt += f"object_angle: {angle:.1f} deg \n"
            GPT_planner.camera_information_prompt += f"pick_mode: {pick_mode} \n"
            if status:
                GPT_planner.camera_information_prompt += f"status: {status}\n\n"


    GPT_planner.camera_information_prompt += "===============================\n"
            
    print("更新相機資訊提示詞:")
    print(GPT_planner.camera_information_prompt)

def angle_fine_tune(arm):
    global shared_object
    if arm == "left":
        objects = shared_object.left
    elif arm == "right":
        objects = shared_object.right
    else:
        return
    for obj in objects:
        angle = obj['angle']
        vector = obj['center_vector']
        name = obj['name']
        if name == 'dustpan tool':
            if arm == "left":
                if vector != None and vector[1] < 0:
                    if angle > 0 or angle == 0: 
                       
                        obj['angle'] = -(180 - angle)
                    else:
                        obj['angle'] = 180 + angle
                rospy.loginfo(f"左:微調角度: {obj['angle']}")
            elif arm == "right":
                if vector != None and vector[1] > 0:
                    if angle > 0 or angle == 0: 
                       
                        obj['angle'] = -(180 - angle)
                    else:
                        obj['angle'] = 180 + angle
                rospy.loginfo(f"右:微調角度: {obj['angle']}")
        elif  name == 'brush tool':
            if arm == "left":
                if vector != None and vector[0] > 0:
                    if angle > 0 or angle == 0: 
                        if angle >= 90:
                            obj['angle'] = angle
                        else:
                            obj['angle'] = -(180 - angle)
                    else:
                        obj['angle'] = 180 + angle
                else:
                    if angle > 90 or angle == 90: 
                        obj['angle'] = -(180 - angle) 
                rospy.loginfo(f"左:微調角度: {obj['angle']}")

            elif arm == "right":
                if vector != None and vector[0] < 0:
                    if angle > 0 or angle == 0: 
                        if angle >= 90:
                            obj['angle'] = angle
                        else:
                            obj['angle'] = -(180 - angle)
                    else:
                        obj['angle'] = 180 + angle
                else:
                    if angle > 90 or angle == 90: 
                        obj['angle'] = -(180 - angle)
                rospy.loginfo(f"右:微調角度: {obj['angle']}")

# ==================================== action functions for camera ====================================
def draw_back_hands():
    time.sleep(3)
    robot_control.dual_move(266.3, 80, -230, "side", -10, 266.3, -80, -230, "side", 10)

def draw_back_hands_high():
    time.sleep(3)
    robot_control.dual_move(280, 80, -130, "side", -10, 280, -80, -130, "side", 10)

# ==================================== camera search functions ====================================

def update_search_phrase(phase_list):
    json_phase = json.dumps(phase_list)
    total_objects_phrase_pub.publish(json_phase)
    time.sleep(2)

def search_trush():
    robot_control.capture_publisher("head")
    time.sleep(20)
    find_trash = False
    print("開始搜尋垃圾...")
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):
            print(f"檢查物件: {obj['name']}")

            if "rice food" in obj['name'] or "rice food" == obj['name']:
                if obj['3d_size'][0] >  300 or obj['3d_size'][1] >  300 or obj['base_center_pos'][0] > 650:
                    rospy.loginfo("垃圾尺寸過大，無法拾取")
                    return False
                else:
                    find_trash = True
                    print("找到垃圾，開始處理...")
                    left_pos = obj.get('left_base_pos', None)
                    right_pos = obj.get('right_base_pos', None)
                    center_pos = obj.get('base_center_pos', None)
                    # 待測試
                    print(f"原始垃圾的相機座標: center {center_pos}, left {left_pos}, right {right_pos}")
                    center_pos[2] -= 20
                    left_pos[2] -= 20
                    right_pos[2] -= 20 # bounding box 大一點
                    min_z = min(center_pos[2], left_pos[2], right_pos[2])
                    # center_pos[2] = center_pos[2] - 10
                    center_pos[2] = (center_pos[2] + min_z) / 2
                    left_pos[2] = (left_pos[2] + min_z) / 2
                    right_pos[2] = (right_pos[2] + min_z) /2
                    print(f"更新垃圾的相機座標: center {center_pos} ")
                    print(f"更新垃圾的相機座標: left {left_pos} ")
                    print(f"更新垃圾的相機座標: right {right_pos} ")

                    rospy.loginfo(f"找到垃圾，索引為: {index}")
                    return True
        if not find_trash:
             rospy.loginfo("未找到垃圾")
             return False
    else:
        rospy.loginfo("未找到垃圾")
        return False

def search_dustpan_broom():
    robot_control.capture_publisher("head")
    time.sleep(16)
    found_dustpan = False
    found_brush = False
    if len(shared_object.total) >= 2:
        for index, obj in enumerate(shared_object.total):
            name = obj['name']
            if obj['3d_size'][0] >  300 or obj['3d_size'][1] >  300 or obj['3d_size'][2] >  300:
                rospy.loginfo("掃把或畚箕尺寸過大，無法拾取")
                return False
            if name == "dustpan tool":
                found_dustpan = True
            if name == "brush tool":
                found_brush = True
        if found_dustpan and found_brush:
            rospy.loginfo("✓ 成功找到掃把和畚箕")
            return True
        else:
            rospy.loginfo("未找到掃把或畚箕")
            return False
    else:
        rospy.loginfo("未找到掃把或畚箕")
        return False  

def search_pepper_shaker(): 
    found_pepper = False
    robot_control.capture_publisher("head")
    time.sleep(15)
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):
            if "pepper shaker" in obj['name']:
                if obj['3d_size'][0] >  200 or obj['3d_size'][1] >  100 or obj['3d_size'][2] >  100:
                    rospy.loginfo("胡椒罐尺寸過大，無法拾取")
                    return False
                elif obj['base_center_pos'][0] > 680 or abs(obj['base_center_pos'][1]) > 300:
                    rospy.loginfo("胡椒罐位置過遠，無法拾取")
                    return False
                else:
                    rospy.loginfo("✓ 成功找到胡椒罐")
                    rospy.loginfo(f"找到胡椒罐，索引為: {index}")
                    if obj['pick_mode'] != "side":
                        obj['pick_mode'] = "side"
                    
                    left_pos = obj.get('left_base_pos', None)
                    right_pos = obj.get('right_base_pos', None)
                    center_pos = obj.get('base_center_pos', None)
                    # z_values = [center_pos[0], left_pos[0], right_pos[0]]
                    # z_values.sort()
                    # mini_z = z_values[0]  # 排序最小值的值
                    # center_pos[0] = center_pos[0]-5
                    if center_pos[1] > 0:
                        center_pos[1] = center_pos[1] -5
                    else:
                        center_pos[1] = center_pos[1] +5
                    # center_pos[0] = left_pos[0] = right_pos[0] = mini_z 
                    center_pos[2] -= 20
                    left_pos[2] -= 20
                    right_pos[2] -= 20 # bounding box 大一點
                    min_z = min(center_pos[2], left_pos[2], right_pos[2])
                    # center_pos[2] = center_pos[2] - 10
                    center_pos[2] = (center_pos[2] + min_z) / 2
                    return True
        
    else:
        rospy.loginfo("未找到胡椒罐")
        return False    
    
def search_toast():
    found_toast = False
    robot_control.capture_publisher("head")
    time.sleep(15)
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):

            if "sliced toast" in obj['name']:
                if obj['3d_size'][0] >  200 or obj['3d_size'][1] >  200 or obj['3d_size'][2] >  100:
                    rospy.loginfo("土司尺寸過大，無法拾取")
                    return False
                elif obj['base_center_pos'][0] > 680 or abs(obj['base_center_pos'][1]) > 300:
                    rospy.loginfo("土司位置過遠，無法拾取")
                    return False
                else:
                    rospy.loginfo("✓ 成功找到土司")
                    rospy.loginfo(f"找到土司，索引為: {index}")
                    if obj['pick_mode'] != "side":
                        obj['pick_mode'] = "side"
                    found_toast = True
                    left_pos = obj.get('left_base_pos', None)
                    right_pos = obj.get('right_base_pos', None)
                    center_pos = obj.get('base_center_pos', None)
                    z_values = [center_pos[2], left_pos[2], right_pos[2]]
                    z_values.sort()
                    mini_z = z_values[0]  # 排序最小值的值
                    center_pos[2] = left_pos[2] = right_pos[2] = mini_z - 10 # 夾爪末端寬度
                    angle = obj['angle']
                    if angle < 30 or angle > 150:
                        if angle < 30:
                            obj['angle'] = obj['angle'] + 8
                            
                        else:
                            obj['angle'] = obj['angle'] - 8
                        
                    if obj['angle']>90 :
                        theta = 180 - obj['angle']
                        sign = -1
                    else:
                        theta = obj['angle']
                        sign = 1
                    if obj['3d_size'][2]> 30:
                        thickness = obj['3d_size'][2] 
                        if thickness > 80:
                            thickness = thickness//2
                        else:
                            thickness = thickness//3
                        # print(f"center_pos before: {center_pos}")
                        # print(f"left_pos before: {left_pos}")
                        # print(f"right_pos before: {right_pos}")
                        obj['base_center_pos'] = [center_pos[0]- thickness * math.cos(math.radians(theta)), center_pos[1] + sign * thickness * math.sin(math.radians(theta)), center_pos[2]]
                        obj['left_base_pos'] = [left_pos[0]- thickness * math.cos(math.radians(theta)), left_pos[1] + sign * thickness * math.sin(math.radians(theta)), left_pos[2]]
                        obj['right_base_pos'] = [right_pos[0]- thickness * math.cos(math.radians(theta)), right_pos[1] + sign * thickness * math.sin(math.radians(theta)), right_pos[2]]
                        # print(f"center_pos after: {obj['base_center_pos']}")
                        # print(f"left_pos after: {obj['left_base_pos']}")
                        # print(f"right_pos after: {obj['right_base_pos']}")
    else:
        print("沒有物件")
           
    if  found_toast:
        return True      
    else:
        rospy.loginfo("未找到土司")
        return False

def search_pepper_and_toast():
    found_toast = False
    found_pepper = False
    robot_control.capture_publisher("head")
    time.sleep(15)
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):

            if "sliced toast" in obj['name']:
                if obj['3d_size'][0] >  200 or obj['3d_size'][1] >  200 or obj['3d_size'][2] >  100:
                    rospy.loginfo("土司尺寸過大，無法拾取")
                    return False
                elif obj['base_center_pos'][0] > 680 or abs(obj['base_center_pos'][1]) > 300:
                    rospy.loginfo("土司位置過遠，無法拾取")
                    return False
                else:
                    rospy.loginfo("✓ 成功找到土司")
                    rospy.loginfo(f"找到土司，索引為: {index}")
                    if obj['pick_mode'] != "side":
                        obj['pick_mode'] = "side"
                    found_toast = True
                    left_pos = obj.get('left_base_pos', None)
                    right_pos = obj.get('right_base_pos', None)
                    center_pos = obj.get('base_center_pos', None)
                    z_values = [center_pos[2], left_pos[2], right_pos[2]]
                    z_values.sort()
                    mini_z = z_values[0]  # 排序最小值的值
                    center_pos[2] = left_pos[2] = right_pos[2] = mini_z - 10 # 夾爪末端寬度
                    angle = obj['angle']
                    if angle < 30 or angle > 150:
                        if angle < 30:
                            obj['angle'] = obj['angle'] + 8
                            
                        else:
                            obj['angle'] = obj['angle'] - 8
                        
                    if obj['angle']>90 :
                        theta = 180 - obj['angle']
                        sign = -1
                    else:
                        theta = obj['angle']
                        sign = 1
                    if obj['3d_size'][2]> 30:
                        thickness = obj['3d_size'][2] 
                        if thickness > 80:
                            thickness = thickness//2
                        else:
                            thickness = thickness//3
                        # print(f"center_pos before: {center_pos}")
                        # print(f"left_pos before: {left_pos}")
                        # print(f"right_pos before: {right_pos}")
                        obj['base_center_pos'] = [center_pos[0]- thickness * math.cos(math.radians(theta)), center_pos[1] + sign * thickness * math.sin(math.radians(theta)), center_pos[2]]
                        obj['left_base_pos'] = [left_pos[0]- thickness * math.cos(math.radians(theta)), left_pos[1] + sign * thickness * math.sin(math.radians(theta)), left_pos[2]]
                        obj['right_base_pos'] = [right_pos[0]- thickness * math.cos(math.radians(theta)), right_pos[1] + sign * thickness * math.sin(math.radians(theta)), right_pos[2]]
                        # print(f"center_pos after: {obj['base_center_pos']}")
                        # print(f"left_pos after: {obj['left_base_pos']}")
                        # print(f"right_pos after: {obj['right_base_pos']}")
                    
            if "pepper shaker" in obj['name']:
                if obj['3d_size'][0] >  200 or obj['3d_size'][1] >  110 or obj['3d_size'][2] >  110:
                    rospy.loginfo("胡椒罐尺寸過大，無法拾取")
                    return False
                elif obj['base_center_pos'][0] > 680 or abs(obj['base_center_pos'][1]) > 300:
                    rospy.loginfo("胡椒罐位置過遠，無法拾取")
                    return False
                else:
                    rospy.loginfo("✓ 成功找到胡椒罐")
                    rospy.loginfo(f"找到胡椒罐，索引為: {index}")
                    if obj['pick_mode'] != "side":
                        obj['pick_mode'] = "side"
                    found_pepper = True
                    left_pos = obj.get('left_base_pos', None)
                    right_pos = obj.get('right_base_pos', None)
                    center_pos = obj.get('base_center_pos', None)
                    center_pos[0]-=10
                    left_pos[0]-=10
                    right_pos[0]-=10
                    if center_pos[1] > 0:
                        min_y = min(center_pos[1], left_pos[1], right_pos[1])
                        center_pos[1] = min_y -15
                    else:
                        max_y = max(center_pos[1], left_pos[1], right_pos[1])
                        center_pos[1] = max_y + 15
                    center_pos[2] -= 20
                    left_pos[2] -= 20
                    right_pos[2] -= 20 #去除底邊容易向上飄移 # bounding box 大一點
                    min_z = min(center_pos[2], left_pos[2], right_pos[2])
                    center_pos[2] = (center_pos[2] + min_z) / 2
    else:
        print("沒有物件")
        return False    
    if  found_toast and found_pepper:
        return True      
    else:
        if not found_toast:
             rospy.loginfo("未找到土司")
        if not found_pepper:
            rospy.loginfo("未找到胡椒罐")
        return False  

def search_sandwich():
    found_sandwich = False
    robot_control.capture_publisher("head")
    time.sleep(15)
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):
            if "sandwich" in obj['name']:
                if obj['base_center_pos'][0] > 680 or abs(obj['base_center_pos'][1]) > 300:
                    rospy.loginfo("三明治位置過遠，無法放置")
                    return False
                elif obj['3d_size'][0] >  200 or obj['3d_size'][1] >  200 or obj['3d_size'][2] >  100:
                    rospy.loginfo("三明治尺寸過大，無法放置")
                    return False
                else:
                    rospy.loginfo("✓ 成功找到三明治")
                    rospy.loginfo(f"找到三明治，索引為: {index}")
                    found_sandwich = True
    else:
        print("沒有物件")
            
         
    if  found_sandwich:
        return True      
    else:
        rospy.loginfo("未找到三明治")
        return False
   
def get_pepper_info():
    global shared_object
    draw_back_hands()
    update_search_phrase(["pepper shaker"]) # bottle
    robot_control.neck_control(0, 45)
    if shared_object.head_camera_ready == True:
        while search_pepper_shaker() == False:
            robot_control.neck_control(0, 45)
    time.sleep(2)
    shared_object.head_camera_ready = False
    print("================= 獲取環境資訊完成 =================")
    show_info(shared_object.total)

def get_toast_info():
    global shared_object
    draw_back_hands()
    update_search_phrase(["sliced toast"]) 
    robot_control.neck_control(0, 45) #40
    if shared_object.head_camera_ready == True:
        while search_toast() == False:
            robot_control.neck_control(0, 45) #40
    time.sleep(2)
    shared_object.head_camera_ready = False
    print("================= 獲取環境資訊完成 =================")
    show_info(shared_object.total)

def get_pepper_toast_info():
    draw_back_hands()
    update_search_phrase(["pepper shaker", "sliced toast"]) 
    robot_control.neck_control(0, 45)
    if shared_object.head_camera_ready == True:
        while search_pepper_and_toast() == False:
            robot_control.neck_control(0, 45)
    time.sleep(2)
    shared_object.head_camera_ready = False
    print("================= 獲取環境資訊完成 =================")
    show_info(shared_object.total)
def get_sandwich():
    global shared_object
    draw_back_hands()
    update_search_phrase(["sandwich"]) 
    robot_control.neck_control(0, 40)
    if shared_object.head_camera_ready == True:
        while search_sandwich() == False:
            robot_control.neck_control(0, 40)
    time.sleep(2)
    shared_object.head_camera_ready = False
    print("================= 獲取環境資訊完成 =================")
    show_info(shared_object.total)
def get_env_info():
    global shared_object
    robot_control.initial_position()
    robot_control.neck_control(0, 70)
    if shared_object.head_camera_ready == True:
        while search_dustpan_broom() == False:
            robot_control.neck_control(0, 70)
    time.sleep(2)
    shared_object.head_camera_ready = False
    draw_back_hands()
    robot_control.neck_control(0, 45)
    update_search_phrase(["rice food"])
    
    if shared_object.head_camera_ready == True:
        while search_trush() == False:
            robot_control.neck_control(0, 45)
    time.sleep(2)
    shared_object.head_camera_ready = False
    robot_control.neck_control(0, 70)
    print("================= 獲取環境資訊完成 =================")
    show_info(shared_object.total)
    robot_control.initial_position()

# ==================================== 動作函式呼叫範例 ====================================
class RobotExecutor:
    """機器人動作執行器"""
    def __init__(self):
        # 建立函式名稱到實際函式的映射表
        self.action_map = {
            "arm_eyeInHand_camera_catch": self.arm_eyeInHand_camera_catch,
            "pick": self.pick,
            # "sweep_the_table": self.sweep_the_table,
            "sweep_towards": self.sweep_towards,
            "place": self.place,
            "sprinkle_pepper":self.sprinkle_pepper,
            # "wait_for_pepper":self.wait_for_pepper,
            "wait": self.wait,
            "move_to_object": self.move_to_object,
            "close_sandwich":self.close_sandwich
        }
        self.active_arm = "left"
        self.auxiliary_arm = "right"
    # === 實際的機器人動作函式 ===
    
    def arm_eyeInHand_camera_catch(self, object_index: int, arm: str): # 讓手臂上相機再照一次，獲得準確物品資訊，更新物品資訊
        global shared_object
        # get object information
        object_info = shared_object.total[int(object_index)]
        object_pos = object_info.get('base_center_pos', None)
        pick_mode = object_info.get('pick_mode', "down")
        name = object_info.get('name', 'unknown')
        # update object phrase
        if arm == "left":
            left_objects_phrase_pub.publish(name)
            sign = -1
        elif arm == "right":
            right_objects_phrase_pub.publish(name)
            sign = 1
        if object_pos:
            rospy.loginfo(f"[{arm}] 相機重新捕捉物品物體 {object_index} 的基座標位置: {object_pos}")
            if pick_mode == "down":
                robot_control.arms_camera_capture(object_pos[0], object_pos[1], object_pos[2], pick_mode, arm)
            elif pick_mode == "side":
                if arm == "left":
                    robot_control.arms_camera_capture(object_pos[0], object_pos[1], object_pos[2], pick_mode, arm)
                elif arm == "right":
                    robot_control.arms_camera_capture(object_pos[0], object_pos[1], object_pos[2], pick_mode, arm)
            time.sleep(5)
    def pick(self, arm: str, object_name: str):
        # get object information
        find_object = False
        if arm == "left":
            if len(shared_object.left)>0:
                object = shared_object.left[0]
                object_name = object.get('name', None)
                find_object = True
            else:
                object = None
        elif arm == "right":
            if len(shared_object.right)>0:
                object = shared_object.right[0]
                object_name = object.get('name', None)
                find_object = True
            else:
                object = None
        if object == None:
            for obj in shared_object.total:
                if object_name in obj['name']:
                    find_object = True
                    object = obj
                    break
        if find_object == False:
            rospy.loginfo(f"未找到物品 {object_name} 的資訊，無法執行抓取")
            return       
        object_pos = object.get('base_center_pos', None)
        left_pos = object.get('left_base_pos', None)
        right_pos = object.get('right_base_pos', None)
        pick_mode = object.get('pick_mode', None)
        size = object.get('3d_size', None)
        angle = object.get('angle', 0)
        
        rospy.loginfo(f"[{arm}] 抓取物品 {object_name}，模式: {pick_mode}，角度: {angle}")
        
        if object_pos and pick_mode and size: 
            if object_name == 'dustpan tool':
                print(f"畚箕高度為: {size[2]}mm")
                handle_size = [size[0], size[1], size[2]]
                if handle_size[2]>32: 
                    handle_size[2] = 31.5-15  # 避免辨識錯誤
            if object_name == 'brush tool':
                if arm == "right":
                    self.active_arm = "right"
                    self.auxiliary_arm = "left"
                handle_size = [size[0], size[1], size[2]]
            if "pepper shaker" in object_name:
                if arm == "right":
                    self.active_arm = "right"
                    self.auxiliary_arm = "left"
                    angle = 30
                    print(f"微調後角度為: {angle}")
                else:
                    self.active_arm = "left"
                    self.auxiliary_arm = "right"
                    angle = 150
                    print(f"微調後角度為: {angle}")
            if object_name == 'brush tool' or object_name == 'dustpan tool':
                print(f"抓取物品尺寸為: {handle_size}")
                robot_control.single_arm_pick( object_pos[0], object_pos[1], object_pos[2], pick_mode, handle_size, angle, arm)
            elif object_name == 'sliced toast' :
                if right_pos[0]< left_pos[0]:
                    pick_pos = [right_pos[0], right_pos[1], right_pos[2]]
                else:
                    pick_pos = [left_pos[0], left_pos[1], left_pos[2]]
                robot_control.single_arm_pick( pick_pos[0], pick_pos[1], pick_pos[2], pick_mode, size, angle, arm)
            else:
                robot_control.single_arm_pick(object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)

    def move_to_object(self, arm: str, object_name: str):
        # get object information
        find_object = False
        
        
        for obj in shared_object.total:
            if object_name in obj['name']:
                find_object = True
                object = obj
                break
        if find_object == False:
            rospy.loginfo(f"未找到物品 {object_name} 的資訊，無法執行移動")
            return       
        
        if object_name == 'rice food'or  'rice food' in object_name:
            
            size = object['3d_size'].copy()
            angle = object.get('angle', 0)
            left_pos = object['left_base_pos'].copy()
            right_pos = object['right_base_pos'].copy()
            center_pos = object['base_center_pos'].copy()
            if left_pos != None and right_pos != None and center_pos != None and size != None:
                # 待測試
                if size[2]<20:
                    if size[2]<13:
                        size[2] = 18
                    size[2] = size[2]+5
                left_pos[2] = left_pos[2] - size[2]
                right_pos[2] = right_pos[2] - size[2]
                center_pos[2] = center_pos[2] - size[2]
               
                z_values = [center_pos[2], left_pos[2], right_pos[2]]
                z_values.sort()
                median_z = z_values[1]  # 排序後中間的值
                
                center_pos[2] = left_pos[2] = right_pos[2] = median_z
                if center_pos[2] > -323:    
                    center_pos[2] = left_pos[2] = right_pos[2] = -323
                elif center_pos[2] < -360:
                    center_pos[2] = left_pos[2] = right_pos[2] = -360
                # center_pos[2] = left_pos[2] = right_pos[2] = max(center_pos[2], left_pos[2], right_pos[2])
                print(f"桌面統一高度為: {center_pos[2]} mm")
            

            gripper_length = 23.5 #30   24    
            brush_length = 110  # 掃把握柄中心到尾端長度為125 105mm
            dustpan_length = 195  # 畚箕長度+距離offset假設為170 190 210 200mm
            brush_dis_offset = 75 #40 #55 # 60 # 掃把距離米的offset距離
            if (arm== "left" and self.active_arm == arm) or(arm == "right" and self.auxiliary_arm == arm):
                sign = 1
                side_angle=160
                print(f"dustpan_height: {shared_object.right[0]['3d_size']}")
                dustpan_height = shared_object.right[0]['3d_size'][2]
                if dustpan_height <32:
                    dustpan_height = 32
                print(f"dustpan_height: {dustpan_height}") 
                print(f"longest_length: {shared_object.left[0]['longest_length']}")
                brush_length= shared_object.left[0]['longest_length']-17 # 15 17 20 #20 #25 #18
                print(f"brush_length: {brush_length}")
                if angle>=0 and angle<=90:
                    if angle <5:
                        angle = angle + 5
                    angle = 180 - angle
                    temp = left_pos[0]
                    left_pos[0] =right_pos[0] # 避免超過 motor4 angle +90~-90
                    right_pos[0] = temp
                # right_angle = -(180 - angle)    
                print(f"left_pos: {left_pos}")
                print(f"right_pos: {right_pos}")
                
                dustpan_dis = dustpan_length
                dustpan_dis2= dustpan_dis + 15
                theta = math.radians((180-angle))
                brush_pos_close=left_pos
                dustpan_pos_close=right_pos
                LandR_dis = 125
                robot_control.neck_control(0, 45)
                while True:
                    if dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta) >580:#[修改]
                        if angle >=175:
                            print("無法調整至合適角度，請重新規劃動作")
                            break
                        else:
                            angle +=5
                            theta = math.radians((180-angle))

                    else:
                        break
                print(f"角度調整為: {angle}")
                
                brush_target=[brush_pos_close[0]- size[1]/4 - brush_dis_offset* math.sin(theta), brush_pos_close[1] + brush_dis_offset* math.cos(theta), brush_pos_close[2]+brush_length]
                brush_target2=[dustpan_pos_close[0]-size[1]/4+ (dustpan_dis-LandR_dis)* math.sin(theta), dustpan_pos_close[1]-(dustpan_dis-LandR_dis)* math.cos(theta) , dustpan_pos_close[2]+brush_length] ####[修改]
                center_target=[center_pos[0], center_pos[1], center_pos[2]+brush_length]
                dustpan_target=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis* math.sin(theta), dustpan_pos_close[1]-dustpan_dis* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
                dustpan_target2=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta), dustpan_pos_close[1]-dustpan_dis2* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
                auxiliary_angle =-(180 - angle)  
                print(f"brush_target: {brush_target}")
                print(f"dustpan_target: {dustpan_target}")
                print(f"brush_target2: {brush_target2}")
                print(f"dustpan_target2: {dustpan_target2}")
                print(f"auxiliary_angle: {auxiliary_angle}")
            else:
                sign = -1
                side_angle = 30 
                dustpan_height = shared_object.left[0]['3d_size'][2]
                if dustpan_height <30: #修改
                    dustpan_height = 32 
                # elif dustpan_height >35:  
                #     dustpan_height -=2 
                print(f"dustpan_height: {dustpan_height}") 
                print(f"longest_length: {shared_object.right[0]['longest_length']}")
                brush_length= shared_object.right[0]['longest_length']-18 # 15 17 20 #20 #25
                print(f"brush_length: {brush_length}")
                if angle>=90:
                    angle = 180 - angle
                    if angle <5:
                        angle = angle + 5
                    temp = left_pos[0]
                    left_pos[0] =right_pos[0]
                    right_pos[0] = temp
                print(f"使用右手掃桌，角度調整為: {angle}")
                print(f"left_pos: {left_pos}")
                print(f"right_pos: {right_pos}")
        
                dustpan_dis = dustpan_length
                dustpan_dis2= dustpan_dis + 15
                theta = math.radians((angle))
                brush_pos_close=right_pos
                dustpan_pos_close=left_pos
                while True:#[修改]
                    if dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta) >580:#[修改]
                        if angle <=5:#[修改]
                            print("無法調整至合適角度，請重新規劃動作")#[修改]
                            break
                        else:#[修改]
                            angle -=5#[修改]
                            theta = math.radians(angle)#[修改]

                    else:#[修改]
                        break#[修改]
                
                brush_target=[brush_pos_close[0]- size[1]/4 - brush_dis_offset* math.sin(theta), brush_pos_close[1] - brush_dis_offset* math.cos(theta), brush_pos_close[2]+brush_length]
                brush_target2=[dustpan_pos_close[0]-size[1]/4 + (dustpan_dis-LandR_dis)* math.sin(theta), dustpan_pos_close[1] + (dustpan_dis-LandR_dis)* math.cos(theta) , dustpan_pos_close[2]+brush_length] ####[修改]
                center_target=[center_pos[0], center_pos[1], center_pos[2]+brush_length]
                dustpan_target=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis* math.sin(theta), dustpan_pos_close[1] + dustpan_dis* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
                dustpan_target2=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta), dustpan_pos_close[1] + dustpan_dis2* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
                auxiliary_angle =angle
                print(f"brush_target: {brush_target}")
                print(f"dustpan_target: {dustpan_target}")
                print(f"brush_target2: {brush_target2}")
                print(f"dustpan_target2: {dustpan_target2}")
                print(f"auxiliary_angle: {auxiliary_angle}")
        else:
            object_pos = object.get('base_center_pos', None)
            pick_mode = object.get('pick_mode', None)   
        rospy.loginfo(f"[{arm}] 移動到物品 {object_name}")

        if object_name == 'rice food'or  'rice food' in object_name:
            if arm == self.active_arm:

                # 待測試
                if arm == "left":   
                    robot_control.dual_move(brush_target[0] , brush_target[1], brush_target[2]+60, "side", angle,  dustpan_target[0], dustpan_target[1], dustpan_target[2]+20, "down", sign*10)
                   
                    robot_control.dual_move(brush_target[0] , brush_target[1], brush_target[2], "side", angle,  dustpan_target[0], dustpan_target[1], dustpan_target[2]+20, "down", auxiliary_angle)
                    
                    robot_control.single_move(self.auxiliary_arm, dustpan_target[0], dustpan_target[1], dustpan_target[2], "down", auxiliary_angle)
                    robot_control.open_gripper(self.auxiliary_arm)
                    robot_control.single_move(self.auxiliary_arm, dustpan_target[0], dustpan_target[1], dustpan_target[2]-gripper_length, "down", auxiliary_angle)
                    robot_control.close_gripper_ang(self.auxiliary_arm, 100)
                else:
                    robot_control.dual_move(dustpan_target[0], dustpan_target[1], dustpan_target[2]+20, "down", sign*10, brush_target[0] , brush_target[1], brush_target[2]+60, "side", angle)
                    robot_control.dual_move(dustpan_target[0], dustpan_target[1], dustpan_target[2]+20, "down", auxiliary_angle, brush_target[0] , brush_target[1], brush_target[2], "side", angle)
                    robot_control.single_move(self.auxiliary_arm, dustpan_target[0], dustpan_target[1], dustpan_target[2], "down", auxiliary_angle)
                    robot_control.open_gripper(self.auxiliary_arm)
                    robot_control.single_move(self.auxiliary_arm, dustpan_target[0], dustpan_target[1], dustpan_target[2]-gripper_length, "down", auxiliary_angle)
                    robot_control.close_gripper_ang(self.auxiliary_arm, 100)
            
            
            if arm == self.auxiliary_arm:
                # 待測試
                print(f"輔助手移動")
                
        

    def sweep_towards(self, arm:str):
        for object in shared_object.total:
            name = object.get('name', 'unknown')
            if name == 'rice food' or (isinstance(name, list) and 'rice food' in name):
                # size = object.get('3d_size', None)
                size = object['3d_size'].copy()
                angle = object.get('angle', 0)
                left_pos = object['left_base_pos'].copy()
                right_pos = object['right_base_pos'].copy()
                center_pos = object['base_center_pos'].copy()
                
                if left_pos != None and right_pos != None and center_pos != None and size != None:
                    # 待測試
                    if size[2]<20:
                        if size[2]<13:
                            size[2] = 18
                        size[2] = size[2]+5


                    left_pos[2] = left_pos[2] - size[2]
                    right_pos[2] = right_pos[2] - size[2]
                    center_pos[2] = center_pos[2] - size[2]
                    
                    # if size[2]<20:
                    #     size[2] = 18
                    # left_pos[2] = left_pos[2] - size[2]*2
                    # right_pos[2] = right_pos[2] - size[2]*2
                    # center_pos[2] = center_pos[2] - size[2]*2
                    z_values = [center_pos[2], left_pos[2], right_pos[2]]
                    z_values.sort()
                    print(f"桌面高度原始值: {center_pos[2]} mm, {left_pos[2]} mm, {right_pos[2]} mm")
                    median_z = z_values[1]  # 排序後中間的值
                   
                    center_pos[2] = left_pos[2] = right_pos[2] = median_z
                    if center_pos[2] > -323:    
                        center_pos[2] = left_pos[2] = right_pos[2] = -323
                        print("桌面高度過高，統一調整為 -323 mm")
                    # center_pos[2] = left_pos[2] = right_pos[2] = max(center_pos[2], left_pos[2], right_pos[2])
                    print(f"桌面統一高度為: {center_pos[2]} mm")
                
        gripper_length = 23.5 #30   24    
        
        dustpan_length = 195  # 畚箕長度+距離offset假設為170 190 210 200mm
        brush_dis_offset = 75 #40 #55 # 60 # 掃把距離米的offset距離
        if (arm== "left" and self.active_arm == arm) or(arm == "right" and self.auxiliary_arm == arm):
            sign = 1
            side_angle=160
            print(f"dustpan_height: {shared_object.right[0]['3d_size']}")
            dustpan_height = shared_object.right[0]['3d_size'][2]
            if dustpan_height <32:
                dustpan_height = 32
            print(f"dustpan_height: {dustpan_height}") 
            print(f"longest_length: {shared_object.left[0]['longest_length']}")
            brush_length= shared_object.left[0]['longest_length']-19 # 15 17 20 #20 #25 #18
            print(f"brush_length: {brush_length}")
            if angle>=0 and angle<=90:
                if angle <5:
                    angle = angle + 5
                angle = 180 - angle
                temp = left_pos[0]
                left_pos[0] =right_pos[0] # 避免超過 motor4 angle +90~-90
                right_pos[0] = temp
            # right_angle = -(180 - angle)    
            print(f"left_pos: {left_pos}")
            print(f"right_pos: {right_pos}")
            
            dustpan_dis = dustpan_length
            dustpan_dis2= dustpan_dis + 15
            theta = math.radians((180-angle))
            brush_pos_close=left_pos
            dustpan_pos_close=right_pos
            LandR_dis = 125
            while True:#[修改]
                if dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta) >580:#[修改]
                    if angle >=175:#[修改]
                        print("無法調整至合適角度，請重新規劃動作")#[修改]
                        break
                    else:#[修改]
                        angle +=5#[修改]
                        theta = math.radians((180-angle))#[修改]

                else:#[修改]
                    break#[修改]
            print(f"角度調整為: {angle}")
            brush_target=[brush_pos_close[0]- size[1]/4 - brush_dis_offset* math.sin(theta), brush_pos_close[1] + brush_dis_offset* math.cos(theta), brush_pos_close[2]+brush_length]
            brush_target2=[dustpan_pos_close[0]-size[1]/4+ (dustpan_dis-LandR_dis)* math.sin(theta), dustpan_pos_close[1]-(dustpan_dis-LandR_dis)* math.cos(theta) , dustpan_pos_close[2]+brush_length] 
            center_target=[center_pos[0], center_pos[1], center_pos[2]+brush_length]
            dustpan_target=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis* math.sin(theta), dustpan_pos_close[1]-dustpan_dis* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
            dustpan_target2=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta), dustpan_pos_close[1]-dustpan_dis2* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
            auxiliary_angle =-(180 - angle)  
            print(f"brush_target: {brush_target}")
            print(f"dustpan_target: {dustpan_target}")
            print(f"brush_target2: {brush_target2}")
            print(f"dustpan_target2: {dustpan_target2}")
            print(f"auxiliary_angle: {auxiliary_angle}")
        else:
            sign = -1
            side_angle = 30 
            dustpan_height = shared_object.left[0]['3d_size'][2]
            if dustpan_height <30: #修改
                dustpan_height = 32 
            print(f"dustpan_height: {dustpan_height}") 
            print(f"longest_length: {shared_object.right[0]['longest_length']}")
            brush_length= shared_object.right[0]['longest_length']-18 # 15 17 20 #20 #25
            print(f"brush_length: {brush_length}")
            if angle>=90:
                angle = 180 - angle
                if angle <5:
                    angle = angle + 5
                temp = left_pos[0]
                left_pos[0] =right_pos[0]
                right_pos[0] = temp
            print(f"使用右手掃桌，角度調整為: {angle}")
            print(f"left_pos: {left_pos}")
            print(f"right_pos: {right_pos}")
    
            dustpan_dis = dustpan_length
            dustpan_dis2= dustpan_dis + 15
            theta = math.radians((angle))
            brush_pos_close=right_pos
            dustpan_pos_close=left_pos
            auxiliary_angle =angle
            while True:
                if dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta) >580:#[修改]
                    if angle <=5:
                        print("無法調整至合適角度，請重新規劃動作")#[修改]
                        break
                    else:#[修改]
                        angle -=5
                        theta = math.radians(angle)
                else:
                    break
            brush_target=[brush_pos_close[0]- size[1]/4 - brush_dis_offset* math.sin(theta), brush_pos_close[1] - brush_dis_offset* math.cos(theta), brush_pos_close[2]+brush_length]
            brush_target2=[dustpan_pos_close[0]-size[1]/4 + (dustpan_dis-LandR_dis)* math.sin(theta), dustpan_pos_close[1] + (dustpan_dis-LandR_dis)* math.cos(theta) , dustpan_pos_close[2]+brush_length] ####[修改]
            center_target=[center_pos[0], center_pos[1], center_pos[2]+brush_length]
            dustpan_target=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis* math.sin(theta), dustpan_pos_close[1] + dustpan_dis* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
            dustpan_target2=[dustpan_pos_close[0]- size[1]/4 + dustpan_dis2* math.sin(theta), dustpan_pos_close[1] + dustpan_dis2* math.cos(theta), dustpan_pos_close[2]+dustpan_height+gripper_length]
         
           
        print(f"brush_target: {brush_target}")
        print(f"dustpan_target: {dustpan_target}")
        print(f"brush_target2: {brush_target2}")
        print(f"dustpan_target2: {dustpan_target2}")
        print(f"auxiliary_angle: {auxiliary_angle}")
        if arm == self.active_arm:
            while True:
                user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
                if user_input == "1":
                    print("✓ 執行抓取菶積...")
                    break
                elif user_input.lower() == "q":
                    print("✗ 取消動作")
                    exit()
                else:
                    print("⚠ 請輸入 1 或 q")
            robot_control.single_move(self.active_arm, brush_target2[0] , brush_target2[1], brush_target2[2], "side", angle)
            robot_control.single_move(self.active_arm, brush_target2[0] , brush_target2[1], brush_target2[2]+45, "side", angle) #抬手
            robot_control.single_move(self.active_arm, center_target[0], center_target[1] , center_target[2]+45, "side", angle)
            robot_control.single_move(self.active_arm, center_target[0], center_target[1], center_target[2], "side", angle)
            robot_control.single_move(self.active_arm, brush_target2[0] , brush_target2[1], brush_target2[2], "side", angle)
            robot_control.single_move(self.active_arm, brush_target2[0] , brush_target2[1], brush_target2[2]+45, "side", angle)
            robot_control.single_move(self.active_arm, brush_target[0], brush_target[1] , brush_target[2]+45, "side", angle)
            robot_control.single_move(self.active_arm, 300, sign*130, -130 , "side", side_angle)
            
            robot_control.single_move(self.auxiliary_arm, dustpan_target2[0], dustpan_target2[1], dustpan_target2[2]-gripper_length, "down", auxiliary_angle)
            robot_control.open_gripper(self.auxiliary_arm)
            robot_control.close_gripper(self.auxiliary_arm)
            robot_control.single_move(self.auxiliary_arm, dustpan_target[0]-50, dustpan_target[1], dustpan_target[2]+20, "down", auxiliary_angle)
            robot_control.single_move(self.auxiliary_arm, dustpan_target[0]-50, dustpan_target[1], dustpan_target[2]+20, "down", sign*10)
            robot_control.single_move(self.auxiliary_arm, dustpan_target[0]-50, dustpan_target[1], dustpan_target[2]+20, "down", sign*90)
        else:
            print("輔助手執行")   
    
    def wait(self,arm:str):
        print(f"[{arm}] 等待中...")
    def sprinkle_pepper(self, arm: str):
        # robot_control.capture_publisher("close")
        global shared_object
        print("執行撒胡椒粉動作")
        yaw_angle = math.radians(40)
        if arm == "left":
            sign = 1
        elif arm == "right":
            sign = -1
        robot_control.single_arm_initial_position(arm)
        found_sandwich = False
        for object in shared_object.total:
            # name = object['name']
            name = object.get('name', 'unknown')
            if 'sandwich' in name:
                size = object.get('3d_size', None)      
                left_pos = object.get('left_base_pos', None)
                right_pos = object.get('right_base_pos', None)
                center_pos = object.get('base_center_pos', None)
                print(f"找到三明治，中心位置: {center_pos}，左位置: {left_pos}, 右位置： {right_pos}")
                print(f"三明治尺寸為: {size}")
                found_sandwich = True
            if "pepper shaker" in name:
                pepper_size = object.get('3d_size', None)
                print(f"胡椒粉尺寸為: {pepper_size}")
            else:
                pepper_size = [70,70, 110]
        if found_sandwich == False:
            print("未找到三明治，無法執行撒胡椒粉動作")
            return
        offset_height = 80
        move_dis = 50
        pepper_height = max(pepper_size)/2
        print(f"胡椒粉高度為: {pepper_height}")
        height_pos = [center_pos[0]+pepper_height*math.sin(yaw_angle), center_pos[1]+sign*pepper_height*math.cos(yaw_angle), center_pos[2]+offset_height+move_dis]
        print(f"灑胡椒粉高處位置: {height_pos}")
        
        low_pos = [center_pos[0], center_pos[1], center_pos[2]+offset_height]
        print(f"灑胡椒粉低處位置: {low_pos}")

        # easy demo: move arm to pepper position and sprinkle
        robot_control.single_move(arm, center_pos[0], center_pos[1], center_pos[2]+offset_height+move_dis, "side_reversal", 0)
        robot_control.single_move(arm, height_pos[0], height_pos[1] , height_pos[2] , "side_reversal", 90) # 向下到的角度
        robot_control.single_move(arm, low_pos[0], low_pos[1] , low_pos[2], "side_reversal", 150)
        robot_control.single_move(arm, center_pos[0]-100, center_pos[1], center_pos[2]+offset_height+move_dis, "side_reversal", 30)
        robot_control.single_arm_initial_position(arm)
        robot_control.single_arm_go_away(arm)      
    def close_sandwich(self, arm: str):
        global shared_object
        if arm == "left":
            sign = 1   
        elif arm == "right":
            sign = -1
        found_sandwich = False
        for object in shared_object.total:
            # name = object['name']
            name = object.get('name', 'unknown')
            if name == 'sandwich' or (isinstance(name, list) and 'sandwich' in name):
                size = object.get('3d_size', None)
                left_pos = object.get('left_base_pos', None)
                right_pos = object.get('right_base_pos', None)
                center_pos = object.get('base_center_pos', None)
                angle = object.get('angle', 0)
                print(f"找到三明治，中心位置: {center_pos}，左位置: {left_pos}, 右位置： {right_pos}")
                print(f"三明治尺寸為: {size}")
                found_sandwich = True
        
        if found_sandwich == False:
            print("未找到三明治，無法執行撒胡椒粉動作")
            return
        toast_height = 150
        offset_height = 40
        gripper_length = 23.5
        dis_to_move = gripper_length+ max(size)
        # dis_to_move = offset_height + gripper_length
        # 危險區域角度
        if angle >=90:
            if arm == "right":
                angle = 10
                theta =  math.radians(10)
            else:
                theta =  math.radians(180 - angle)
        else:
            if arm == "left":
                angle = 170
                theta =  math.radians(10)
            else: 
                theta = math.radians(angle)
        
        center_pos[0] -= 15 
        thickness = max(size)/2 
        place_pos = [center_pos[0]- thickness * math.sin(theta), center_pos[1] + sign * thickness * math.cos(theta), center_pos[2]]

        # place_pos = [center_pos[0]- thickness * math.cos(math.radians(theta)), center_pos[1] + sign * thickness * math.sin(math.radians(theta)), center_pos[2]]
        print(f"place_pos : {place_pos}")
        
        leave_pos = [place_pos[0] - dis_to_move * math.sin(theta), place_pos[1]+sign * dis_to_move * math.cos(theta), place_pos[2]+offset_height-5]
        print(f"離開位置: {leave_pos}")
        

        robot_control.single_move(arm, place_pos[0], place_pos[1], place_pos[2]+toast_height, "side", angle)
        
        robot_control.single_move(arm, place_pos[0], place_pos[1], place_pos[2]+toast_height, "side_down", angle)

        # robot_control.single_move(arm, place_pos[0], place_pos[1], place_pos[2]+offset_height, "side_down", angle)
    
        robot_control.single_move(arm, place_pos[0], place_pos[1], place_pos[2]+offset_height, "side_tilt", angle)
        

        robot_control.close_gripper_ang(arm, 100)

        robot_control.single_move(arm, leave_pos[0], leave_pos[1], leave_pos[2], "side_tilt", angle)

        
        robot_control.single_move(arm, leave_pos[0], leave_pos[1], leave_pos[2], "side", angle)

        robot_control.single_arm_initial_position(arm)
        
        robot_control.single_arm_initial_position(arm)

    def place(self, object_name: str, mode: str, angle: float, arm: str):
        print(f"[{arm}] 放置物品 {object_name}，模式: {mode}，角度: {angle}")
        # robot_control.neck_control(0, 76)
        find_object = False
        if arm == "left":
            if len(shared_object.left)>0:
                object = shared_object.left[0]
                object_name = object.get('name', None)
                find_object = True
            else:
                object = None
        elif arm == "right":
            if len(shared_object.right)>0:
                object = shared_object.right[0]
                object_name = object.get('name', None)
                find_object = True
            else:
                object = None
       
        if object == None:
            for obj in shared_object.total:
                if object_name in obj['name']:
                    find_object = True
                    object = obj
                    break        
        if find_object == False:
            rospy.loginfo(f"未找到物品 {object_name} 的資訊，無法執行抓取")
            return     
        object_pos = object.get('base_center_pos', None)
        pick_mode = object.get('pick_mode', None)
        size = object.get('3d_size', None)
        angle = object.get('angle', 0)
        object_name = object.get('name', 'unknown')
        if object_name == 'dustpan tool':
                if size[2]>32:
                    size[2] = 31.5-15 
                print(f"畚箕高度為: {size[2]}mm")
        if object_name == 'brush tool' or object_name == 'dustpan tool':
            robot_control.neck_control(0, 70)
        if "pepper shaker" in object_name:
            robot_control.neck_control(0, 45)
            if arm == "right":
                self.active_arm = "right"
                self.auxiliary_arm = "left"
                angle = 30
                print(f"微調後角度為: {angle}")
            else:
                self.active_arm = "left"
                self.auxiliary_arm = "right"
                angle = 150
                print(f"微調後角度為: {angle}")
        if object_pos and pick_mode and size:
            robot_control.single_arm_place(object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)
        
    # === 執行引擎 ===   
    def execute_step(self, step: ActionStep):
        global shared_object
        """
        執行單一步驟
        使用 ActionStep 物件直接提取參數，避免字串解析
        """
        # 根據動作類型從映射表中取得對應函式
        if step.action_type == ActionType.EYE_IN_HAND:
            func = self.action_map["arm_eyeInHand_camera_catch"]
            func(step.object_index, step.arm.value)
           
        elif step.action_type == ActionType.PICK:
            func = self.action_map["pick"]
            func(step.arm.value, step.object_name)
            if "pepper shaker" in step.object_name or  "sliced toast" in step.object_name:
                shared_object.record_pick(step.object_name, step.arm.value)
        elif step.action_type == ActionType.SWEEP:
            func = self.action_map["sweep_towards"]
            func(step.arm.value)
        elif step.action_type == ActionType.PLACE:
            func = self.action_map["place"]
            func(step.object_name, step.mode.value, step.angle, step.arm.value)

            shared_object.record_place(step.object_name, step.arm.value)
        elif step.action_type == ActionType.SPRINKLE_PEPPER:
            func = self.action_map["sprinkle_pepper"]
            func(step.arm.value)
        elif step.action_type == ActionType.CLOSE_SANDWICH:
            func = self.action_map["close_sandwich"]
            func(step.arm.value)
            shared_object.record_place('sliced toast', step.arm.value)
        elif step.action_type == ActionType.WAIT:
            func = self.action_map["wait"]
            func(step.arm.value)
        elif step.action_type == ActionType.MOVE:
            func = self.action_map["move_to_object"]
            func(step.arm.value, step.object_name)
        
        else:
            raise ValueError(f"未知的動作類型: {step.action_type}")
    
    def execute_plan(self, robot_plan: RobotPlan):
        """
        執行完整計畫
        依據雙手協調邏輯執行所有步驟
        """
        print(f"\n開始執行任務: {robot_plan.task_description}")
        print("=" * 60)
        
        # 方案 B: 交錯執行（根據 step_id 排序）
        all_steps = []
        for step in robot_plan.left_arm:
            all_steps.append(step)
        for step in robot_plan.right_arm:
            all_steps.append(step)
        all_steps.sort(key=lambda s: s.step_id)
        # --- 新增邏輯: 過濾重複的 SWEEP ---
        final_steps = []
        seen_sweep_ids = set()  # 用來記錄哪些 step_id 已經有掃地動作了

        for step in all_steps:
            if step.action_type == ActionType.SWEEP:
                # 如果這個 step_id 已經被記錄過有 SWEEP，就跳過這次 (去重)
                if step.step_id in seen_sweep_ids:
                    continue
                # 否則將此 step_id 加入已見集合
                seen_sweep_ids.add(step.step_id)
            
            final_steps.append(step)
            
        all_steps = final_steps
        # --------------------------------
        for step in all_steps:
            print(f"\n步驟 {step.step_id} [{step.arm.value}]: {step.action_type.value}")

            self.execute_step(step)
       

    def execute_plan_together(self, robot_plan: RobotPlan):
        
        """
        執行完整計畫
        自動偵測連續的 pick/place 動作並並行執行（如果是不同手臂）
        """
        print(f"\n開始執行任務: {robot_plan.task_description}")
        print("=" * 60)
        
        # 合併並排序所有步驟
        all_steps = []
        for step in robot_plan.left_arm:
            all_steps.append(step)
        for step in robot_plan.right_arm:
            all_steps.append(step)
        all_steps.sort(key=lambda s: s.step_id)
      
        
        found_eye_in_hand = False
        # 執行步驟（支援並行）
        for step in all_steps:
            if step.action_type == ActionType.EYE_IN_HAND:
                found_eye_in_hand = True

        i = 0
        while i < len(all_steps):
            current_step = all_steps[i]
            # 檢查下一步是否可以並行執行
            if i + 1 < len(all_steps):
                next_step = all_steps[i + 1]
                # 條件：連續兩步都是 pick 或 place，且使用不同手臂
                if self._can_execute_parallel(current_step, next_step):
                    print(f"\n🔄 並行執行步驟 {current_step.step_id} 和 {next_step.step_id}")
                    print(f"   [{current_step.arm.value}]: {current_step.action_type.value}")
                    print(f"   [{next_step.arm.value}]: {next_step.action_type.value}")
                    if found_eye_in_hand and current_step.action_type == ActionType.PICK:
                        print("   存在 EYE_IN_HAND 動作，暫停 15 秒等待相機辨識完成")
                        time.sleep(15) #等待相機辨識完全
                    # 建立兩個執行緒
                    thread1 = threading.Thread(
                        target=self.execute_step, 
                        args=(current_step,)
                    )
                    thread2 = threading.Thread(
                        target=self.execute_step, 
                        args=(next_step,)
                    )
                    # 同時啟動
                    thread1.start()
                    thread2.start()
                    # 等待兩個都完成
                    thread1.join()
                    thread2.join()
                    
                    print(f"✓ 步驟 {current_step.step_id} 和 {next_step.step_id} 完成")
                    # 跳過下一步（因為已經執行了）
                    i += 2
                else:
                    # 不能並行，單獨執行當前步驟
                    print(f"\n步驟 {current_step.step_id} [{current_step.arm.value}]: {current_step.action_type.value}")
                    self.execute_step(current_step)
                    i += 1
            else:
                # 最後一步，直接執行
                print(f"\n步驟 {current_step.step_id} [{current_step.arm.value}]: {current_step.action_type.value}")
                self.execute_step(current_step)
                i += 1
    

    def _can_execute_parallel(self, step1: ActionStep, step2: ActionStep) -> bool:
        """
        判斷兩個步驟是否可以並行執行
        
        條件：
        1. 兩步驟的 action_type 相同
        2. 都是 PICK 或 PLACE
        3. 使用不同的手臂
        """
        # 檢查動作類型是否相同
        if step1.action_type != step2.action_type:
            return False
        # 檢查是否為 PICK 或 PLACE
        if step1.action_type not in {ActionType.PICK, ActionType.PLACE}:
            return False
        # 檢查是否使用不同手臂
        if step1.arm == step2.arm:
            return False
        return True


# ==================================== Task planning ====================================

def generate_task_plan():
    global GPT_planner, robot_status

    max_retries = 5
    robot_plan = None
    passed = False

    robot_plan = GPT_planner.task_planning()    
    for attempt in range(max_retries):
        print(f"\n[{'='*20} 第 {attempt + 1} 次嘗試生成計畫 {'='*20}]")
        
        # 1. 呼叫 LLM 產生計畫
        robot_plan = GPT_planner.task_planning()
        print("生成的計畫:")
        print(json.dumps(robot_plan.model_dump(), indent=2, ensure_ascii=False))  # 注意：Pydantic V2 建議用 model_dump() 取代 dict()
        
        # 2. 靜態驗證
        validator = PlanValidator()
        passed, errors, penalty = validator.validate_plan(
            robot_plan, 
            robot_status.left_arm_status, 
            robot_status.right_arm_status
        )
        validator.print_report()
        
        # 3. 判斷是否通過
        if passed:
            print("✅ 計畫完美通過驗證！")
            break
        else:
            print(f"⚠️ 計畫未通過驗證（扣分: {penalty}）。")
            if attempt < max_retries - 1:
                print("🔄 將錯誤訊息回傳給 LLM，要求重新規劃...")
                
                # --- 將 Validator 的錯誤打包成字串，餵給 Planner ---
                error_msgs = "\n".join([
                    f"- [{e.error_type}] 步驟 {e.step_id} ({e.arm} arm): {e.description}" 
                    for e in errors
                ])
                GPT_planner.error_feedback_prompt = (
                    "你上次生成的計畫有以下致命錯誤，請重新思考並修正：\n" + error_msgs
                )
            else:
                print("❌ 已達最大重試次數，規劃失敗，請手動介入。")
                return False
        
    if passed and robot_plan:
        print("\n開始執行計畫...")
        plan_print(robot_plan)
        return robot_plan
def plan_print(robot_plan: RobotPlan):
    """執行計畫前的打印，展示所有步驟細節"""
    # 根據 step_id 順序執行
    all_steps = []
    for step in robot_plan.left_arm:
        all_steps.append((step, "left"))
    for step in robot_plan.right_arm:
        all_steps.append((step, "right"))
    
    # 按 step_id 排序（如果需要嚴格順序）
    all_steps.sort(key=lambda x: x[0].step_id)
    
    for step, arm in all_steps:
        func_call = step.to_function_call()
        print(f"[{arm}] 執行: {func_call}")



 
# 3. 測試清掃任務(待測試的部分)
if __name__ == '__main__': 
    ros_sub_init()
    executor = RobotExecutor()
    # robot_control.neck_control(0, 45)# 40
    # robot_control.neck_control(0, 0)# 40
    # 1. 手臂測試
    time.sleep(2)
    # draw_back_hands()
    print("開始測試...")
    shared_object.head_camera_ready = True
    