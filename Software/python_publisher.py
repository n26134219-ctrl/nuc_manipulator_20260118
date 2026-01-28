
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rospy
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


robot_control = CommandPublisher()
GPT_planner = GPTPlanner()

# load_dotenv()  # 讀取 .env 檔
# client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
# task_description_prompt = "請利用桌面上的掃把與畚箕，將桌上掃乾淨。"

total_objects_phrase_pub = rospy.Publisher('assign_object_phase', String, queue_size=10)
left_objects_phrase_pub = rospy.Publisher('assign_left_object_phase', String, queue_size=10)
right_objects_phrase_pub = rospy.Publisher('assign_right_object_phase', String, queue_size=10)

# received_base_positions = []  # 按順序儲存收到的基座標
class SharedObject:
    def __init__(self):
        self.total = []  # 總物體列表
        self.left = []   # 左側物體列表
        self.right = []  # 右側物體列表
        self.other = []  # 其他物體列表
        self.head_camera_ready = False
        self.left_camera_ready = False
        self.right_camera_ready = False

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
def camera_ready_callback(msg):
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
    category = msg.data
    if category == "cleaning":
        get_env_info()
        update_camera_prompt()

def ros_sub_init():
    rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    rospy.Subscriber('task_explanation', String, task_explanation_callback)
    rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    rospy.Subscriber('/camera/left_objects', String, left_objects_callback)
    rospy.Subscriber('/camera/right_objects', String, right_objects_callback)
    rospy.Subscriber('/camera/camera_ready', String, camera_ready_callback)
    rospy.Subscriber('task_type', String, task_type_callback)

    # rospy.spin()
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
        if pos:
            GPT_planner.camera_information_prompt += f"object_name: {name} \n"
            GPT_planner.camera_information_prompt += f"object_index: {idx} \n"
            GPT_planner.camera_information_prompt += f"object_position: px={pos[0]:.1f}mm, py={pos[1]:.1f}mm, pz={pos[2]:.1f}mm \n"
            GPT_planner.camera_information_prompt += f"object_angle: {angle:.1f} deg \n"
            GPT_planner.camera_information_prompt += f"pick_mode: {pick_mode} \n\n"

    GPT_planner.camera_information_prompt += "===============================\n"
            
    print("更新相機資訊提示詞:")
    print(GPT_planner.camera_information_prompt)


        
def draw_back_hands():
    time.sleep(3)
    robot_control.dual_move(266.3, 80, -230, "side", -10, 266.3, -80, -230, "side", 10)

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






   

# ==================================== camera search functions ====================================

def update_search_phrase(phrase):
    total_objects_phrase_pub.publish(phrase)
    time.sleep(2)

def search_trush():
    robot_control.capture_publisher("head")
    time.sleep(15)
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):
            if obj['name'] == "rice food":
                if obj['3d_size'][0] >  300 or obj['3d_size'][1] >  300 or obj['base_center_pos'][0] > 650:
                    rospy.loginfo("垃圾尺寸過大，無法拾取")
                    return False
                rospy.loginfo(f"找到垃圾，索引為: {index}")
                return True
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
def get_env_info():
    global shared_object
    robot_control.initial_position()
    if shared_object.head_camera_ready == True:
        while search_dustpan_broom() == False:
            robot_control.neck_control(0, 76)
    time.sleep(2)
    shared_object.head_camera_ready = False

    draw_back_hands()

    robot_control.neck_control(0, 45)
    update_search_phrase("rice food")
    if shared_object.head_camera_ready == True:
        while search_trush() == False:
            robot_control.neck_control(0, 45)
    time.sleep(2)
    shared_object.head_camera_ready = False

    robot_control.neck_control(0, 76)
    print("================= 獲取環境資訊完成 =================")
    show_info(shared_object.total)
    robot_control.initial_position()



# step1 :語音-> 更新相機phrase -> 判斷是否為清掃任務
# step2 : get_env_info -> 更新物品資訊
# step3 : update_camera_prompt ＝>根據物品資訊與任務需求，產生動作規劃 =>gpt
# step4 : 執行動作規劃& vlm check



# ==================================== 動作函式呼叫範例 ====================================


class RobotExecutor:
    """機器人動作執行器"""
    
    def __init__(self):
        # 建立函式名稱到實際函式的映射表
        self.action_map = {
            "arm_eyeInHand_camera_catch": self.arm_eyeInHand_camera_catch,
            "pick": self.pick,
            "sweep_the_table": self.sweep_the_table,
            "place": self.place
        }
        self.active_arm = "left"
    
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
            # while True:
            #     if arm == "left":
            #         objects = shared_object.left

            #         sign = 1
            #     elif arm == "right":
            #         objects = shared_object.right
                    
            #         sign = -1
            #     if len(objects)>0:
            #         arm_object_pos = objects[0].get('base_center_pos', None)
            #         if arm_object_pos:
            #                 y = arm_object_pos[1]
            #                 if y * sign > -60:
            #                     break
                


    
    

    # def pick(self, object_index: int, pick_mode: str, angle: float, arm: str):
    def pick(self, arm: str):
        # get object information
        if arm == "left":
            object = shared_object.left[0]
        elif arm == "right":
            object = shared_object.right[0]
        object_pos = object.get('base_center_pos', None)
        pick_mode = object.get('pick_mode', None)
        size = object.get('3d_size', None)
        angle = object.get('angle', 0)
        object_name = object.get('name', 'unknown')
        rospy.loginfo(f"[{arm}] 抓取物品 {object_name}，模式: {pick_mode}，角度: {angle}")
        if object_pos and pick_mode and size: 
            if object_name == 'dustpan tool':
                # size[2] = size[2] *2 # *2 *1 
                if size[2]>32:
                    size[2] = 31.5-15 
                print(f"畚箕高度為: {size[2]}mm")
            
            if object_name == 'brush tool':
                if arm == "right":
                    self.active_arm = "right"
                size[2] = size[2] # *2 *1 
            robot_control.single_arm_pick( object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)

            
   

    def sweep_the_table(self):
        """執行掃桌動作"""
        print("執行掃桌動作")
        for object in shared_object.total:
            name = object['name']
            if name == 'rice food':
                # pos = object.get('base_center_pos', None)
                size = object.get('3d_size', None)
                angle = object.get('angle', 0)
                left_pos = object.get('left_base_pos', None)
                right_pos = object.get('right_base_pos', None)
                center_pos = object.get('base_center_pos', None)
                if left_pos != None and right_pos != None and center_pos != None and size != None:
                    left_pos[2] = left_pos[2] - size[2]*2
                    right_pos[2] = right_pos[2] - size[2]*2
                    center_pos[2] = center_pos[2] - size[2]*2
                    z_values = [center_pos[2], left_pos[2], right_pos[2]]
                    z_values.sort()
                    median_z = z_values[1]  # 排序後中間的值
                   
                    center_pos[2] = left_pos[2] = right_pos[2] = median_z

                    # center_pos[2] = left_pos[2] = right_pos[2] = max(center_pos[2], left_pos[2], right_pos[2])
                    print(f"桌面統一高度為: {center_pos[2]} mm")
                

        gripper_length = 23.5 #30   24    
        brush_length = 110  # 掃把握柄中心到尾端長度為125 105mm
        dustpan_length = 185  # 畚箕長度假設為170 190 210 200mm
        
        dis = 125  
        robot_control.neck_control(0, 45)
        if self.active_arm == "left":
            print(f"dustpan_height: {shared_object.right[0]['3d_size']}")
            dustpan_height = shared_object.right[0]['3d_size'][2]
            if dustpan_height <22:
                dustpan_height += 22
            print(f"dustpan_height: {dustpan_height}") 
            print(f"longest_length: {shared_object.left[0]['longest_length']}")
            brush_length= shared_object.left[0]['longest_length']-15 # 15 17 20 #20 #25
            print(f"brush_length: {brush_length}")
            if angle>=0 and angle<=90:
                if angle <5:
                    angle = angle + 5
                angle = 180 - angle
                tem = left_pos
                left_pos[0] =right_pos[0] # 避免超過 motor4 angle +90~-90
                right_pos[0] = tem[0]
            right_angle = -(180 - angle)    
            print(f"使用左手掃桌，角度調整為: {angle}")
            print(f"left_pos: {left_pos}")
            print(f"right_pos: {right_pos}")
            left_dis = 55 #40
            right_dis = dustpan_length
            theta = math.radians((180-angle))
            left_target=[left_pos[0]-size[1]/2- left_dis* math.sin(theta), left_pos[1]+left_dis* math.cos(theta), left_pos[2]+brush_length]
            center_target=[center_pos[0], center_pos[1], center_pos[2]+brush_length]
            right_target=[right_pos[0]-size[1]/2+ right_dis* math.sin(theta), right_pos[1]-right_dis* math.cos(theta), right_pos[2]+dustpan_height+gripper_length]
            print(f"left_target: {left_target}")
            print(f"right_target: {right_target}")
            # while True:
            #     user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            #     if user_input == "1":
            #         print("✓ 繼續執行掃地...")
            #         break
            #     elif user_input.lower() == "q":
            #         print("✗ 取消動作")
            #         exit()
            #     else:
            #         print("⚠ 請輸入 1 或 q")
            robot_control.single_move("left", left_target[0]-50, left_target[1], left_target[2]+60, "side", angle)
            robot_control.single_move("left", left_target[0], left_target[1], left_target[2]+60, "side", angle)
            robot_control.single_move("left", left_target[0], left_target[1], left_target[2], "side", angle)
            
            robot_control.single_move("right", right_target[0]-50, right_target[1], right_target[2]+20, "down", 90)
            robot_control.single_move("right", right_target[0]-50, right_target[1], right_target[2]+20, "down", right_angle)
            
            robot_control.single_move("right", right_target[0], right_target[1], right_target[2], "down", right_angle)
            robot_control.open_gripper("right")
            robot_control.single_move("right", right_target[0], right_target[1], right_target[2]-gripper_length, "down", right_angle)
            # robot_control.close_gripper("right")
            # while True:
            #     user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            #     if user_input == "1":
            #         print("✓ 執行掃地...")
            #         break
            #     elif user_input.lower() == "q":
            #         print("✗ 取消動作")
            #         exit()
            #     else:
            #         print("⚠ 請輸入 1 或 q")
            
            # move right and down at down boundary
            robot_control.single_move("left", right_target[0] - dis* math.sin(theta), right_target[1]+dis* math.cos(theta), left_target[2], "side", angle)
            robot_control.single_move("left", right_target[0] - dis* math.sin(theta), right_target[1]+dis* math.cos(theta), left_target[2]+45, "side", angle)



            robot_control.single_move("left", center_target[0], center_target[1] , center_target[2]+45, "side", angle)
            robot_control.single_move("left", center_target[0], center_target[1], center_target[2], "side", angle)
            robot_control.single_move("left", right_target[0] - dis* math.sin(theta), right_target[1]+dis* math.cos(theta), left_target[2], "side", angle)
            robot_control.single_move("left", right_target[0] - dis* math.sin(theta), right_target[1]+dis* math.cos(theta), left_target[2]+45, "side", angle)

            robot_control.single_move("left", left_target[0], left_target[1] , left_target[2]+45, "side", angle)
            robot_control.single_move("left", 300, 130, -130 , "side", 160)
            # while True:
            #     user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            #     if user_input == "1":
            #         print("✓ 執行右手抓取菶積...")
            #         break
            #     elif user_input.lower() == "q":
            #         print("✗ 取消動作")
            #         exit()
            #     else:
            #         print("⚠ 請輸入 1 或 q")
            robot_control.close_gripper("right")
            robot_control.single_move("right", right_target[0]-50, right_target[1], right_target[2]+20, "down", right_angle)
            robot_control.single_move("right", right_target[0]-50, right_target[1], right_target[2]+20, "down", 90)
                       
        else:
            brush_length= shared_object.right[0]['longest_length']
            if angle>=90:
                angle = 180 - angle
                if angle <5:
                    angle = angle + 5
                tem = left_pos
                left_pos[0] =right_pos[0]
                right_pos[0] = tem[0]



            print(f"使用右手掃桌，角度調整為: {angle}")
            print(f"left_pos: {left_pos}")
            print(f"right_pos: {right_pos}")
            while True:
                user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
                if user_input == "1":
                    print("✓ 繼續執行掃地...")
                    break
                elif user_input.lower() == "q":
                    print("✗ 取消動作")
                    exit()
                else:
                    print("⚠ 請輸入 1 或 q")

            robot_control.single_move("right", right_pos[0]-size[1]/2-50, right_pos[1]-30, right_pos[2]+brush_length+60, "side", angle)
            robot_control.single_move("right", right_pos[0]-size[1]/2, right_pos[1]-30, right_pos[2]+brush_length, "side", angle)
            
            robot_control.single_move("left", left_pos[0]-size[1]/2-50, left_pos[1]+dustpan_length, left_pos[2]+ dustpan_height+60, "down", -90)
            robot_control.single_move("left", left_pos[0]-size[1]/2-50, left_pos[1]+dustpan_length, left_pos[2]+ dustpan_height+60, "down", angle)
            robot_control.single_move("left", left_pos[0]-size[1]/2, left_pos[1]+dustpan_length, left_pos[2]+ dustpan_height, "down", angle)
            robot_control.open_gripper("left")
            robot_control.close_gripper("left")
            while True:
                user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
                if user_input == "1":
                    print("✓ 繼續執行掃地...")
                    break
                elif user_input.lower() == "q":
                    print("✗ 取消動作")
                    exit()
                else:
                    print("⚠ 請輸入 1 或 q")
            
            
            

            # move left and down at down boundary
            robot_control.single_move("right", left_pos[0]-size[1]/2, left_pos[1]+dustpan_length-dis, left_pos[2]+brush_length, "side", angle)

            robot_control.single_move("right", right_pos[0], right_pos[1]-30 , right_pos[2]+brush_length+45, "side", angle)
            robot_control.single_move("left", left_pos[0], left_pos[1]+dustpan_length, left_pos[2]+ dustpan_height+30, "down", angle)

            robot_control.single_move("right", right_pos[0], right_pos[1]-30, right_pos[2]+brush_length, "side", angle)
            robot_control.single_move("left", left_pos[0], left_pos[1]+dustpan_length, left_pos[2]+ dustpan_height, "down", angle)
            # move left and down at center
            robot_control.single_move("right", left_pos[0], left_pos[1]+dustpan_length-dis, left_pos[2]+brush_length, "side", angle)
            robot_control.single_move("right", right_pos[0], right_pos[1]-30 , right_pos[2]+brush_length+45, "side", angle)


            robot_control.single_move("right", 400, -130, -130, "side", angle)  
            robot_control.single_move("left", left_pos[0]-size[1]/2-60, 130, left_pos[2]+ dustpan_height+60, "down", angle)
            robot_control.single_move("left", left_pos[0]-size[1]/2-60, 130, left_pos[2]+ dustpan_height+60, "down", -90)
            # robot_control.single_move("right", left_pos[0], left_pos[1]+dustpan_length/3, left_pos[2]+brush_length*2/3, "side", angle)
            # robot_control.single_move("right", right_pos[0], right_pos[1]-30 , right_pos[2]+brush_length+30, "side", angle)

        # 實際執行 ROS 掃桌動作
        # self.sweep_service.call()
    
    def place(self, object_index: int, mode: str, angle: float, arm: str):
        print(f"[{arm}] 放置物品 {object_index}，模式: {mode}，角度: {angle}")
        robot_control.neck_control(0, 76)
        if arm == "left":
            object = shared_object.left[0]
        elif arm == "right":
            object = shared_object.right[0]
        object_pos = object.get('base_center_pos', None)
        pick_mode = object.get('pick_mode', None)
        size = object.get('3d_size', None)
        angle = object.get('angle', 0)
        object_name = object.get('name', 'unknown')
        if object_name == 'dustpan tool':
                # size[2] = size[2] *2 # *2 *1 
                if size[2]>32:
                    size[2] = 31.5-15 
                print(f"畚箕高度為: {size[2]}mm")
        if object_pos and pick_mode and size:
            robot_control.single_arm_place(object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)
        
    # === 執行引擎 ===
    
    def execute_step(self, step: ActionStep):
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
            func(step.arm.value)
        
        elif step.action_type == ActionType.SWEEP:
            func = self.action_map["sweep_the_table"]
            func()
        
        elif step.action_type == ActionType.PLACE:
            func = self.action_map["place"]
            func(step.object_index, step.mode.value, step.angle, step.arm.value)
        
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
            # while True:
            #     user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            #     if user_input == "1":
            #         print("✓ 繼續執行...")
            #         self.execute_step(step)
            #         break
            #     elif user_input.lower() == "q":
            #         print("✗ 取消動作")
            #         exit()
            #     else:
            #         print("⚠ 請輸入 1 或 q")




# ==================================== Task planning ====================================


def generate_task_plan():
    global GPT_planner
    
   
    while True:
        robot_plan = GPT_planner.task_planning()    
        print("生成的計畫:")
        # print(json.dumps(robot_plan.dict(), indent=2, ensure_ascii=False))
        print(json.dumps(robot_plan.model_dump(), indent=2, ensure_ascii=False))
        # 2. 靜態驗證
        validator = PlanValidator()
        passed, errors, penalty = validator.validate_plan(robot_plan)
        validator.print_report()
        if not passed:
            print(f"⚠️  計畫未通過驗證（扣分: {penalty}），需要重新規劃")
        else:
            print("✅  計畫通過驗證")
            break

        # 可以選擇：
        # - 重新呼叫 GPT 並附上錯誤訊息
        # - 使用規則自動修正
        return False
    return robot_plan

     
if __name__ == '__main__': #1.菶機不夠後退   5. 菶積在抓一次會不平 
    ros_sub_init()
    executor = RobotExecutor()
    # 1. 手臂測試
    time.sleep(2)
    
    print("開始測試...")
    shared_object.head_camera_ready = True
    # while True:
    #         user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
    #         if user_input == "1":
    #             print("✓ 繼續執行...")
    #             time.sleep(2)
    #             # robot_control.single_move("left", 300, 130, -130 , "side", 160)
    #             # robot_control.open_gripper("left")
    #             # robot_control.single_move("left", 250, 130, -350 , "down", -90) #-350
    #             # robot_control.close_gripper("left")
    #             # robot_control.single_move("left", 250, 130, -250 , "down", -90) #-350
    #             # robot_control.single_move("right", 480, -350, -300 , "side", 50) #-350
    #             robot_control.close_gripper("right")
    #             # robot_control.single_move("right", 480, -350, -300 , "side", 50)
    #             break 
    #         elif user_input.lower() == "q":
    #             print("✗ 取消動作")
    #             exit()
    #         else:
    #             print("⚠ 請輸入 1 或 q")
    
    # 7. 完整計畫測試
    get_env_info()
    # while True:
    #         user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
    #         if user_input == "1":
    #             print("✓ 繼續執行...")
    #             update_camera_prompt()
    #             break 
    #         elif user_input.lower() == "q":
    #             print("✗ 取消動作")
    #             exit()
    #         else:
    #             print("⚠ 請輸入 1 或 q")
    update_camera_prompt()
    robot_plan = generate_task_plan()
    if robot_plan:

        executor.execute_plan(robot_plan)

    rospy.spin()
