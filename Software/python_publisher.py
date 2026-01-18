
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
                        # 找到了！更新它
                        shared_object.total[idx] = new_item
                        rospy.loginfo(f'更新物體資訊: {new_item["name"]}')
                        matched = True
                        break
            # 5. 沒找到，這是新物體，加入清單
            if not matched:
                shared_object.total.append(new_item)
        show_info(shared_object.total)
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
    robot_control.dual_move(266.3, 80, -230, "side", 10, 266.3, -80, -230, "side", 10)

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
        if name == 'dustpan tool' or name == 'brush tool':
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
    time.sleep(15)
    if len(shared_object.total) > 0:
        for index, obj in enumerate(shared_object.total):
            if obj['name'] == "dustpan" or obj['name'] == "broom":
                if obj['3d_size'][0] >  300 or obj['3d_size'][1] >  300 or obj['3d_size'][2] >  300:
                    rospy.loginfo("掃把或畚箕尺寸過大，無法拾取")
                    return False
                return True             
def get_env_info():
    global shared_object
    robot_control.initial_position()
    if shared_object.head_camera_ready == True:
        while search_dustpan_broom() == False:
            robot_control.neck_control(0, 76)
    time.sleep(2)
    shared_object.head_camera_ready = False

    draw_back_hands()

    robot_control.neck_control(0, 42)
    update_search_phrase("rice food")
    if shared_object.head_camera_ready == True:
        while search_trush() == False:
            robot_control.neck_control(0, 42)
    time.sleep(2)
    shared_object.head_camera_ready = False

    robot_control.neck_control(0, 76)
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
            
    
    

    # def pick(self, object_index: int, pick_mode: str, angle: float, arm: str):
    def pick(self, arm: str):
        # get object information
        if arm == "left":
            object = shared_object.left[int(object_index)]
        elif arm == "right":
            object = shared_object.right[int(object_index)]
        object_pos = object.get('base_center_pos', None)
        pick_mode = object.get('pick_mode', None)
        size = object.get('3d_size', None)
        angle = object.get('angle', 0)
        object_name = object.get('name', 'unknown')
        rospy.loginfo(f"[{arm}] 抓取物品 {object_name}，模式: {pick_mode}，角度: {angle}")
        if object_pos and pick_mode and size: 
            robot_control.single_arm_pick( object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)
        if object_name == 'brush tool':
            if arm == "right":
                self.active_arm = "right"

            
        # if object_name == 'dustpan tool':
            
        # elif object_name == 'brush tool':
            # robot_control.single_move(arm, 350, +-?120, -130,  "side", 10)

    
    def sweep_the_table(self):#未做完
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
        brush_length = 130  # 掃把握柄中心到尾端長度為130mm
        dustpan_length = 170  # 畚箕長度假設為170mm
        dustpan_height = 30  # 畚箕高度假設為30mm
        if self.active_arm == "left":
            
            robot_control.single_move("left", left_pos[0], left_pos[1]+30, left_pos[2]+brush_length, "side", angle)
            robot_control.single_move("right", right_pos[0], right_pos[1]-dustpan_length, right_pos[2]+dustpan_height, "down", angle)

            # move right and down
            robot_control.single_move("left", right_pos[0], right_pos[1]-dustpan_length/3, right_pos[2]+brush_length-15, "side", angle)
            robot_control.single_move("left", left_pos[0], left_pos[1]+30 , left_pos[2]+brush_length+30, "side", angle)

            # robot_control.single_move("left", right_pos[0], right_pos[1]-dustpan_length/3, right_pos[2]+brush_length*2/3, "side", angle)
            # robot_control.single_move("left", left_pos[0], left_pos[1]+30 , left_pos[2]+brush_length+30, "side", angle)
                        
        else:
            robot_control.single_move("right", right_pos[0], right_pos[1]-30, right_pos[2]+brush_length, "side", angle)
            robot_control.single_move("left", left_pos[0], left_pos[1]+dustpan_length, left_pos[2]+ dustpan_height, "down", angle)
            # move left and down
            robot_control.single_move("right", left_pos[0], left_pos[1]+dustpan_length/3, left_pos[2]+brush_length-15, "side", angle)
            robot_control.single_move("right", right_pos[0], right_pos[1]-30 , right_pos[2]+brush_length+30, "side", angle)

            # robot_control.single_move("right", left_pos[0], left_pos[1]+dustpan_length/3, left_pos[2]+brush_length*2/3, "side", angle)
            # robot_control.single_move("right", right_pos[0], right_pos[1]-30 , right_pos[2]+brush_length+30, "side", angle)

        # 實際執行 ROS 掃桌動作
        # self.sweep_service.call()
    
    def place(self, object_index: int, mode: str, angle: float, arm: str):
        print(f"[{arm}] 放置物品 {object_index}，模式: {mode}，角度: {angle}")
        if arm == "left":
            object = shared_object.left[0]
        elif arm == "right":
            object = shared_object.right[0]
        object_pos = object.get('base_center_pos', None)
        pick_mode = object.get('pick_mode', None)
        size = object.get('3d_size', None)
        angle = object.get('angle', 0)
        if object_pos and pick_mode and size:
            robot_control.single_arm_place(object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)
        robot_control.initial_position()    
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
        
        # # 方案 A: 依序執行（先左手後右手）
        # print("\n【左手動作】")
        # for step in robot_plan.left_arm:
        #     print(f"\n步驟 {step.step_id}:")
        #     self.execute_step(step)
        
        # print("\n【右手動作】")
        # for step in robot_plan.right_arm:
        #     print(f"\n步驟 {step.step_id}:")
        #     self.execute_step(step)
        
        # 方案 B: 交錯執行（根據 step_id 排序）
        all_steps = []
        for step in robot_plan.left_arm:
            all_steps.append(step)
        for step in robot_plan.right_arm:
            all_steps.append(step)
        all_steps.sort(key=lambda s: s.step_id)
        
        for step in all_steps:
            print(f"\n步驟 {step.step_id} [{step.arm.value}]:")

            # self.execute_step(step)
            while True:
                user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
                if user_input == "1":
                    print("✓ 繼續執行...")
                    self.execute_step(step)
                    break
                elif user_input.lower() == "q":
                    print("✗ 取消動作")
                    exit()
                else:
                    print("⚠ 請輸入 1 或 q")

# === 使用範例 ===


# # 建立測試計畫
# robot_plan = RobotPlan(
#     task_description="清理桌面",
#     left_arm=[
#         ActionStep(
#             step_id=1,
#             action_type=ActionType.EYE_IN_HAND,
#             arm=ArmType.LEFT,
#             object_index=1,
#             prerequisites=[]
#         ),
#         ActionStep(
#             step_id=2,
#             action_type=ActionType.PICK,
#             arm=ArmType.LEFT,
#             object_index=1,
#             mode=PickMode.DOWN,
#             angle=0.0,
#             prerequisites=[1]
#         )
#     ],
#     right_arm=[
#         ActionStep(
#             step_id=3,
#             action_type=ActionType.SWEEP,
#             arm=ArmType.RIGHT,
#             prerequisites=[2]
#         )
#     ]
# )

# # 執行計畫
# executor.execute_plan(robot_plan)







# ==================================== Task planning ====================================


def generate_task_plan():
    global GPT_planner
    robot_plan = GPT_planner.task_planning()    
    print("生成的計畫:")
    print(json.dumps(robot_plan.dict(), indent=2, ensure_ascii=False))
    
    # 2. 靜態驗證
    validator = PlanValidator()
    passed, errors, penalty = validator.validate_plan(robot_plan)
    validator.print_report()
    if not passed:
        print(f"⚠️  計畫未通過驗證（扣分: {penalty}），需要重新規劃")
        # 可以選擇：
        # - 重新呼叫 GPT 並附上錯誤訊息
        # - 使用規則自動修正
        return False
    return robot_plan


if __name__ == '__main__':
    ros_sub_init()
    executor = RobotExecutor()
    # 1. 手臂測試
    time.sleep(3)
    
    print("開始測試...")
    get_env_info()
    # robot_control.single_move("left", 400, 180, -230 , "down", -90)
    # robot_control.single_move("left", 400, 180, -130, "side", -20) #極限
    # robot_control.single_move("left", 250, 100, -140 , "down", -90)
    # robot_control.capture_publisher("left")
    # robot_control.single_move("left", 400, 180, -250 , "down", -90)
    # robot_control.single_move("left", 580, 180, -130, "side", -20) #極限
    # rospy.spin()
    # robot_control.single_move("right", 400, -150, -250 , "down", 90)
    # robot_control.single_move("right", 500, -120, -250, "side", 15)
    
    # robot_control.single_move("right", 580, -120, -130, "side", 15) #極限
    # robot_control.initial_position()
    
    robot_control.arms_camera_capture(250+57.5/2, 100, -304 , "down", "left")
    while True:
            user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            if user_input == "1":
                print("✓ 繼續執行...")
                # executor.arm_eyeInHand_camera_catch(, arm: str)
                executor.pick("left")
                break
            elif user_input.lower() == "q":
                print("✗ 取消動作")
                exit()
            else:
                print("⚠ 請輸入 1 或 q")
    
    # # robot_control.capture_publisher("left")
    # # robot_control.single_move("left", 400, 180, -250 , "down", -90)
    while True:
            user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
            if user_input == "1":
                print("✓ 繼續執行...")
                executor.sweep_the_table()
                break
            elif user_input.lower() == "q":
                print("✗ 取消動作")
                exit()
            else:
                print("⚠ 請輸入 1 或 q")
    # # robot_control.single_move("left", 450, 180, -130, "side", -20) #極限
    # while True:
    #         user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
    #         if user_input == "1":
    #             print("✓ 繼續執行...")
    #             robot_control.single_move("left", 450, 180, -130, "side", -20) #極限
    #             break
    #         elif user_input.lower() == "q":
    #             print("✗ 取消動作")
    #             exit()
    #         else:
    #             print("⚠ 請輸入 1 或 q")
    
    rospy.spin()
    
    


    # 5. 掃桌測試
    # executor.sweep_the_table()

    # 6. 放置測試
    # executor.place(0, "down", -90, "left")
    # executor.place(0, "down", 90, "right")
    # 7. 完整計畫測試
    # get_env_info()
    # update_camera_prompt()
    # robot_plan = generate_task_plan()
    # if robot_plan:
    #     executor.execute_plan(robot_plan)


    # 發送不同的命令
    # rospy.Subscriber('/base/object_point', Point, base_callback)
    # rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    # rospy.Subscriber('/camera/left_objects', String, left_objects_callback)
    # rospy.Subscriber('/camera/right_objects', String, right_objects_callback)
    # rospy.Subscriber('action_state', String, action_state_callback)
    # rospy.Subscriber('camera_search_control', String, camera_search_callback)
    # draw_back_hands()
    # robot_control.neck_control(0, 42)
    
    # get_env_info()
    # robot_control.capture_publisher("left")
    # robot_control.capture_publisher("head")
    # robot_control.capture_publisher("right")
    

    # robot_control.initial_position()
    # wait_for_robot_action_completion()

    # robot_control.single_move("left", 280, 150, -200 , "down", 90) # -8.53526 -45.2531  31.2841 -98.8004 -86.6848  14.2207
    # robot_control.single_move("left", 280, 150, -230 , "down", 90) # -2.50993 -44.9466  31.2317 -93.5691 -85.7203   13.831
    # robot_control.single_move("left", 280, 150, -280 , "down", 90) # 7.68809  -44.743  30.5737 -83.7688 -83.2515  14.0285
    # robot_control.single_move("left", 300, 150, -280 , "down", 90) # 7.41276 -42.9546  25.5973 -84.4193 -83.1228  17.3947（不能太高）



    # robot_control.single_move("left", 350, 150, -250, "side", 150)  #47.2661 -68.1573  11.5913 -34.4207   39.676  70.2017
    
    # robot_control.single_move("left", 350, 150, -200, "side", 150) #41.2828 -74.2487  19.7827 -34.2197  35.9402  65.9149
    # robot_control.neck_control(0, 56)
    # wait_for_robot_action_completion()
    # robot_control.single_move("right", 350, -200, -130-40, "side", 10)
    # wait_for_robot_action_completion()
    # robot_control.close_gripper("both")
    # robot_control.single_move("right", 266.3544006347656-20, -157.84457397460938, -304.7738037109375+170, "down", 90)
    # wait_for_robot_action_completion()
    # robot_control.capture_publisher("right")
    # robot_control.single_move("left", 284.922607421875, 58.331573486328125+30, -304.58917236328125+100, "down", 90)
    # time.sleep(2)
    # search_trush()
    # robot_control.single_move("left", 240.49017333984375, 110.89153289794922, -301.78619384765625+170, "down", 90)
    # robot_control.single_move("left", 240.01659628358166, 131.83216277406916+13, -358.55397651681096130+15, "down", -24.378479306439857)


    # robot_control.single_move("right", 230.49917602539062, -124.79796600341797, -313.226806640625+170, "down", 90)
    # robot_control.single_move("right", 222.95776722347003, -190.0875250356836, -375.43046578598205+100, "down", 90)
    # wait_for_robot_action_completion()
    # robot_control.single_move("right", 222.95776722347003, -190.0875250356836, -375.43046578598205+30, "down", -151.02540788414407)
    # wait_for_robot_action_completion()
    # robot_control.close_gripper("right")
    # while True:
    #         user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
    #         if user_input == "1":
    #             print("✓ 繼續執行...")
    #             break
    #         elif user_input.lower() == "q":
    #             print("✗ 取消動作")
    #             exit()
    #         else:
    #             print("⚠ 請輸入 1 或 q")

    # draw_back_hands()
    # robot_control.open_gripper("right")
    # time.sleep(2)
    # robot_control.single_move("left", 249.082744508674, 116.21285488773877, -356.13556303439213+30, "down", -22.54971061047786)

    # robot_control.single_move("left", 286.3, 80, -195.2, "side", 0)
    # robot_control.neck_control(0, 58)
    

    # robot_control.single_move("right", 230.49917602539062, -124.79796600341797, -313.226806640625, "down", 90)
    # robot_control.open_gripper("right")
    # time.sleep(2)
    # robot_control.capture_publisher("left")
    # time.sleep(5)
    # robot_control.capture_publisher("head")
    # robot_control.single_move("right", 420, -125, -100, "side", 30)
    # robot_control.single_move("right", 480, -125, -100, "side", 30)
    # robot_control.single_move("right", 580, -125, -50, "side", 30)
    # rospy.spin()
    # if len(received_base_positions) < 1:
    #     rospy.logwarn("未收到足夠的基座標，無法執行動作。")
        
    # else:
    #     bx, by, bz = received_base_positions[0]
    #     rospy.loginfo(f"使用基座標進行動作: ({bx:.1f}, {by:.1f}, {bz:.1f})")
    #     time.sleep(2)
    #     while True:
    #         user_input = input("輸入 1 繼續下一步動作，或按 q 退出: ")
    #         if user_input == "1":
    #             print("✓ 繼續執行...")
    #             break
    #         elif user_input.lower() == "q":
    #             print("✗ 取消動作")
    #             exit()
    #         else:
    #             print("⚠ 請輸入 1 或 q")
    #     robot_control.single_move("right", bx, by-80.0, bz, "side", 0)
    #     time.sleep(2)
    #     robot_control.single_move("right", bx, by, bz, "side", 0)
    #     time.sleep(2)
        
    # # robot_control.single_move("right", 245.0, -22.1-80.0, -224.1, "side", 0)
    # # robot_control.single_move("right", 245.0, -22.1, -224.1, "side", 0)
    # robot_control.close_gripper("right")
   
    # robot_control.open_gripper("both")
    # robot_control.single_move("right", 380.0, -110.717, -200, "side", 0)
   

    
    # time.sleep(2)
    # robot_control.single_move("right", 241.9, -91.5, -229.6, "side", 0)


