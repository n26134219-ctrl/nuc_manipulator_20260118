
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

robot_control = CommandPublisher()
robot_state = False
load_dotenv()  # 讀取 .env 檔
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
task_description_prompt = "請利用桌面上的掃把與畚箕，將桌上掃乾淨。"

total_objects_phase_pub = rospy.Publisher('assign_object_phase', String, queue_size=10)

# received_base_positions = []  # 按順序儲存收到的基座標
class SharedObject:
    def __init__(self):
        self.total = []  # 總物體列表
        self.left = []   # 左側物體列表
        self.right = []  # 右側物體列表
        self.other = []  # 其他物體列表

shared_object = SharedObject()


# def action_state_callback(msg):
#     global robot_state
#     """接收並處理動作狀態"""
#     state = msg.data
#     rospy.loginfo(f"收到動作狀態: {state}")
#     if state == "Done":
#         rospy.loginfo("機器人動作完成，可以進行下一步操作。")
#         robot_state = True
        # 在這裡可以加入動作完成後的處理邏輯
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
            # rospy.loginfo(f"更新總物體列表: {shared_object.total}")    
        # else:    
        #     shared_object.total = data
        # rospy.loginfo(f"更新總物體列表: {shared_object.total}")
        show_info(shared_object.total)
    except json.JSONDecodeError as e:
            rospy.logerr(f"JSON 解析失败: {e}")
    except Exception as e:
        rospy.logerr(f"處理失敗: {e}")

def left_objects_callback(msg):
    """接收並更新左側物體列表"""
    global shared_object
    try:
        data = json.loads(msg.data)
        shared_object.left = data
        # rospy.loginfo(f"更新左側物體列表: {shared_object.left}")
        show_info(shared_object.left)
                
        angle_fine_tune()
    except json.JSONDecodeError as e:
            rospy.logerr(f"JSON 解析失败: {e}")
    except Exception as e:
        rospy.logerr(f"處理失敗: {e}")

def right_objects_callback(msg):
    """接收並更新右側物體列表"""
    global shared_object
    try:
        data = json.loads(msg.data)
        shared_object.right = data
        show_info(shared_object.right)
        
        angle_fine_tune()
        # rospy.loginfo(f"更新右側物體列表: {shared_object.right}")
    except json.JSONDecodeError as e:
            rospy.logerr(f"JSON 解析失败: {e}")
    except Exception as e:
        rospy.logerr(f"處理失敗: {e}")
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

        
def  draw_back_hands():
    time.sleep(3)
    robot_control.dual_move(266.3, 80, -220, "side", 0, 266.3, -80, -220, "side", 0)
    

def angle_fine_tune():
    global shared_object
    left = shared_object.left
    right = shared_object.right
    for obj in shared_object.left:
        angle = obj['angle']
        vector = obj['center_vector']
        if vector != None and vector[0]>0:
            obj['angle'] = -(180 - angle)
            rospy.loginfo(f"左:微調角度: {obj['angle']}")

    for obj in shared_object.right:
        angle = obj['angle']
        vector = obj['center_vector']
        if vector != None and vector[0]<0:
            obj['angle'] = -(180 - angle)
            rospy.loginfo(f"右:微調角度: {obj['angle']}")



def task_explanation_callback(msg):
    global task_description
    """接收並處理任務說明"""
    task_description = msg.data
    rospy.loginfo(f"收到任務說明: {task_description}")
   

def ros_sub_init():
    rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    rospy.Subscriber('task_explanation', String, task_explanation_callback)
    rospy.spin()

def search_trush():
    total_objects_phase_pub.publish("rice food")
    time.sleep(12)
    # robot_control.capture_publisher("head")
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
# def wait_for_robot_action_completion():
#     global robot_state
#     robot_state = False
#     while not robot_state:
#         time.sleep(0.5)
#     robot_state = False
def get_env_info():

    robot_control.initial_position()
    # wait_for_robot_action_completion()
    robot_control.capture_publisher("head")
    time.sleep(10)
    draw_back_hands()
    # wait_for_robot_action_completion()
    robot_control.neck_control(0, 42)
    # wait_for_robot_action_completion()
    while search_trush() == False:
        robot_control.neck_control(0, 42)
        # wait_for_robot_action_completion()
    time.sleep(2)
    robot_control.neck_control(0, 76)
    # wait_for_robot_action_completion()
    show_info(shared_object.total)
    robot_control.initial_position()
    # wait_for_robot_action_completion()

if __name__ == '__main__':
    
    
    # 發送不同的命令
    # rospy.Subscriber('/base/object_point', Point, base_callback)
    rospy.Subscriber('/camera/total_objects', String, total_objects_callback)
    rospy.Subscriber('/camera/left_objects', String, left_objects_callback)
    rospy.Subscriber('/camera/right_objects', String, right_objects_callback)
    # rospy.Subscriber('action_state', String, action_state_callback)
    # rospy.Subscriber('camera_search_control', String, camera_search_callback)
    # draw_back_hands()
    # robot_control.neck_control(0, 42)
    time.sleep(3)
    # get_env_info()
    # robot_control.capture_publisher("left")
    # robot_control.capture_publisher("head")
    # robot_control.capture_publisher("right")
    

    # robot_control.initial_position()
    # wait_for_robot_action_completion()

    robot_control.single_move("left", 320, 120, -130-40, "down", 90)
    wait_for_robot_action_completion()
    robot_control.single_move("right", 320, -120, -130-40, "side", 10)
    wait_for_robot_action_completion()
    robot_control.close_gripper("both")
    # robot_control.single_move("right", 266.3544006347656-20, -157.84457397460938, -304.7738037109375+170, "down", 90)
    # wait_for_robot_action_completion()
    # robot_control.capture_publisher("right")
    # robot_control.single_move("left", 284.922607421875, 58.331573486328125+30, -304.58917236328125+100, "down", 90)
    time.sleep(2)
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
    time.sleep(2)
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
    rospy.spin()
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


def arm_eyeInHand_camera_catch(object_index, arm):
    global shared_object
    object_info = shared_object.total[int(object_index)]
    object_pos = object_info.get('base_center_pos', None)
    pick_mode = object_info.get('pick_mode', "down")
    dis =  160.0 # 安全距離 mm
    if object_pos:
        rospy.loginfo(f"物體 {object_index} 的基座標位置: {object_pos}")
        if pick_mode == "down":
            robot_control.single_move(arm, object_pos[0], object_pos[1], object_pos[2]+dis, pick_mode, 90)
        elif pick_mode == "side":
            if arm == "left":
                robot_control.single_move(arm, object_pos[0], object_pos[1]+dis, object_pos[2], pick_mode, 0)
            elif arm == "right":
                robot_control.single_move(arm, object_pos[0], object_pos[1]-dis, object_pos[2], pick_mode, 0)
        time.sleep(20)
        robot_control.capture_publisher(arm)
def pick(object_index, pick_mode, angle, arm):
    if arm == "left":
        object = shared_object.left[int(object_index)]
    elif arm == "right":
        object = shared_object.right[int(object_index)]
    object_pos = object.get('base_center_pos', None)
    pick_mode = object.get('pick_mode', None)
    size = object.get('3d_size', None)
    angle = object.get('angle', 0)
    if object_pos and pick_mode and size:
        robot_control.single_arm_pick(arm, object_pos[0], object_pos[1], object_pos[2], pick_mode, size, angle, arm)


def place(arm):
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
 

# def sweep_the_table():
#     while search_trush() == true:
#         for index, obj in enumerate(shared_object.total):
#             if obj['name'] == "trush":
    # 掃把要改成side,但菶機down            
                

   
#     robot_control.single_move(arm, object_pos[0], object_pos[1]+ sign * size[0]/2, object_pos[2], pick_mode, angle)


# 1. 定義 System Prompt
system_prompt = """
你是一個雙手機器人動作規劃器，需要根據已知的資訊去做任務規劃的排程。
你的任務是:
1. 分析相機感測器的環境資訊與任務需求
2. 根據給予的動作函式描述，使用這些函式，完成安全且高效滿足任務需求
3. 產生的動作規劃需滿足環境資訊、雙手手臂狀態約束條件
4. 生成的動作規劃都需要根據給予的資訊去規劃，不可自行生成未給予的資訊
5. 根據任務規劃紀錄，如果有對應相同任務，可以參考內容去做調整，請勿完全抄襲，需根據實際任務需求與環境資訊去調整
6. 參考輸出格式說明，產生對應格式輸出，請勿抄襲範例格式，需根據實際任務需求與環境資訊去調整
請遵循給予的資訊去規劃，請勿自行生成未給予的資訊，並提供清晰的推理過程後，再根據輸出規格格式給予適當輸出。


"""


# 2. 定義多段 User Prompt 組合函式
def create_robot_planning_messages(
        
    system_prompt: str,
    action_info_prompt:str,
    camera_info: str,
    task_desc: str,
    environment_info: str ,
    safety_constraints: str ,
    task_planning_profile:str,
    output_format: str 
):
    sections = []

    if camera_info:
        sections.append(f"## 感測器資訊\n{camera_info}")
    if environment_info:
        sections.append(f"## 環境限制資訊\n{environment_info}")
    if action_info_prompt:
        sections.append(f"Action function description:\n{action_info_prompt}")
    
    if task_desc:
        sections.append(f"## 任務描述\n{task_desc}")
    if safety_constraints:
        sections.append(f"## 安全限制\n{safety_constraints}")
    if task_planning_profile:
        sections.append(f"## 任務規劃檔案\n{task_planning_profile}")
    if output_format:
        sections.append(f"## 輸出格式\n{output_format}")

    sections.append("## 執行要求\n請根據以上所有資訊，生成詳細的動作規劃。")
    user_content = "\n\n".join(sections)

    return [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_content}
    ]

# 3. 範例 Prompt 組件
camera_information_prompt = (
    "[object] info: "
    "object_name: dustpan"
    "object_index: 0"
    "object_position: px=300mm, py=-22mm, pz=-300mm"
    "object_angle: 120 deg"
    "pick_mode: side"
    
    "object_name: broom"
    "object_index: 1"
    "object_position: px=300mm, py=80mm, pz=-300mm"
    "object_angle: 20 deg"
    "pick_mode: down"

    "object_name: trash"
    "object_index: 2"
    "object_min: px=290mm, py=15mm, pz=-300mm"
    "object_max: px=320mm, py=-15mm, pz=-300mm"
    "object_angle: 0 deg"
    "==============================="

)


environment_context_prompt = (
    "掃把與畚箕平躺在桌面上，pick_mode = down"
    "垃圾平躺在桌上"
    "桌面位置 -330mm"
    # "垃圾範圍: x: 290 ~ 320mm, py= 15 ~ -15mm"

)
action_description_prompt = (
    "以下為可以使用的動作函式:"

    "arm_eyeInHand_camera_catch(object_index, arm) "
    "pick(object_index,string pick_mode, angle, arm)"
    "sweep_the_table()"
    "place(object_index,string place_mode, angle, arm)"

    "======================================"
    "以下為動作函式說明:"
    "arm_eyeInHand_camera_catch(object_index, arm): 對應物品，分配對應手臂，讓手臂上相機再照一次，獲得準確物品資訊，更新物品資訊。"
    "object_index: 物體的索引，為數值。"
    "arm: 指定使用的手臂，選項為 'left' 或 'right'。"

    "pick(object_index,string pick_mode, angle, arm): 分配手臂執行抓取特定物品動作，object_index為物體索引，pick_mode為抓取模式，angle為手臂角度，arm為使用的手臂(left or right)。"
    "object_index: 物體的索引，為數值。"
    "pick_mode: 抓取模式，為字串(string)，包含 'down' 表示夾爪向下方抓取; 'side' 表示夾爪從物品側面抓取; 'forward' 表示從夾爪向前方抓取。共以上三種模式"
    "angle: 物品角度，為數值。"
    "sweep_the_table(): 執行掃桌動作。"
    
    "place(object_index,string place_mode, angle, arm): 執行放置物品動作，object_index為物體索引，place_mode為放置模式，angle為手臂角度，arm為使用的手臂(left or right)。"
    "object_index: 物體的索引，為數值。"
    "place_mode: 放置模式，為字串(string)，包含 'down' 表示夾爪向下方放置; 'side' 表示夾爪從物品側面放置; 'forward' 表示從夾爪向前方放置。共以上三種模式"
    "angle: 物品角度，為數值。"
    "arm: 指定使用的手臂，選項為 'left' 或 'right'。"
)
safety_constraints_prompt = (
    
    "物品y座標大於20，使用左手手臂抓取，物品y座標小於-20，使用右手手臂抓取。"
    "物品y座標介於-20到20之間，根據物品角度，角度小於90度分配給左手，角度大於等於90度分配給右手。"
)
task_planning_profile_prompt = (
    "歷史任務規劃紀錄: "
    "==============================="
    "任務: 使用掃把與畚箕清理桌面"
    "動作規劃: "
    "left arm:"
    "Step 1. arm_eyeInHand_camera_catch(1, 'left')"
    "Step 2. pick(1, 'down', 0, 'left')"
    "Step 3. sweep_the_table()"
    "Step 4. place(1, 'down', 0, 'left')"

    "right arm:"
    "Step 1. arm_eyeInHand_camera_catch(0, 'right')"
    "Step 2. pick(0, 'side', 10, 'right')"
    "Step 3. sweep_the_table()"
    "Step 4. place(0, 'down', 10, 'right')"
    "==============================="
)
output_format_prompt = (
    "輸出格式: "
    "1. 動作步驟需清楚標示每一步驟所使用的函式與參數"
    "2. 輸出需使用提供相關物品的資訊與動作函式"
    "3. 輸出需符合使用者的需求與期望"
    "4. 請勿說明與解釋，僅輸出動作規劃步驟"
    "輸出範例: "
    "==============================="
    "left arm:"
    "Step 1. arm_eyeInHand_camera_catch(1, 'left')"
    "Step 2. pick(1, 'down', 0, 'left')"
    "Step 3. sweep_the_table()"
    "Step 4. place(1, 'down', 0, 'left')"

    "right arm:"
    "Step 1. arm_eyeInHand_camera_catch(0, 'right')"
    "Step 2. pick(0, 'side', 10, 'right')"
    "Step 3. sweep_the_table()"
    "Step 4. place(0, 'down', 10, 'right')"
    "==============================="
)

# 4. 產生 messages
messages = create_robot_planning_messages(
    system_prompt=system_prompt,
    action_info_prompt=action_description_prompt,
    camera_info=camera_information_prompt,
    task_desc=task_description_prompt,
    environment_info=environment_context_prompt,
    safety_constraints=safety_constraints_prompt,
    task_planning_profile=task_planning_profile_prompt,
    output_format=output_format_prompt
)



# # 5. 呼叫 GPT-4o API
# client = OpenAI()
# response = client.chat.completions.create(
#     model="gpt-4o",
#     messages=messages,
#     temperature=0.0
# )

# # 6. 顯示模型回應
# msg = response.choices[0].message
# print(json.dumps({
#     "role": msg.role,
#     "content": msg.content
# }, ensure_ascii=False, indent=2))
# import re

# output_text = msg.content

# # 用正則找出 `````` 或 =============================== 內容
# import re

# text = msg.content   # 假設模型回應已存於 msg.content
# pattern = r"={10,}[\s]*([\s\S]+?)[\s]*={10,}"
# match = re.search(pattern, text)
# extracted = match.group(1).strip() if match else None

# print(extracted)



# # 寫入檔案（例如 output.txt）
# with open("output.txt", "w", encoding="utf-8") as f:
#     f.write(extracted)


