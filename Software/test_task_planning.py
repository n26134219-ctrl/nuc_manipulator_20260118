
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import json
import time
from geometry_msgs.msg import Point

# from dotenv import load_dotenv
import os
# from openai import OpenAI
from actionCommand import *
from task_planning import *
from plan_validator import *

import requests  # 新增：用來呼叫 API
import base64    # 新增：用來編碼圖片
import zlib      # 新增：用來壓縮資料


class Robotstatus:
    def __init__(self):
        self.left_arm_status = "empty"
        self.right_arm_status = "empty"

def main():
    planner = GPTPlanner()
    robot_state = Robotstatus()
    # 1. 生成計畫
    planner.task_description_prompt = "請使用桌上工具，清掃桌面"
    planner.camera_information_prompt = """object_name: dustpan tool 
    object_index: 0 
    object_position: px=261.5mm, py=-145.6mm, pz=-302.3mm 
    object_angle: 122.0 deg 
    pick_mode: down 

    object_name: brush tool 
    object_index: 1 
    object_position: px=280.6mm, py=70.7mm, pz=-319.4mm 
    object_angle: 20.3 deg 
    pick_mode: down 

    object_name: rice food 
    object_index: 2 
    object_position: px=471.6mm, py=68.1mm, pz=-284.5mm 
    object_angle: 175.4 deg 
    pick_mode: down """

    # planner.task_description_prompt = "請規劃一個機器人抓取胡椒粉罐動作"
    # planner.camera_information_prompt = """object_name: pepper shaker
    # object_index: 0 
    # object_position: px=561.5mm, py=-105.6mm, pz=-100.5mm 
    # object_angle: 150 deg 
    # pick_mode: side """

    # planner.task_description_prompt = "請規劃一個機器人完成抓取吐司動作"
    # planner.camera_information_prompt = """object_name: pepper shaker
    # object_index: 0 
    # object_position: px=561.5mm, py=-105.6mm, pz=-100.5mm 
    # object_angle: 150 deg 
    # pick_mode: side 
    # status: right arm is holding pepper shaker

    # object_name: sliced toast
    # object_index: 1
    # object_position: px=580.0mm, py= 135.0mm, pz=-250.0mm
    # object_angle: 48 deg
    # pick_mode: side
    
    # """
    # planner.robot_status_prompt = (
    #         "機器人狀態: \n"
    #         "left arm: empty\n"
    #         "right arm: holding pepper shaker\n"
    # )


    # planner.task_description_prompt = "請規劃一個機器人完成抓取胡椒粉與吐司動作"
    # planner.camera_information_prompt = """object_name: pepper shaker
    # object_index: 0 
    # object_position: px=561.5mm, py=105.6mm, pz=-100.5mm 
    # object_angle: 150 deg 
    # pick_mode: side 
    # status: left arm is holding pepper shaker

    # object_name: sliced toast
    # object_index: 1
    # object_position: px=580.0mm, py= -135.0mm, pz=-250.0mm
    # object_angle: 48 deg
    # pick_mode: side
    # """
    # planner.robot_status_prompt = (
    #         "機器人狀態: \n"
    #         "left arm: empty\n"
    #         "right arm: empty\n"
    # )
    

   


    planner.task_description_prompt = (
        "請規劃一個機器人完成放胡椒粉到原位動作。"
    )
    planner.environment_context_prompt = (
            ""
    )
    print(planner.task_description_prompt)
    planner.camera_information_prompt = """object_name: pepper shaker
    object_index: 0 
    object_position: px=561.5mm, py=105.6mm, pz=-100.5mm 
    object_angle: 150 deg 
    pick_mode: side 
    status: left arm is holding pepper shaker

    object_name: sliced toast
    object_index: 1
    object_position: px=580.0mm, py= -135.0mm, pz=-250.0mm
    object_angle: 48 deg
    pick_mode: side
    status: right arm is empty

    object_name: sandwich
    object_index: 2
    object_position: px=400.0mm, py=0.0mm, pz=-300.0mm
    object_angle: 30 deg
    pick_mode: down
    """
    planner.robot_status_prompt = (
            "機器人狀態: \n"
            # "left arm: empty\n"
            "left arm: holding pepper shaker\n"
            # "right arm: holding pepper shaker\n"
            "right arm: empty\n"
    )
    robot_state.left_arm_status = "sliced toast"
    # robot_state.right_arm_status = "pepper shaker"

    # planner.task_description_prompt = "請規劃一個機器人完成準備三明治動作"
    # planner.camera_information_prompt = """object_name: pepper shaker
    # object_index: 0 
    # object_position: px=561.5mm, py=105.6mm, pz=-100.5mm 
    # object_angle: 150 deg 
    # pick_mode: side 
    # status: left arm is holding pepper shaker

    # object_name: sliced toast
    # object_index: 1
    # object_position: px=580.0mm, py= -135.0mm, pz=-250.0mm
    # object_angle: 48 deg
    # pick_mode: side
    # status: right arm is holding sliced toast

    # object_name: sandwich
    # object_index: 2
    # object_position: px=400.0mm, py=0.0mm, pz=-300.0mm
    # object_angle: 30 deg
    # pick_mode: down
    # """
    # planner.robot_status_prompt = (
    #         "機器人狀態: \n"
    #         "left arm: holding pepper shaker\n"
    #         "right arm: holding sliced toast\n"
    # )
    # robot_state.left_arm_status = "pepper shaker"
    # robot_state.right_arm_status = "sliced toast"
    
 
    # planner.task_description_prompt = "請規劃一個機器人完成準備三明治動作"
    # planner.camera_information_prompt = """object_name: pepper shaker
    # object_index: 0 
    # object_position: px=561.5mm, py=-105.6mm, pz=-100.5mm 
    # object_angle: 150 deg 
    # pick_mode: side 
    # status: right arm is holding pepper shaker

    # object_name: sliced toast
    # object_index: 1
    # object_position: px=580.0mm, py= 135.0mm, pz=-250.0mm
    # object_angle: 48 deg
    # pick_mode: side
    # status: left arm is holding sliced toast

    # object_name: sandwich
    # object_index: 2
    # object_position: px=400.0mm, py=0.0mm, pz=-300.0mm
    # object_angle: 30 deg
    # pick_mode: down
    # """
    # planner.robot_status_prompt = (
    #         "機器人狀態: \n"
    #         "left arm: holding sliced toast\n"
    #         "right arm: holding pepper shaker\n"
    # )
    # robot_state.left_arm_status = "sliced toast"
    # robot_state.right_arm_status = "pepper shaker"
    
    max_retries = 5
    robot_plan = None
    passed = False

    robot_plan = planner.task_planning()
    for attempt in range(max_retries):
        print(f"\n[{'='*20} 第 {attempt + 1} 次嘗試生成計畫 {'='*20}]")
        
        # 1. 呼叫 LLM 產生計畫
        robot_plan = planner.task_planning()
        print("生成的計畫:")
        print(json.dumps(robot_plan.model_dump(), indent=2, ensure_ascii=False))  # 注意：Pydantic V2 建議用 model_dump() 取代 dict()
        
        # 2. 靜態驗證
        validator = PlanValidator()
        passed, errors, penalty = validator.validate_plan(
            robot_plan, 
            robot_state.left_arm_status, 
            robot_state.right_arm_status
        )
        validator.print_report()
        # mermaid_code = validator.visualize_plan_mermaid(robot_plan)
        # print("\n=== 可視化流程圖 (Mermaid) ===")
        # print(mermaid_code)
        
        # # (可選) 保留 Markdown 檔案供文字檢視
        # with open("plan_visual.md", "w", encoding="utf-8") as f:
        #     f.write("```mermaid\n")
        #     f.write(mermaid_code)
        #     f.write("\n```\n")
            
        # # ⭐ 新增這行：直接呼叫 API 將流程圖存成 PNG 圖片！
        # save_mermaid_as_png(mermaid_code, f"plan_visual_attempt_{attempt + 1}.png")
        # ==========================================
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
                planner.error_feedback_prompt = (
                    "你上次生成的計畫有以下致命錯誤，請重新思考並修正：\n" + error_msgs
                )
            else:
                print("❌ 已達最大重試次數，規劃失敗，請手動介入。")
                return False

    # 4. 執行計畫 (只有在順利 passed 才會走到這裡)
    if passed and robot_plan:
        print("\n開始執行計畫...")
        execute_plan(robot_plan)
        
        return True

def execute_plan(robot_plan: RobotPlan):
    """執行計畫"""
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
        # 實際呼叫對應函式
        # eval(func_call)  # 不建議直接 eval，應該用 match/case 或字典映射
def save_mermaid_as_png(mermaid_str: str, filename="plan_visual.png"):
    """使用免費的 Kroki API 將 Mermaid 語法直接轉換為 PNG 圖片檔並儲存"""
    try:
        print(f"正在生成流程圖圖片 {filename} ...")
        # 1. 依照 Kroki 格式進行壓縮與編碼
        compressed = zlib.compress(mermaid_str.encode('utf-8'), 9)
        encoded = base64.urlsafe_b64encode(compressed).decode('ascii')
        url = f"https://kroki.io/mermaid/png/{encoded}"
        
        # 2. 請求圖片
        response = requests.get(url)
        
        # 3. 存成圖檔
        if response.status_code == 200:
            with open(filename, 'wb') as f:
                f.write(response.content)
            print(f"✅ 可視化圖片已成功儲存！請查看資料夾中的: {filename}")
        else:
            print(f"❌ 圖片產生失敗 (狀態碼: {response.status_code})")
    except Exception as e:
        print(f"❌ 圖片產生發生錯誤: {e}")
if __name__ == "__main__":
    main()

