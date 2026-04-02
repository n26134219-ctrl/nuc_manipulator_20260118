
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rospy
import json
import time
from geometry_msgs.msg import Point

# from dotenv import load_dotenv
import os
# from openai import OpenAI
from actionCommand import *
from task_planning import *
from plan_validator import *


def main():
    planner = GPTPlanner()
    # 1. 生成計畫
    # planner.task_description_prompt = "請使用桌上工具，清掃桌面"
    # planner.camera_information_prompt = """object_name: dustpan tool 
    # object_index: 0 
    # object_position: px=261.5mm, py=145.6mm, pz=-302.3mm 
    # object_angle: 122.0 deg 
    # pick_mode: down 

    # object_name: brush tool 
    # object_index: 1 
    # object_position: px=280.6mm, py=-70.7mm, pz=-319.4mm 
    # object_angle: 20.3 deg 
    # pick_mode: down 

    # object_name: rice food 
    # object_index: 2 
    # object_position: px=471.6mm, py=68.1mm, pz=-284.5mm 
    # object_angle: 175.4 deg 
    # pick_mode: down """

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


    # planner.task_description_prompt = "請規劃一個機器人完成抓取吐司動作"
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
    #         "left arm: holding pepper shaker\n"
    #         "right arm: empty\n"
    # )

    # planner.task_description_prompt = "請規劃一個機器人完成準備三明治動作"
    # planner.task_description_prompt = "請規劃一個機器人完成放置胡椒粉動作"
    planner.task_description_prompt = (
        "請規劃一個機器人完成放胡椒粉到原位動作。"
    )
    planner.environment_context_prompt = (
            ""
    )
    print(planner.task_description_prompt)
    planner.camera_information_prompt = """object_name: pepper shaker
    object_index: 0 
    object_position: px=561.5mm, py=-105.6mm, pz=-100.5mm 
    object_angle: 150 deg 
    pick_mode: side 
    status: right arm is holding pepper shaker

    object_name: sliced toast
    object_index: 1
    object_position: px=580.0mm, py= 135.0mm, pz=-250.0mm
    object_angle: 48 deg
    pick_mode: side
    status: left arm is holding sliced toast

    object_name: sandwich
    object_index: 2
    object_position: px=400.0mm, py=0.0mm, pz=-300.0mm
    object_angle: 30 deg
    pick_mode: down
    """
    planner.robot_status_prompt = (
            "機器人狀態: \n"
            "left arm: empty\n"
            # "left arm: holding sliced toast\n"
            "right arm: holding pepper shaker\n"
    )

    # planner.task_description_prompt = "請規劃一個機器人完成三明治動作"
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
    
    
 
    robot_plan = planner.task_planning()
   
    print("生成的計畫:")


    print(json.dumps(robot_plan.dict(), indent=2, ensure_ascii=False))
    
    # 2. 靜態驗證
    validator = PlanValidator()
    passed, errors, penalty = validator.validate_plan(robot_plan)
    validator.print_report()
    
    # 3. 決定是否執行或重新規劃
    if not passed:
        print(f"⚠️  計畫未通過驗證（扣分: {penalty}），需要重新規劃")
        # 可以選擇：
        # - 重新呼叫 GPT 並附上錯誤訊息
        # - 使用規則自動修正
        return False
    
    # 4. 執行計畫
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

if __name__ == "__main__":
    main()

