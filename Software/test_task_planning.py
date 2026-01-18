
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
    # all_steps.sort(key=lambda x: x[0].step_id)
    
    for step, arm in all_steps:
        func_call = step.to_function_call()
        print(f"[{arm}] 執行: {func_call}")
        # 實際呼叫對應函式
        # eval(func_call)  # 不建議直接 eval，應該用 match/case 或字典映射

if __name__ == "__main__":
    main()

