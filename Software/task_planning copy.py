#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rospy
import json
import time
from geometry_msgs.msg import Point
import os
from dotenv import load_dotenv
from openai import OpenAI
import re
load_dotenv()  # 讀取 .env 檔
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))



class GPTPlanner:
    def __init__(self):
        self.system_prompt = """
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
        self.task_description_prompt = "請利用桌面上的掃把與畚箕，將桌上掃乾淨。"

        self.camera_information_prompt = (
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
        
        self.environment_context_prompt = (
            "掃把與畚箕平躺在桌面上，pick_mode = down"
            "垃圾平躺在桌上"
            "桌面位置 -340mm"
            
            # "垃圾範圍: x: 290 ~ 320mm, py= 15 ~ -15mm"
        )
        self.action_description_prompt = (
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
        self.safety_constraints_prompt = (
        
            "物品y座標 大於 10，使用左手手臂抓取，物品y座標 小於 -10，使用右手手臂抓取。"
            "物品y座標介於-10到10之間，根據物品角度，角度小於90度分配給左手，角度大於等於90度分配給右手。"
        )
        self.task_planning_profile_prompt = (
            "任務規劃範例: "
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
        self.output_format_prompt = (
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
        self.sections = []

    
# 2. 定義多段 User Prompt 組合函式
    def create_robot_planning_messages( self):
    

        if self.camera_information_prompt:
            self.sections.append(f"## 感測器資訊\n{self.camera_information_prompt}")
        if self.environment_context_prompt:
            self.sections.append(f"## 環境限制資訊\n{self.environment_context_prompt}")
        if self.action_description_prompt:
            self.sections.append(f"Action function description:\n{self.action_description_prompt}")
        
        if self.task_description_prompt:
            self.sections.append(f"## 任務解釋\n{self.task_description_prompt}")
        if self.safety_constraints_prompt:
            self.sections.append(f"## 安全限制\n{self.safety_constraints_prompt}")
        if self.task_planning_profile_prompt:
            self.sections.append(f"## 任務規劃範例\n{self.task_planning_profile_prompt}")
        if self.output_format_prompt:
            self.sections.append(f"## 輸出格式\n{self.output_format_prompt}")

        self.sections.append("## 執行要求\n請根據以上所有資訊，生成詳細的動作規劃。")
        self.user_content = "\n\n".join(self.sections)

        return [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": self.user_content}
        ]


    def task_planning(self):
        # 4. 產生 messages
        messages = self.create_robot_planning_messages()

        # 5. 呼叫 GPT-4o API
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=0.0
        )

        # 6. 顯示模型回應
        msg = response.choices[0].message
        print(json.dumps({
            "role": msg.role,
            "content": msg.content
        }, ensure_ascii=False, indent=2))
        text = msg.content   # 假設模型回應已存於 msg.content
        pattern = r"={10,}[\s]*([\s\S]+?)[\s]*={10,}"
        match = re.search(pattern, text)
        extracted = match.group(1).strip() if match else None

        print(extracted)
        self.output_text = extracted
        return msg.content





# # 寫入檔案（例如 output.txt）
# with open("output.txt", "w", encoding="utf-8") as f:
#     f.write(extracted)
