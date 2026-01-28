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
from pydantic import BaseModel, Field
from typing import List, Literal, Optional
from enum import Enum

load_dotenv()  # 讀取 .env 檔
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# === 定義枚舉類型（Enum）：限制可選值，避免輸入錯誤 ===
class ArmType(str, Enum):
    """
    定義手臂類型的枚舉
    繼承 str 和 Enum，使其既是字串又是枚舉類型
    這樣可以直接用於字串比較，也能獲得枚舉的類型安全
    """
    LEFT = "left"    # 左手臂
    RIGHT = "right"  # 右手臂

class ActionType(str, Enum):
    """
    定義動作類型的枚舉
    限制只能使用這四種預定義的動作類型
    防止 LLM 生成不存在的函式名稱
    """
    EYE_IN_HAND = "arm_eyeInHand_camera_catch"  # 使用手眼相機重新捕捉物品
    PICK = "pick"                                # 抓取動作
    SWEEP = "sweep_the_table"                    # 掃桌動作
    PLACE = "place"                              # 放置動作


class PickMode(str, Enum):
    """
    定義抓取/放置模式的枚舉
    對應不同的夾爪姿態
    """
    DOWN = "down"        # 夾爪向下（適合平躺物品）
    SIDE = "side"        # 夾爪從側面（適合豎立物品）
    FORWARD = "forward"  # 夾爪向前


# === 定義資料模型（BaseModel）：使用 Pydantic 進行結構化定義 ===

class ActionStep(BaseModel):
    """
    單一動作步驟的資料模型
    繼承 BaseModel 後，Pydantic 會自動：
    1. 驗證每個欄位的類型是否正確
    2. 將輸入的 JSON/dict 轉換為此物件
    3. 提供 .dict() 和 .json() 方法進行序列化
    """
    
    # Field() 用於定義欄位的額外資訊（描述、預設值、驗證規則等）
    # 使用 ge (greater than or equal，大於等於)
    # 強制此欄位必須是整數，且不可為負數
    step_id: int = Field(ge=0, description="步驟編號，必須大於等於 0")
   
    
    action_type: ActionType = Field(description="動作類型")
    # 限制只能是 ActionType 枚舉中的值，防止錯誤的函式名
    
    arm: ArmType = Field(description="使用的手臂")
    # 限制只能是 "left" 或 "right"
    
    # Optional[int] 表示此欄位可以是整數或 None
    # Field(None, ...) 中的 None 是預設值，表示此欄位可選
    object_index: Optional[int] = Field(None, description="物體索引")
    # 某些動作（如 sweep）不需要物體索引，所以設為可選
    
    mode: Optional[PickMode] = Field(None, description="放置模式")
    # 只有  place 需要 mode，其他動作可為 None
    
    angle: Optional[float] = Field(None, description="角度")
    # 角度參數，某些動作不需要所以設為可選
    
    # default_factory=list 表示如果沒提供此欄位，預設建立空列表
    # 避免可變預設值（mutable default）的問題
    prerequisites: List[int] = Field(default_factory=list, description="前置步驟ID列表")
    # 記錄此步驟依賴哪些前置步驟（用於依賴關係檢查）
    
    def to_function_call(self) -> str:
        """
        將此動作步驟轉換為可執行的函式呼叫字串
        
        Returns:
            str: 可執行的 Python 函式呼叫字串
            
        此方法根據 action_type 生成對應的函式呼叫格式
        使用 .value 取得枚舉的實際字串值
        """
        if self.action_type == ActionType.EYE_IN_HAND:
            # 手眼相機動作：需要物體索引和手臂
            return f"arm_eyeInHand_camera_catch({self.object_index}, '{self.arm.value}')"
        
        elif self.action_type == ActionType.PICK:
            # 抓取動作：需要手臂
            return f"pick('{self.arm.value}')"
        
        elif self.action_type == ActionType.SWEEP:
            # 掃地動作：不需要額外參數
            return "sweep_the_table()"
        
        elif self.action_type == ActionType.PLACE:
            # 放置動作：需要物體索引、模式、角度和手臂
            return f"place({self.object_index}, '{self.mode.value}', {self.angle}, '{self.arm.value}')"


class ArmPlan(BaseModel):
    """
    單臂動作計畫
    將單隻手臂的所有步驟組合在一起
    """
    arm: ArmType  # 指定是左手還是右手的計畫
    # 這個變數是一個「列表」，列表中每個元素的型別必須是 ActionStep 物件（可以確認型態）
    steps: List[ActionStep]  # 此手臂要執行的所有步驟列表


class RobotPlan(BaseModel):
    """
    完整的機器人動作計畫
    包含左右兩隻手臂的所有動作步驟
    
    這是最頂層的資料結構，當 LLM 輸出 JSON 時：
    1. 將 JSON 字串解析為 dict
    2. 使用 RobotPlan(**dict) 建立物件
    3. Pydantic 自動驗證所有欄位類型和值
    4. 如果有錯誤，會拋出 ValidationError 並說明哪個欄位有問題
    """
    
    # default_factory=list 確保即使 LLM 沒提供此欄位，也會建立空列表
    left_arm: List[ActionStep] = Field(default_factory=list)
    # 左手的所有動作步驟
    
    right_arm: List[ActionStep] = Field(default_factory=list)
    # 右手的所有動作步驟
    
    task_description: str = Field(description="任務描述")
    # 必填欄位，描述此計畫要完成什麼任務



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
            "object_position: px=310mm, py=0mm, pz=-300mm"
            "object_angle: 0 deg"
            "==============================="
        )
        
        self.environment_context_prompt = (
            "掃把與畚箕平躺在桌面上，pick_mode = down"
            "垃圾平躺在桌上，要清掃起來而非抓取"
            
            
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

            "pick( arm): 分配手臂執行抓取特定物品動作，arm為使用的手臂(left or right)。"
            
            "sweep_the_table(): 執行掃桌動作。"
            
            "place(object_index,string place_mode, angle, arm): 執行放置物品動作，object_index為物體索引，place_mode為放置模式，angle為手臂角度，arm為使用的手臂(left or right)。"
            "object_index: 物體的索引，為數值。"
            "place_mode: 放置模式，為字串(string)，包含 'down' 表示夾爪向下方放置; 'side' 表示夾爪從物品側面放置; 'forward' 表示從夾爪向前方放置。共以上三種模式"
            "angle: 物品角度，為數值。"
            "arm: 指定使用的手臂，選項為 'left' 或 'right'。"
        )
        self.safety_constraints_prompt = (
        
            "物品y座標大於20，使用左手手臂抓取，物品y座標小於-20，使用右手手臂抓取。"
            "物品y座標介於-20到20之間，根據物品角度，角度小於90度分配給左手，角度大於等於90度分配給右手。"
        )
        self.task_planning_profile_prompt = (
            "歷史任務規劃紀錄: "
            "==============================="
            "任務: 使用掃把與畚箕清理桌面"
            "動作規劃: "
            "left arm:"
            "Step 1. arm_eyeInHand_camera_catch(1, 'left')"
            "Step 2. pick('left')"
            "Step 3. sweep_the_table()"
            "Step 4. place(1, 'down', 0, 'left')"

            "right arm:"
            "Step 1. arm_eyeInHand_camera_catch(0, 'right')"
            "Step 2. pick('right')"
            "Step 3. sweep_the_table()"
            "Step 4. place(0, 'down', 10, 'right')"
            "==============================="
        )
        # self.output_format_prompt = (
        #     "輸出格式: "
        #     "1. 動作步驟需清楚標示每一步驟所使用的函式與參數"
        #     "2. 輸出需使用提供相關物品的資訊與動作函式"
        #     "3. 輸出需符合使用者的需求與期望"
        #     "4. 請勿說明與解釋，僅輸出動作規劃步驟"
        #     "輸出範例: "
        #     "==============================="
        #     "left arm:"
        #     "Step 1. arm_eyeInHand_camera_catch(1, 'left')"
        #     "Step 2. pick(1, 'down', 0, 'left')"
        #     "Step 3. sweep_the_table()"
        #     "Step 4. place(1, 'down', 0, 'left')"

        #     "right arm:"
        #     "Step 1. arm_eyeInHand_camera_catch(0, 'right')"
        #     "Step 2. pick(0, 'side', 10, 'right')"
        #     "Step 3. sweep_the_table()"
        #     "Step 4. place(0, 'down', 10, 'right')"
        #     "==============================="
        # )
           # 修改輸出格式提示
        
        self.output_format_prompt = (
            "## 輸出格式要求\n"
            "\n"
            "### 基本規則\n"
            "1. **僅輸出 JSON 格式**：不要包含任何說明、解釋或推理過程\n"
            "2. **嚴格遵循 JSON 語法**：使用雙引號，正確的逗號和括號\n"
            "3. **完整性**：每個步驟必須清楚標示函式名稱與所有必要參數\n"
            "4. **一致性**：所有資訊必須來自前面提供的感測器資訊和動作函式描述\n"
            "\n"
            "### JSON 結構說明\n"
            "```json\n"
            "{\n"
            "  \"task_description\": \"<任務描述字串>\",\n"
            "  \"left_arm\": [\n"
            "    {\n"
            "      \"step_id\": <正整數，從1開始>,\n"
            "      \"action_type\": \"<動作類型，必須是以下之一: arm_eyeInHand_camera_catch, pick, sweep_the_table, place>\",\n"
            "      \"arm\": \"<手臂選擇，必須是: left 或 right>\",\n"
            "      \"object_index\": <物體索引數值，可選，某些動作不需要>,\n"
            "      \"mode\": \"<模式，可選，必須是: down, side, forward 之一>\",\n"
            "      \"angle\": <角度數值，可選，浮點數或整數>,\n"
            "      \"prerequisites\": [<前置步驟ID列表，整數陣列>]\n"
            "    }\n"
            "  ],\n"
            "  \"right_arm\": [\n"
            "    {\n"
            "      \"step_id\": <正整數，從1開始>,\n"
            "      \"action_type\": \"<動作類型>\",\n"
            "      \"arm\": \"right\",\n"
            "      \"object_index\": <物體索引>,\n"
            "      \"mode\": \"<模式>\",\n"
            "      \"angle\": <角度>,\n"
            "      \"prerequisites\": [<前置步驟ID列表>]\n"
            "    }\n"
            "  ]\n"
            "}\n"
            "```\n"
            "\n"
            "### 欄位詳細說明\n"
            "\n"
            "**必填欄位（所有步驟都需要）：**\n"
            "- `step_id`: 步驟編號，正整數，建議從1開始依序編號\n"
            "- `action_type`: 動作類型，必須是以下四種之一：\n"
            "  - \"arm_eyeInHand_camera_catch\" - 手眼相機重新捕捉\n"
            "  - \"pick\" - 抓取動作\n"
            "  - \"sweep_the_table\" - 掃桌動作\n"
            "  - \"place\" - 放置動作\n"
            "- `arm`: 使用的手臂，\"left\" 或 \"right\"\n"
            "- `prerequisites`: 前置步驟ID陣列，表示此步驟依賴哪些步驟完成。若無依賴則使用空陣列 []\n"
            "\n"
            "**條件必填欄位（依據 action_type）：**\n"
            "- `object_index`: 物體索引（整數），以下動作需要：\n"
            "  - arm_eyeInHand_camera_catch\n"
            "  - place\n"
            "  - sweep_the_table 不需要此欄位\n"
            "\n"
            "- `mode`: 抓取/放置模式（字串），以下動作需要：\n"
            "  - place: 必須是 \"down\", \"side\", \"forward\" 之一\n"
            "  - 其他動作不需要此欄位\n"
            "\n"
            "- `angle`: 角度（數值），以下動作需要：\n"
            "  - place: 根據放置需求設定\n"
            "  - 其他動作不需要此欄位\n"
            "\n"
            "### 完整輸出範例\n"
            "\n"
            "**範例任務：使用掃把與畚箕清理桌面**\n"
            "\n"
            "```json\n"
            "{\n"
            "  \"task_description\": \"使用掃把與畚箕清理桌面\",\n"
            "  \"left_arm\": [\n"
            "    {\n"
            "      \"step_id\": 1,\n"
            "      \"action_type\": \"arm_eyeInHand_camera_catch\",\n"
            "      \"arm\": \"left\",\n"
            "      \"object_index\": 1,\n"
            "      \"prerequisites\": []\n"
            "    },\n"
            "    {\n"
            "      \"step_id\": 2,\n"
            "      \"action_type\": \"pick\",\n"
            "      \"arm\": \"left\",\n"
            "      \"prerequisites\":[1] \n"
            "    },\n"
            "    {\n"
            "      \"step_id\": 3,\n"
            "      \"action_type\": \"sweep_the_table\",\n"
            "      \"arm\": \"left\",\n"
            "      \"prerequisites\":[2] \n"
            "    },\n"
            "    {\n"
            "      \"step_id\": 4,\n"
            "      \"action_type\": \"place\",\n"
            "      \"arm\": \"left\",\n"
            "      \"object_index\": 1,\n"
            "      \"mode\": \"down\",\n"
            "      \"angle\": 20,\n"
            "      \"prerequisites\":[3] \n"
            "    }\n"
            "  ],\n"
            "  \"right_arm\": [\n"
            "    {\n"
            "      \"step_id\": 1,\n"
            "      \"action_type\": \"arm_eyeInHand_camera_catch\",\n"
            "      \"arm\": \"right\",\n"
            "      \"object_index\": 0,\n"
            "      \"prerequisites\": []\n"
            "    },\n"
            "    {\n"
            "      \"step_id\": 2,\n"
            "      \"action_type\": \"pick\",\n"
            "      \"arm\": \"right\",\n"
            "      \"prerequisites\":[1] \n"
            "    },\n"
            "    {\n"
            "      \"step_id\": 3,\n"
            "      \"action_type\": \"sweep_the_table\",\n"
            "      \"arm\": \"right\",\n"
            "      \"prerequisites\":[2] \n"
            "    },\n"
            "    {\n"
            "      \"step_id\": 4,\n"
            "      \"action_type\": \"place\",\n"
            "      \"arm\": \"right\",\n"
            "      \"object_index\": 0,\n"
            "      \"mode\": \"side\",\n"
            "      \"angle\": 120,\n"
            "      \"prerequisites\":[3] \n"
            "    }\n"
            "  ]\n"
            "}\n"
            "```\n"
            "\n"
            "### 重要提醒\n"
            "1. 請勿在 JSON 外包含任何文字說明\n"
            "2. 確保所有字串使用雙引號（不是單引號）\n"
            "3. 陣列和物件的最後一個元素後面不要加逗號\n"
            "4. 數值型態不要加引號（如 step_id, object_index, angle）\n"
            "5. prerequisites 必須是陣列，即使為空也要寫成 []\n"
            "6. 請根據實際的感測器資訊和任務需求調整參數，不要完全照抄範例\n"
            "7. 每個步驟的 arm 欄位必須與該計畫所屬的手臂一致（left_arm 中所有步驟的 arm 都是 \"left\"）\n"
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
            self.sections.append(f"## 任務規劃檔案\n{self.task_planning_profile_prompt}")
        if self.output_format_prompt:
            self.sections.append(f"## 輸出格式\n{self.output_format_prompt}")

        self.sections.append("## 執行要求\n請根據以上所有資訊，生成詳細的動作規劃。")
        self.user_content = "\n\n".join(self.sections)

        return [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": self.user_content}
        ]


    # def task_planning(self):
    #     # 4. 產生 messages
    #     messages = self.create_robot_planning_messages()

    #     # 5. 呼叫 GPT-4o API
    #     response = client.chat.completions.create(
    #         model="gpt-4o",
    #         messages=messages,
    #         temperature=0.0
    #     )

    #     # 6. 顯示模型回應
    #     msg = response.choices[0].message
    #     print(json.dumps({
    #         "role": msg.role,
    #         "content": msg.content
    #     }, ensure_ascii=False, indent=2))
    #     text = msg.content   # 假設模型回應已存於 msg.content
    #     pattern = r"={10,}[\s]*([\s\S]+?)[\s]*={10,}"
    #     match = re.search(pattern, text)
    #     extracted = match.group(1).strip() if match else None

    #     print(extracted)
    #     self.output_text = extracted
    #     return msg.content

    def task_planning(self) -> RobotPlan:
        """生成任務計畫並返回結構化物件"""
        messages = self.create_robot_planning_messages()

        response = client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=0.2,
            response_format={"type": "json_object"}  # 強制 JSON 輸出
        )

        content = response.choices[0].message.content
        
        # 解析並驗證輸出
        try:
            plan_dict = json.loads(content) # 將 JSON 字串轉換為 Python 字典
            robot_plan = RobotPlan(**plan_dict)#  將字典的鍵值對轉換為函式參數 使用 Pydantic 驗證並建立物件 Pydantic 會自動驗證
            return robot_plan # 返回結構化的 RobotPlan 物件
        except Exception as e:
            print(f"解析失敗: {e}")
            print(f"原始輸出: {content}")
            raise



# # 寫入檔案（例如 output.txt）
# with open("output.txt", "w", encoding="utf-8") as f:
#     f.write(extracted)
