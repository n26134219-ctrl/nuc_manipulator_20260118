from typing import Dict, List, Tuple
from dataclasses import dataclass
from task_planning import RobotPlan, ActionStep, ActionType

@dataclass
class ValidationError:
    """驗證錯誤"""
    error_type: str  # "狀態衝突", "缺少前提", "冗餘動作", "工具錯誤"
    severity: int    # 1-10 嚴重程度
    step_id: int
    arm: str
    description: str

class PlanValidator:
    """通用的任務計畫靜態驗證器 (支援依賴模擬與 Mermaid 可視化)"""
    
    def __init__(self):
        self.errors: List[ValidationError] = []
        self.total_penalty = 0
        
        # 【泛用化核心】定義各動作執行前，手中必須持有的物品關鍵字
        # 只要新增任務，直接在這裡擴充即可，不需要改寫底層邏輯
        self.tool_requirements = {
            ActionType.SWEEP: ["brush", "dustpan"],             # 清掃需要刷子或畚箕
            ActionType.SPRINKLE_PEPPER: ["pepper"],             # 撒胡椒需持有胡椒罐
            ActionType.CLOSE_SANDWICH: ["toast", "sandwich"], # 蓋三明治需持有吐司
        }


    
    def validate_plan(self, robot_plan: RobotPlan, 
                      current_left_held: str = None, 
                      current_right_held: str = None) -> Tuple[bool, List[ValidationError], int]:
        
        """
        驗證計畫並模擬雙手狀態
        """
        self.errors = []
        self.total_penalty = 0
        require_empty_at_end = False  # 任務結束時是否要求雙手都空著（視任務需求而定）
        # 建立時間軸
        all_steps = []
        if hasattr(robot_plan, 'left_arm'):
            all_steps.extend(robot_plan.left_arm)
        if hasattr(robot_plan, 'right_arm'):
            all_steps.extend(robot_plan.right_arm)
            
        all_steps.sort(key=lambda s: s.step_id)

        # ====== 關鍵修正區塊 ======
        # 1. 處理 "empty" 字串：把它們轉換成 Python 的 None，這樣後面的 is None 判斷才會正確
        if current_left_held == "empty": 
            current_left_held = None
        if current_right_held == "empty": 
            current_right_held = None

        # 2. 將傳入的真實狀態，寫入虛擬沙盒中 (注意這裡千萬不能寫死成 None)
        arm_states = {
            "left": current_left_held, 
            "right": current_right_held
        }

        # 💡 新增 1：空間追蹤 (紀錄手臂目前停留在哪個物品上，以及動作發生的 step_id)
        arm_locations = {
            "left": (None, -1),
            "right": (None, -1)
        }

        # 💡 新增 2：時序狀態追蹤 (用於三明治任務)
        timeline_flags = {
            "has_sprinkled": False,
            "has_closed": False
        }
        completed_steps = set()
        print(f"初始手臂狀態: Left arm holds '{arm_states['left']}', Right arm holds '{arm_states['right']}'")
        for step in all_steps:
            # 兼容 Enum 或 String 格式
            arm_val = step.arm.value if hasattr(step.arm, 'value') else str(step.arm)
            action = step.action_type
            
            # 獲取物品名稱 (兼容屬性差異)
            obj_name = getattr(step, 'object_name', '') or str(getattr(step, 'object_index', ''))
            obj_name_lower = str(obj_name).lower()

            # --- 1. 檢查前置依賴 (Prerequisites) ---
            for prereq_id in getattr(step, 'prerequisites', []):
                if prereq_id not in completed_steps and prereq_id < step.step_id:
                    self._add_error("缺少前提", 10, step.step_id, arm_val, f"明確的前置步驟 {prereq_id} 尚未完成")

            # --- 2. 狀態機與動作邏輯驗證 ---
            if action == ActionType.MOVE:
                arm_locations[arm_val] = (obj_name_lower, step.step_id)
                other_arm = "right" if arm_val == "left" else "left"
                
                # 取出另一隻手的狀態
                other_obj, other_step_id = arm_locations[other_arm]
                
                # 【關鍵修正】：只有「目標相同」且「發生的 step_id 也完全一樣」時，才算同時對撞
                if other_obj == obj_name_lower and obj_name_lower and other_step_id == step.step_id:
                    if obj_name_lower in ["rice food"]:
                        pass  # rice food 不算會對撞的危險物品
                    else:
                        self._add_error("碰撞風險", 8, step.step_id, arm_val, 
                                    f"左右手在同一個步驟 (Step {step.step_id}) 同時移動到同一個目標 ('{obj_name}')，將發生機械臂對撞！請使用 wait 動作")
            elif action == ActionType.EYE_IN_HAND:
                pass  # 相機捕捉不改變夾爪狀態

            elif action == ActionType.PICK:
                if arm_states[arm_val] is not None:
                    self._add_error("狀態衝突", 9, step.step_id, arm_val, 
                                    f"手臂已持有 {arm_states[arm_val]}，無法同時再抓取 {obj_name}")
                arm_states[arm_val] = obj_name

            elif action == ActionType.PLACE:
                if arm_states[arm_val] is None:
                    self._add_error("狀態衝突", 8, step.step_id, arm_val, "放置前手臂未抓取任何物品")
                    # require_empty_at_end = True
                else:
                    arm_states[arm_val] = None
                    # require_empty_at_end = True
            elif action == ActionType.SWEEP:
                require_empty_at_end = True
                # 將左右手狀態轉為小寫字串方便比對
                left_item = str(arm_states["left"]).lower() if arm_states["left"] else ""
                right_item = str(arm_states["right"]).lower() if arm_states["right"] else ""
                
                # 檢查整體系統(雙手)是否擁有這兩樣工具
                has_brush = "brush" in left_item or "brush" in right_item
                has_dustpan = "dustpan" in left_item or "dustpan" in right_item
                
                if not (has_brush and has_dustpan):
                    self._add_error("缺少工具", 10, step.step_id, arm_val, 
                                    f"執行 sweep_towards 失敗！機器人必須雙手並用，一手拿 brush，一手拿 dustpan (當前左手: {left_item or '空'}, 右手: {right_item or '空'})")
            # ==================================================
            # 🛑 檢查: 灑胡椒粉狀態
            elif action == ActionType.SPRINKLE_PEPPER:
                timeline_flags["has_sprinkled"] = True
                left_item = str(arm_states["left"]).lower() if arm_states["left"] else ""
                right_item = str(arm_states["right"]).lower() if arm_states["right"] else ""
                
                if  "pepper shaker" in left_item:
                    if arm_val != "left":
                        self._add_error("工具錯誤", 9, step.step_id, arm_val, 
                                    f"執行 sprinkle_pepper 動作時，左手持有 {left_item}，但必須使用左手來撒胡椒粉！")
                elif "pepper shaker" in right_item:
                    if arm_val != "right":
                        self._add_error("工具錯誤", 9, step.step_id, arm_val, 
                                        f"執行 sprinkle_pepper 動作時，右手持有 {right_item}，但必須使用右手來撒胡椒粉！")  
            
              
            # 🛑 檢查: 蓋三明治狀態
            elif action == ActionType.CLOSE_SANDWICH:
                timeline_flags["has_closed"] = True
                left_item = str(arm_states["left"]).lower() if arm_states["left"] else ""
                right_item = str(arm_states["right"]).lower() if arm_states["right"] else ""
                print(f"執行 close_sandwich 動作，檢查雙手持有物: 左手 '{left_item or '空'}', 右手 '{right_item or '空'}'")
                if "sliced toast" in left_item :
                    if arm_val != "left":
                        self._add_error("工具錯誤", 9, step.step_id, arm_val, 
                                        f"執行 close_sandwich 動作時，左手持有 {left_item}，但必須使用左手來蓋三明治！")
                elif "sliced toast" in right_item:
                    if arm_val != "right":
                        self._add_error("工具錯誤", 9, step.step_id, arm_val, 
                                        f"執行 close_sandwich 動作時，右手持有 {right_item}，但必須使用右手來蓋三明治！")
            # 🛑 檢查: 等待胡椒粉時機 (必須在 sprinkled 之後，closed 之前)
            # elif action == ActionType.WAIT:
            #     if not timeline_flags["has_sprinkled"]:
            #         self._add_error("順序錯誤", 9, step.step_id, arm_val, 
            #                         "wait 必須在 sprinkle_pepper 動作『之後』執行！")
            #     if timeline_flags["has_closed"]:
            #         self._add_error("順序錯誤", 9, step.step_id, arm_val, 
            #                         "wait 必須在 close_sandwich 動作『之前』執行！")
            # 泛用任務動作檢查 (掃地、撒胡椒、蓋吐司等)
            elif action in self.tool_requirements:
                held_item = arm_states[arm_val]
                req_keywords = self.tool_requirements[action]
                
                if held_item is None:
                    self._add_error("缺少工具", 10, step.step_id, arm_val, 
                                    f"執行 {action.name} 前未持有工具 (需持有包含 {req_keywords} 的物品)")
                else:
                    # 檢查持有的物品是否吻合任務所需關鍵字
                    if not any(kw in held_item.lower() for kw in req_keywords):
                        self._add_error("工具錯誤", 9, step.step_id, arm_val, 
                                        f"執行 {action.name} 需要 {req_keywords}，但目前持有的是 {held_item}")

            # 標記完成
            completed_steps.add(step.step_id)


        #  避免忘記放下物品的情況，要求任務結束時雙手都空著（視任務需求而定）
        if require_empty_at_end:
                    if arm_states["left"] is not None:
                        self._add_error("未歸位", 8, all_steps[-1].step_id if all_steps else 0, "left", 
                                        f"任務結束時，左手仍持有 {arm_states['left']} 未放下，必須規劃 place 動作")
                                        
                    if arm_states["right"] is not None:
                        self._add_error("未歸位", 8, all_steps[-1].step_id if all_steps else 0, "right", 
                                        f"任務結束時，右手仍持有 {arm_states['right']} 未放下，必須規劃 place 動作")

        
        # --- 3. 檢查不合理的重複動作 ---
        self._check_duplicates(all_steps)

        passed = self.total_penalty < 5
        return passed, self.errors, self.total_penalty

    def _check_duplicates(self, steps: List[ActionStep]):
        """檢查同一隻手連續出現完全一樣的無意義動作"""
        arm_steps = {"left": [], "right": []}
        for s in steps:
            arm_val = s.arm.value if hasattr(s.arm, 'value') else str(s.arm)
            arm_steps.setdefault(arm_val, []).append(s)
            
        for arm, s_list in arm_steps.items():
            for i in range(len(s_list) - 1):
                curr, nxt = s_list[i], s_list[i + 1]
                if curr.action_type == nxt.action_type :
                    if curr.action_type in [ActionType.WAIT]:
                        continue  # 等待動作可以連續出現多次，不算冗餘
                    obj1 = getattr(curr, 'object_name', '') or getattr(curr, 'object_index', '')
                    obj2 = getattr(nxt, 'object_name', '')  or getattr(nxt, 'object_index', '')
                    # 若對相同物品連續 EYE_IN_HAND 兩次，或是連續 SWEEP 兩次
                    if obj1 == obj2 and curr.action_type in [ActionType.EYE_IN_HAND]:
                        self._add_error("冗餘動作", 3, nxt.step_id, arm, f"不必要的連續重複動作: {curr.action_type.name}")

    def _add_error(self, err_type: str, severity: int, step_id: int, arm: str, desc: str):
        self.errors.append(ValidationError(err_type, severity, step_id, arm, desc))
        self.total_penalty += severity

    def print_report(self):
        """列印文字版驗證報告"""
        print(f"\n{'='*60}")
        print(f"驗證報告 - 總扣分: {self.total_penalty}")
        print(f"{'='*60}")
        if not self.errors:
            print("✅ 計畫通過所有邏輯檢查")
        else:
            for error in self.errors:
                print(f"❌ [{error.error_type}] 嚴重度: {error.severity}/10 | 步驟: {error.step_id} ({error.arm} arm)")
                print(f"   描述: {error.description}")
        print(f"{'='*60}\n")

    def visualize_plan_mermaid(self, robot_plan: RobotPlan) -> str:
        """
        生成 Mermaid 流程圖語法 (可視化任務序列與錯誤)
        能在 Markdown 預覽或 VS Code 中直接渲染成圖表
        """
        mermaid_lines = [
            "graph TD", 
            "classDef errorFill fill:#ffcccc,stroke:#ff0000,stroke-width:2px;" # 錯誤節點用紅色標示
        ]
        
        # 建立錯誤查找表，快速找出哪些 step_id 被扣分了
        error_map = {e.step_id: e for e in self.errors}
        
        left_steps = getattr(robot_plan, 'left_arm', [])
        right_steps = getattr(robot_plan, 'right_arm', [])
        
        def build_subgraph(arm_name, steps):
            lines = [f"  subgraph {arm_name.capitalize()} Arm"]
            prev_id = None
            for s in steps:
                action_name = s.action_type.name if hasattr(s.action_type, 'name') else str(s.action_type)
                obj_name = getattr(s, 'object_name', '') or str(getattr(s, 'object_index', ''))
                
                # 節點文字內容
                label = f"[{s.step_id}] {action_name}"
                if obj_name:
                    label += f"<br>({obj_name})"
                
                node_id = f"S{s.step_id}"
                lines.append(f"    {node_id}[\"{label}\"]")
                
                # 若該步驟有錯，標記紅色
                if s.step_id in error_map:
                    lines.append(f"    class {node_id} errorFill")
                    
                # 手臂內動作畫連續箭頭
                if prev_id:
                    lines.append(f"    {prev_id} --> {node_id}")
                prev_id = node_id
            lines.append("  end")
            return lines

        mermaid_lines.extend(build_subgraph("Left", left_steps))
        mermaid_lines.extend(build_subgraph("Right", right_steps))
        
        # 繪製跨手臂 / 動作間的先決條件 (Prerequisites) 虛線依賴
        all_steps = left_steps + right_steps
        for s in all_steps:
            for prereq in getattr(s, 'prerequisites', []):
                mermaid_lines.append(f"  S{prereq} -. 依賴 .-> S{s.step_id}")
                
        return "\n".join(mermaid_lines)