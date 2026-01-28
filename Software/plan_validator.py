from typing import Dict, List, Tuple
from dataclasses import dataclass
from task_planning import RobotPlan, ActionStep, ActionType
@dataclass
class ValidationError:
    """驗證錯誤"""
    error_type: str  # "重複步驟", "缺少前提", "順序衝突", "不必要動作"
    severity: int  # 1-10 嚴重程度
    step_id: int
    arm: str
    description: str

class PlanValidator:
    """任務計畫靜態驗證器"""
    
    def __init__(self):
        self.errors: List[ValidationError] = []
        self.total_penalty = 0
        
    def validate_plan(self, robot_plan: RobotPlan) -> Tuple[bool, List[ValidationError], int]:
        """
        驗證計畫
        返回: (是否通過, 錯誤列表, 總扣分)
        """
        self.errors = []
        self.total_penalty = 0
        
        # 驗證左右手計畫
        self._validate_arm_plan(robot_plan.left_arm, "left")
        self._validate_arm_plan(robot_plan.right_arm, "right")
        
        # 驗證雙手協調
        self._validate_coordination(robot_plan.left_arm, robot_plan.right_arm)
        
        passed = self.total_penalty < 15  # 設定門檻值
        return passed, self.errors, self.total_penalty
    
    def _validate_arm_plan(self, steps: List[ActionStep], arm: str):
        """驗證單臂計畫"""
        # 1. 檢查重複步驟
        self._check_duplicates(steps, arm)
        
        # 2. 檢查前提條件
        self._check_prerequisites(steps, arm)
        
        # 3. 檢查順序邏輯
        self._check_sequence_logic(steps, arm)
        
        # 4. 檢查不必要動作
        self._check_unnecessary_actions(steps, arm)

        # 5. 檢查同一物品只能分配一隻手
        self._check_single_hand_per_object(steps, arm)

      
    
    def _check_duplicates(self, steps: List[ActionStep], arm: str):
        """檢查重複步驟"""
        for i in range(len(steps) - 1):
            current = steps[i]
            next_step = steps[i + 1]
            
            # 檢查連續兩步是否完全相同
            if (current.action_type == next_step.action_type and
                current.object_index == next_step.object_index and
                current.mode == next_step.mode):
                
                error = ValidationError(
                    error_type="重複步驟",
                    severity=8,
                    step_id=next_step.step_id,
                    arm=arm,
                    description=f"步驟 {next_step.step_id} 與步驟 {current.step_id} 重複"
                )
                self.errors.append(error)
                self.total_penalty += error.severity
    
    # def _check_prerequisites(self, steps: List[ActionStep], arm: str):
    #     """檢查前提條件"""
    #     held_objects = set()  # 記錄手上持有的物品
        
    #     for step in steps:
    #         # 檢查 sweep_the_table 前是否有 pick 掃把
    #         if step.action_type == ActionType.SWEEP:
    #             # 檢查之前是否有抓取掃把或畚箕
    #             has_broom_or_dustpan = any(
    #                 s.action_type == ActionType.PICK and 
    #                 s.step_id < step.step_id 
    #                 for s in steps
    #             )
    #             if not has_broom_or_dustpan:
    #                 error = ValidationError(
    #                     error_type="缺少前提",
    #                     severity=10,
    #                     step_id=step.step_id,
    #                     arm=arm,
    #                     description=f"步驟 {step.step_id} 執行掃地前未抓取掃把或畚箕"
    #                 )
    #                 self.errors.append(error)
    #                 self.total_penalty += error.severity
            
    #         # 檢查 place 前是否有對應的 pick
    #         if step.action_type == ActionType.PLACE:
    #             if step.object_index not in held_objects:
    #                 error = ValidationError(
    #                     error_type="缺少前提",
    #                     severity=9,
    #                     step_id=step.step_id,
    #                     arm=arm,
    #                     description=f"步驟 {step.step_id} 放置物品 {step.object_index} 前未抓取"
    #                 )
    #                 self.errors.append(error)
    #                 self.total_penalty += error.severity
            
    #         # 更新持有狀態
    #         if step.action_type == ActionType.PICK:
    #             held_objects.add(step.object_index)
    #         elif step.action_type == ActionType.PLACE:
    #             held_objects.discard(step.object_index)
    def _check_prerequisites(self, steps: List[ActionStep], arm: str):
        """檢查前提條件和依賴關係"""
        completed_steps = set()  # 已完成的步驟 ID
        held_objects = {}  # {object_index: step_id} 記錄物品被哪個步驟抓取
        last_eye_in_hand = {}  # {step_id: object_index} 記錄 eye_in_hand 偵測的物品
        
        for step in steps:
            # 1. 檢查 prerequisites 中的步驟是否都已完成
            for prereq_id in step.prerequisites:
                if prereq_id not in completed_steps:
                    error = ValidationError(
                        error_type="缺少前提",
                        severity=10,
                        step_id=step.step_id,
                        arm=arm,
                        description=f"步驟 {step.step_id} 的前置步驟 {prereq_id} 尚未完成"
                    )
                    self.errors.append(error)
                    self.total_penalty += error.severity
            
            # 2. 記錄 eye_in_hand 偵測的物品
            if step.action_type == ActionType.EYE_IN_HAND:
                if step.object_index is not None:
                    last_eye_in_hand[step.step_id] = step.object_index
            
            # 3. 檢查 pick 前是否有 eye_in_hand 前置步驟
            elif step.action_type == ActionType.PICK:
                # 檢查 prerequisites 中是否有 eye_in_hand
                eye_in_hand_prereqs = [
                    prereq_id for prereq_id in step.prerequisites
                    if any(s.step_id == prereq_id and s.action_type == ActionType.EYE_IN_HAND 
                        for s in steps)
                ]
                
                if not eye_in_hand_prereqs:
                    error = ValidationError(
                        error_type="缺少前提",
                        severity=8,
                        step_id=step.step_id,
                        arm=arm,
                        description=f"步驟 {step.step_id} pick 前未偵測物品 (缺少 eye_in_hand 前置步驟)"
                    )
                    self.errors.append(error)
                    self.total_penalty += error.severity
                else:
                    # 從最近的 eye_in_hand 前置步驟推斷抓取的物品
                    last_eye_prereq = max(eye_in_hand_prereqs)
                    if last_eye_prereq in last_eye_in_hand:
                        obj_idx = last_eye_in_hand[last_eye_prereq]
                        held_objects[obj_idx] = step.step_id
            
            # 4. 檢查 sweep 前是否有 pick 前置步驟
            elif step.action_type == ActionType.SWEEP:
                # 檢查 prerequisites 中是否有 pick
                pick_prereqs = [
                    prereq_id for prereq_id in step.prerequisites
                    if any(s.step_id == prereq_id and s.action_type == ActionType.PICK 
                        for s in steps)
                ]
                
                if not pick_prereqs:
                    error = ValidationError(
                        error_type="缺少前提",
                        severity=10,
                        step_id=step.step_id,
                        arm=arm,
                        description=f"步驟 {step.step_id} sweep 前未抓取工具 (缺少 pick 前置步驟)"
                    )
                    self.errors.append(error)
                    self.total_penalty += error.severity
            
            # 5. 檢查 place 前是否有對應物品的 pick
            elif step.action_type == ActionType.PLACE:
                if step.object_index is not None:
                    # 檢查該物品是否已被抓取
                    if step.object_index not in held_objects:
                        error = ValidationError(
                            error_type="缺少前提",
                            severity=9,
                            step_id=step.step_id,
                            arm=arm,
                            description=f"步驟 {step.step_id} 放置物品 {step.object_index} 前未抓取"
                        )
                        self.errors.append(error)
                        self.total_penalty += error.severity
                    else:
                        # 放置後移除持有狀態
                        del held_objects[step.object_index]
            
            # 標記此步驟已完成
            completed_steps.add(step.step_id)



    def _check_sequence_logic(self, steps: List[ActionStep], arm: str):
        """檢查順序衝突"""
        for i in range(len(steps) - 1):
            current = steps[i]
            next_step = steps[i + 1]
            
            # 檢查「放下掃把 → 掃地」這類衝突
            if (current.action_type == ActionType.PLACE and 
                current.object_index == 1 and  # 掃把
                next_step.action_type == ActionType.SWEEP):
                
                error = ValidationError(
                    error_type="順序衝突",
                    severity=7,
                    step_id=next_step.step_id,
                    arm=arm,
                    description=f"步驟 {next_step.step_id} 在放下掃把後無法執行掃地"
                )
                self.errors.append(error)
                self.total_penalty += error.severity
            # 檢查「掃地 → 抓取掃把」這類衝突
            if (current.action_type == ActionType.SWEEP and 
                next_step.action_type == ActionType.PICK and 
                next_step.object_index == 1):
                
                error = ValidationError(
                    error_type="順序衝突",
                    severity=10,
                    step_id=next_step.step_id,
                    arm=arm,
                    description=f"步驟 {next_step.step_id} 在掃地後無法立即抓取掃把"
                )
                self.errors.append(error)
                self.total_penalty += error.severity
    
    def _check_unnecessary_actions(self, steps: List[ActionStep], arm: str):
        """檢查不必要動作（可選）"""
        # 檢查是否有多次 eye_in_hand 同一物品
        eye_in_hand_counts = {}
        for step in steps:
            if step.action_type == ActionType.EYE_IN_HAND:
                obj_idx = step.object_index
                eye_in_hand_counts[obj_idx] = eye_in_hand_counts.get(obj_idx, 0) + 1
                
                if eye_in_hand_counts[obj_idx] > 1:
                    error = ValidationError(
                        error_type="不必要動作",
                        severity=3,
                        step_id=step.step_id,
                        arm=arm,
                        description=f"步驟 {step.step_id} 重複檢視物品 {obj_idx}"
                    )
                    self.errors.append(error)
                    self.total_penalty += error.severity
    
    def _check_single_hand_per_object(self, steps: List[ActionStep], arm: str):
        """檢查同一物品只能分配一隻手"""
        object_hand_map = {}
        
        for step in steps:
            if step.action_type in {ActionType.PICK, ActionType.PLACE, ActionType.EYE_IN_HAND}:
                obj_idx = step.object_index
                if obj_idx in object_hand_map:
                    if object_hand_map[obj_idx] != arm:
                        error = ValidationError(
                            error_type="不必要動作",
                            severity=9,
                            step_id=step.step_id,
                            arm=arm,
                            description=f"物品 {obj_idx} 已被另一隻手分配"
                        )
                        self.errors.append(error)
                        self.total_penalty += error.severity
                else:
                    object_hand_map[obj_idx] = arm





    def _validate_coordination(self, left_steps: List[ActionStep], right_steps: List[ActionStep]):
        """驗證雙手協調（可選）"""
        # 檢查兩隻手是否同時掃地
        left_sweep_steps = []
        for s in left_steps:
            if s.action_type == ActionType.SWEEP:
                left_sweep_steps.append(s.step_id)
        right_sweep_steps = []
        for s in right_steps:
            if s.action_type == ActionType.SWEEP:
                right_sweep_steps.append(s.step_id)
        # left_sweep_steps = [s.step_id for s in left_steps if s.action_type == ActionType.SWEEP]
        # right_sweep_steps = [s.step_id for s in right_steps if s.action_type == ActionType.SWEEP]
        
        if len(left_sweep_steps) + len(right_sweep_steps) == 0:
            error = ValidationError(
                error_type="缺少前提",
                severity=5,
                step_id=0,
                arm="both",
                description="計畫中沒有掃地動作"
            )
            self.errors.append(error)
            self.total_penalty += error.severity
    
    def print_report(self):
        """列印驗證報告"""
        print(f"\n{'='*60}")
        print(f"驗證報告 - 總扣分: {self.total_penalty}")
        print(f"{'='*60}")
        
        if not self.errors:
            print("✓ 計畫通過所有檢查")
        else:

            for error in self.errors:
                print(f"\n[{error.error_type}] 嚴重度: {error.severity}/10")
                print(f"  手臂: {error.arm}")
                print(f"  步驟: {error.step_id}")
                print(f"  描述: {error.description}")
        
        print(f"{'='*60}\n")
