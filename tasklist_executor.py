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
    
    # === 實際的機器人動作函式 ===
    
    def arm_eyeInHand_camera_catch(self, object_index: int, arm: str):
        """使用手眼相機重新捕捉物品"""
        print(f"[{arm}] 相機重新捕捉物品 {object_index}")
        # 實際執行 ROS 相機動作
        # self.camera_service.call(object_index, arm)
    
    def pick(self, object_index: int, mode: str, angle: float, arm: str):
        """執行抓取動作"""
        print(f"[{arm}] 抓取物品 {object_index}，模式: {mode}，角度: {angle}")
        # 實際執行 ROS 抓取動作
        # self.pick_service.call(object_index, mode, angle, arm)
    
    def sweep_the_table(self):
        """執行掃桌動作"""
        print("執行掃桌動作")
        # 實際執行 ROS 掃桌動作
        # self.sweep_service.call()
    
    def place(self, object_index: int, mode: str, angle: float, arm: str):
        """執行放置動作"""
        print(f"[{arm}] 放置物品 {object_index}，模式: {mode}，角度: {angle}")
        # 實際執行 ROS 放置動作
        # self.place_service.call(object_index, mode, angle, arm)
    
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
            func(step.object_index, step.mode.value, step.angle, step.arm.value)
        
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
        
        # 方案 A: 依序執行（先左手後右手）
        print("\n【左手動作】")
        for step in robot_plan.left_arm:
            print(f"\n步驟 {step.step_id}:")
            self.execute_step(step)
        
        print("\n【右手動作】")
        for step in robot_plan.right_arm:
            print(f"\n步驟 {step.step_id}:")
            self.execute_step(step)
        
        # 方案 B: 交錯執行（根據 step_id 排序）
        # all_steps = []
        # for step in robot_plan.left_arm:
        #     all_steps.append(step)
        # for step in robot_plan.right_arm:
        #     all_steps.append(step)
        # all_steps.sort(key=lambda s: s.step_id)
        # 
        # for step in all_steps:
        #     print(f"\n步驟 {step.step_id} [{step.arm.value}]:")
        #     self.execute_step(step)

# === 使用範例 ===
executor = RobotExecutor()

# 建立測試計畫
robot_plan = RobotPlan(
    task_description="清理桌面",
    left_arm=[
        ActionStep(
            step_id=1,
            action_type=ActionType.EYE_IN_HAND,
            arm=ArmType.LEFT,
            object_index=1,
            prerequisites=[]
        ),
        ActionStep(
            step_id=2,
            action_type=ActionType.PICK,
            arm=ArmType.LEFT,
            object_index=1,
            mode=PickMode.DOWN,
            angle=0.0,
            prerequisites=[1]
        )
    ],
    right_arm=[
        ActionStep(
            step_id=3,
            action_type=ActionType.SWEEP,
            arm=ArmType.RIGHT,
            prerequisites=[2]
        )
    ]
)

# 執行計畫
executor.execute_plan(robot_plan)