# 状态机lei
class TaskController:
    def __init__(self, state, plan, vision, car):
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car

        # 状态映射表：将状态常量映射到对应的处理函数
        self.handlers = {
            state.NAVIGATE: self.handle_navigate,
            state.SCAN:     self.handle_scan,
            state.SERVO:    self.handle_servo,
            state.MOVE:     self.handle_move,
            # ... 其他状态
        }

    def run(self):
        # 获取当前状态对应的函数并执行
        handler = self.handlers.get(self.state_machine.state)
        if handler:
            handler()

    def handle_navigate(self):
        # 原本 if state == NAVIGATE 下的代码放在这里
        pass

    def handle_scan(self):
        # 原本 if state == SCAN 下的代码放在这里
        pass