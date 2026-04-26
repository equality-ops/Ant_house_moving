# 状态机lei
class TaskController:
    def __init__(self, state, plan, vision, car, plan_data):
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car
        self.my_plan_data = plan_data

        # 状态映射表：将状态常量映射到对应的处理函数
        self.handlers = {
            state.NAVIGATE: self.handle_navigate,
            state.SCAN:     self.handle_scan,
            state.SERVO:    self.handle_servo,
            state.MOVE:     self.handle_move,
            state.CALIBRATE: self.handle_calibrate,
            state.RETURN:    self.handle_return,
            state.STOP:      self.handle_stop,
            # ... 其他状态
        }

        # 标志位
        self.if_transitioning = False  # 是否正在进行状态转换

    # 不同模式下的执行函数
    def run(self):
        # 获取当前状态对应的函数并执行
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函数
    def enter(self):
        pass

    def exit(self):
        pass

    # 每个状态对应的处理函数
    def handle_navigate(self):
        # if state == NAVIGATE
        pass

    def handle_scan(self):
        # if state == SCAN
        pass

    def handle_servo(self):
        # if state == SERVO
        pass    

    def handle_move(self):
        # if state == MOVE
        pass

    def handle_calibrate(self):
        # if state == CALIBRATE
        pass

    def handle_return(self):
        # if state == RETURN
        pass

    def handle_stop(self):
        # if state == STOP
        pass