# 状态机类
class TaskController:
    def __init__(self, beep, math, state, plan, vision, car, plan_data, order_manager, art_protocal, main_protocol):
        # 注入对象
        self.my_beep = beep
        self.my_math = math
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car
        self.my_plan_data = plan_data
        self.my_order_manager = order_manager
        self.my_art_protocol = art_protocal
        self.my_main_protocol = main_protocol

        # 状态映射表：将状态常量映射到对应的处理函数
        self.handlers = {
            state.READT_NAVIGATE: self.handle_ready_navigate,
            state.NAVIGATE: self.handle_navigate,
            state.SCAN:     self.handle_scan,
            state.SERVO:    self.handle_servo,
            state.MOVE:     self.handle_move,
            state.CALIBRATE: self.handle_calibrate,
            state.RETURN:    self.handle_return,
            state.STOP:      self.handle_stop,
            # ... 其他状态
        }

        self.navigate_message = [[0.0, 0.0], 0.0]  # 导航信息：目标点坐标和朝向
        self.car_turn = 'U'  # 小车当前朝向，初始值为 'U'（假设初始朝向向上）
        # 标志位
        self.if_transitioning = True  # 是否正在进行状态转换

    # 不同模式下的执行函数
    def run(self):
        # 获取当前状态对应的函数并执行
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函数
    def enter(self):
        state = self.my_state.state
        if state == self.my_state.READT_NAVIGATE:
            self.my_plan.v_target, self.my_plan.turn_angle_target = 0, self.my_car.now_yaw * 180 / self.my_math.PI
        elif state == self.my_state.NAVIGATE:
            pass
        elif state == self.my_state.SCAN:
            if self.car_turn == 'U':
                self.navigate_message[0][0] = self.my_car.x_current
                self.navigate_message[0][1] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][1] + 5.0
            elif self.car_turn == 'R':
                self.navigate_message[0][0] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][0] + 5.0
                self.navigate_message[0][1] = self.my_car.y_current
            elif self.car_turn == 'D':
                self.navigate_message[0][0] = self.my_car.x_current
                self.navigate_message[0][1] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][1] - 5.0
            elif self.car_turn == 'L':
                self.navigate_message[0][0] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][1] - 5.0
                self.navigate_message[0][1] = self.my_car.y_current

    def exit(self):
        state = self.my_state.state
        if state == self.my_state.READT_NAVIGATE:
            self.my_state.state = self.my_state.NAVIGATE
            self.if_transitioning = True
        elif state == self.my_state.NAVIGATE:
            if self.my_plan.finish_navigate == True:
                self.my_state.state = self.my_state.SCAN
                self.my_plan.reset_navigate()  # 重置导航相关变量
                self.if_transitioning = True
        elif state == self.my_state.SCAN:
            if self.my_plan.finish_navigate == False:
                target_point = self.my_art_protocol.coordinate_receive()
                if target_point and target_point[1] > self.my_vision.dist_threshold and (target_point[2] in [ord('T'), ord('S'), ord('E'), ord('W'), ord('B')]):  
                    self.my_vision.ready_servo_and_orbit(target_point)
                    self.my_plan.reset_navigate_flags()
                    self.my_state.state = self.my_state.SERVO
                    self.if_transitioning = True
                    # 测试是否看到物体
                    self.my_beep.test()
            else:
                self.my_plan.reset_navigate_flags()
                self.my_state.state = self.my_state.READY_NAVIGATE
                self.if_transitioning = True
                self.my_plan.current_index += 1  # 更新当前搬运物体索引
                # 将openart置为等待模式
                self.my_order_manager.finish()

    # 每个状态对应的处理函数
    def handle_ready_navigate(self):
        # if state == READY_NAVIGATE
        # 进入准备导航状态，进行相关初始化
        # 先对下一导航阶段的小车姿态角进行优化
        now_yaw = self.my_car.now_yaw * 180 / self.my_math.PI
        dirs = ['U', 'R', 'D', 'L']
        # 将 -180~180 映射到 0~3 索引
        idx = int((now_yaw + 45) // 90) % 4
        car_turn = dirs[idx] if now_yaw >= -135 else 'D' # 边界微调

        # 获取目标方位字符串
        target_dir = self.my_plan_data.rogue_planning[self.my_plan.current_index][2]
        STRATEGY_MAP = {
            'U': {
                'match': 'U',                # 保持原样的情况
                'turn_d': {'U'},             # 需要转180度的情况
                'turn_r': {'LU', 'L'},       # 结果为 'R' 的集合
                'turn_l': {'RU', 'R'}        # 结果为 'L' 的集合
            },
            'R': {
                'match': 'R',
                'turn_l': {'R'},
                'turn_d': {'RU', 'U'},
                'turn_u': {'RD', 'D'}
            },
            'D': { 
                'match': 'D',
                'turn_u': {'D'},
                'turn_r': {'LD', 'L'}, 
                'turn_l': {'RD', 'R'}, 
            },
            'L': { 
                'match': 'L',
                'turn_r': {'L'},
                'turn_d': {'LU', 'U'}, 
                'turn_u': {'LD', 'D'}, 
            }
        }
        ELUDE_MAP = {
            'L':{
                'U':'RU',
                'C':'R',
                'D':'RD',
                'LU':{'U','RU'},
                'L':{'C','R'},
                'LD':{'D','RD'},
            },
            'R':{
                'U':'LU',
                'C':'L',
                'D':'LD',
                'RU':{'U','LU'},
                'R':{'C','L'},
                'RD':{'D','LD'},
            },
            'D':{
                'L':'LU',
                'C':'U',
                'R':'RU',
                'LD':{'LU','L'},
                'L':{'C','R'},
                'LD':{'D','RD'},
            }
        }
        # 执行查表逻辑
        if car_turn in STRATEGY_MAP:
            strategy = STRATEGY_MAP[car_turn]
            
            # 根据 target_dir 查找结果
            if target_dir in strategy.get('turn_r', {}):
                self.car_turn = 'R'
            elif target_dir in strategy.get('turn_l', {}):
                self.car_turn = 'L'
            elif target_dir in strategy.get('turn_u', {}):
                self.car_turn = 'U'
            elif target_dir in strategy.get('turn_d', {}):
                self.car_turn = 'D'
            else:
                # 保持原样
                self.car_turn = strategy['match']
        
        # 将下一目标点输入导航信息表中
        if self.car_turn == 'U':
            self.navigate_message[0][0] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][0]  # 目标点坐标
            self.navigate_message[0][1] = self.my_plan_data.rectangle_corners[2][1] + 5.0
            self.navigate_message[1] = 0.0
        elif self.car_turn == 'R':
            self.navigate_message[0][0] = self.my_plan_data.rectangle_corners[2][0] + 5.0
            self.navigate_message[0][1] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][1]  # 目标点坐标
            self.navigate_message[1] = 90.0
        elif self.car_turn == 'D':
            self.navigate_message[0][0] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][0]  # 目标点坐标
            self.navigate_message[0][1] = self.my_plan_data.rectangle_corners[0][1] - 5.0
            self.navigate_message[1] = 180.0
        elif self.car_turn == 'L':
            self.navigate_message[0][0] = self.my_plan_data.rectangle_corners[0][0] - 5.0
            self.navigate_message[0][1] = self.my_plan_data.navigate_points[self.my_plan_data.navigate_index][0][1]  # 目标点坐标
            self.navigate_message[1] = -90.0
        # 重置导航相关变量
        self.my_plan.reset_navigate()

    def handle_navigate(self):
        # if state == NAVIGATE
        self.my_plan.navigate([self.navigate_message[0]], self.navigate_message[1], "Y")

    def handle_scan(self):
        # if state == SCAN
        self.my_plan.navigate([self.navigate_message[0], [self.my_car.x_current, self.my_car.y_current]], if_elude = "N")

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