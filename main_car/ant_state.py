import math
import gc

object_to_line_dict = {
    'T': 'U',
    'S': 'L',
    'E': 'L',
    'W': 'R',
    'B': 'R'
}

# 状态机类
class TaskController:
    def __init__(self, beep, state,  car, path, plan, vision, moving, plan_data, order_manager, art_protocal, main_protocol, assist_protocol):
        # 注入对象
        self.my_beep = beep
        self.my_path = path
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car
        self.my_moving = moving
        self.data = plan_data
        self.my_order_manager = order_manager
        self.my_art_protocol = art_protocal
        self.my_main_protocol = main_protocol
        self.my_assist_protocol = assist_protocol

        # 状态映射表：将状态常量映射到对应的处理函数
        self.handlers = {
            state.READY_NAVIGATE: self.handle_ready_navigate,
            state.NAVIGATE: self.handle_navigate,
            state.SCAN:     self.handle_scan,
            state.SERVO:    self.handle_servo,
            state.MOVE:     self.handle_move,
            state.CALIBRATE: self.handle_calibrate,
            state.ADJUST:   self.handle_adjust,
            state.RETURN:    self.handle_return,
            state.STOP:      self.handle_stop,
            # ... 其他状态
        }

        self.navigate_message = []  # 导航信息：目标点坐标和朝向
        self.slave_navigate_message = []  # 从车导航信息：目标点坐标和朝向
        self.scan_message = []  # 扫描信息：目标物体位置
        self.current_object = ''  # 当前目标物体种类
        # 标志位
        self.if_transitioning = True  # 是否正在进行状态转换
        self.if_send_path = False  # 是否已经发送路径规划信息

        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作
        
    # 不同模式下的执行函数
    def run(self):
        if self.if_transitioning:
            self.enter()  # 进入新状态执行一次性的进入函数

        # 获取当前状态对应的函数并执行
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函数
    def enter(self):
        state = self.my_state.state
        self.if_transitioning = False  # 进入新状态，重置状态转换标志位

        if state == self.my_state.READY_NAVIGATE:
            # 进入准备导航状态，做好路径规划准备和导航信息准备
            self.my_plan.reset_navigate_angle()
        elif state == self.my_state.NAVIGATE:
            # 进入导航状态，开始执行路径跟随
            self.my_plan.reset_navigate_angle()
        elif state == self.my_state.SCAN:
            # 进入扫描状态，开始寻找目标物体
            self.my_order_manager.mode_target() # 打开目标识别模式
            self.my_art_protocol.send_object_kind(self.my_plan.current_object)  # 发送目标物体种类信息
        elif state == self.my_state.SERVO:
            # 进入伺服状态，开始精确对准目标物体
            pass
        elif state == self.my_state.MOVE:
            # 进入搬运状态，开始搬运物体
            self.my_plan.reset_navigate_angle()
            self.my_moving.ready_move()  # 准备搬运动作
        elif state == self.my_state.CALIBRATE:
            # 进入校准状态，进行位置或传感器校准
            # 记录小车在哪个边线
            self.my_vision.car_position = object_to_line_dict.get(self.current_object)
        elif state == self.my_state.ADJUST:
            # 进入调整状态，根据需要进行微调
            pass
        elif state == self.my_state.RETURN:
            # 进入返回状态，返回起始点或下一任务点
            self.my_plan.reset_navigate_angle()
        elif state == self.my_state.STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()

    def exit(self):
        state = self.my_state.state

        if state == self.my_state.READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            self.my_state.state = self.my_state.NAVIGATE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.NAVIGATE:
            if not self.if_send_path:
                self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0])  # 发送路径信息给从车

            # 向辅助车发送回线消息
            self.my_assist_protocol.send_back_message()
            # 退出导航状态，停止路径跟随
            self.if_send_path = False  # 重置路径发送标志位
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = self.my_state.SCAN  # 直接切换到扫描状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.SCAN:
            # 退出扫描状态，停止寻找目标物体
            if not self.my_plan.if_finish_navigate:
                self.my_plan.reset_navigate()
                self.my_state.state = self.my_state.SERVO
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                # 如果小车并没有找到物体，先跳过当前物体
                self.my_plan.reset_navigate()
                self.data.current_index += 1  # 跳过当前物体，进入下一个物体的准备导航状态
                if self.data.current_index >= self.data.total_objects_num:
                    self.my_state.state = self.my_state.RETURN  # 如果所有物体都处理完了，进入返回状态
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                else:
                    self.my_state.state = self.my_state.READY_NAVIGATE
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                # 发送主车信息给从车
                if self.if_send_path == False:
                    self.my_main_protocol.send_path(self.current_object, self.slave_navigate_message[1], self.slave_navigate_message[0])  
                    self.if_send_path = True  # 设置标志位，避免重复发送路径信息

                if self.my_main_protocol.get_slave_state() == "get":
                    # 向辅助车发送预先到达的边界
                    line = object_to_line_dict.get(self.current_object)
                    self.my_assist_protocol.send_advanced_line(line)

                    self.if_send_path = False  # 重置路径发送标志位
                    self.my_vision.if_finish_servo = False  # 重置伺服完成标志
                    self.my_state.state = self.my_state.MOVE  # 直接切换到搬运状态
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                # 将openart置为等待模式
                self.my_order_manager.finish()
                self.my_plan.reset_navigate()
                self.data.current_index += 1  # 跳过当前物体，进入下一个物体的准备导航状态
                if self.data.current_index >= self.data.total_objects_num:
                    self.my_state.state = self.my_state.RETURN  # 如果所有物体都处理完了，进入返回状态
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                else:
                    self.my_state.state = self.my_state.READY_NAVIGATE
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.MOVE:
            # 退出搬运状态，停止搬运动作
            self.my_moving.if_finish_move = False  # 重置搬运完成标志
            self.my_state.state = self.my_state.CALIBRATE  # 直接切换到校准状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更新
            self.my_vision.reset_apriltag_calibrate()  # 重置校准标志
            self.my_state.state = self.my_state.READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物体
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.ADJUST:
            # 退出调整状态，完成微调后进行必要的状态更新
            pass
        elif state == self.my_state.RETURN:
            # 退出返回状态，完成返回后进行必要的状态更新
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = self.my_state.STOP  # 直接切换到停止状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == self.my_state.STOP:
            # 退出停止状态，准备进入下一任务或待命状态
            self.my_beep.test()  # 任务完成，发出提示音
    
    def handle_ready_navigate(self):
        # 进入准备导航状态，做好路径规划准备和导航信息准备
        target_x = self.data.rogue_planning[self.data.current_index][0][0]
        target_y = self.data.rogue_planning[self.data.current_index][0][1]
        self.current_object = self.data.rogue_planning[self.data.current_index][1]  # 提取当前物体种类信息
        turn = self.data.rogue_planning[self.data.current_index][3]

        # 主车最终目标点
        main_final_pt = []
        # 小车导航到物体前的距离
        stop_threshold = 20.0
        # 根据小车进入的边界信息选择合适的角度和扫描点信息
        if turn == "L":
            target_angle = 90.0
            self.scan_message = [[target_x - stop_threshold, target_y]]
            self.slave_navigate_message = [[self.data.fixed_point[1][0] - stop_threshold, target_y], target_angle]
            main_final_pt = [self.data.fixed_point[1][0], target_y]
        elif turn == "R":
            target_angle = -90.0
            self.scan_message = [[target_x + stop_threshold, target_y]]
            self.slave_navigate_message = [[self.data.fixed_point[2][0] + stop_threshold, target_y], target_angle]
            main_final_pt = [self.data.fixed_point[2][0], target_y]
        elif turn == "U":
            target_angle = 180.0
            self.scan_message = [[self.my_car.x_current, target_y + stop_threshold]]
            self.slave_navigate_message = [[target_x, self.data.fixed_point[2][1] + stop_threshold], target_angle]
            main_final_pt = [target_x, self.data.fixed_point[2][1]]
        else:
            target_angle = 0.0
            self.scan_message = [[self.my_car.x_current, target_y - stop_threshold]]
            self.slave_navigate_message = [[target_x, self.data.fixed_point[1][1] - stop_threshold], target_angle]
            main_final_pt = [target_x, self.data.fixed_point[1][1]]

         # 进行路径规划
        self.my_path.plan_path(main_final_pt[0], main_final_pt[1])  

        self.navigate_message = [self.my_path.ready_path, target_angle]  # 准备导航信息
        self.exit()  # 退出当前状态，进入导航状态

    def handle_navigate(self):
        # if state == NAVIGATE
        self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])
        
        # 主车行驶多远后给从车发送路径信息
        dist_threshold = 30.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信息

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状态

    def handle_scan(self):
        # if state == SCAN
        self.my_plan.navigate(path = self.scan_message)

        target_point = self.my_art_protocol.coordinate_receive()
        if target_point and chr(target_point[2]) == self.current_object:  
            self.my_vision.ready_servo_and_orbit(target_point)

            self.exit()  # 退出当前状态，进入扫描状态

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            self.my_plan.navigate(path = [[self.my_car.x_current+10.0, self.my_car.y_current], [self.my_car.x_current-10.0, self.my_car.y_current], self.navigate_message[-1]])
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.my_vision.current_servo_object:
                self.my_vision.ready_servo_and_orbit(target_point)
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False

        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()

        if self.my_moving.if_finish_move:
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_calibrate(self):
        # if state == CALIBRATE
        self.my_vision.apriltag_calibrate_control()

        if self.my_vision.if_finish_calibrate:
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_adjust(self):
        # if state == ADJUST
        pass

    def handle_return(self):
        # if state == RETURN
        self.my_plan.navigate(path = [self.data.fixed_point[3]])  # 返回起始点

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()