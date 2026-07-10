from micropython import const
import time
import gc

PI = const(3.1415926)
READY_NAVIGATE = const(0)   # 准备导航状�?
NAVIGATE = const(1)       # 导航状�?
SCAN = const(2)           # 扫描状�?
SERVO = const(3)          # 视觉伺服状�?
ORBIT = const(4)          # 环绕状�?
MOVE = const(5)           # 搬运状�?
CALIBRATE = const(6)      # 校准状�?
ADJUST = const(7)           # 微调状�?
RETURN = const(8)		    # 返回状�?
STOP = const(9)           # 停止状�?
RETREAT = const(10)
object_to_line_dict = {
    'T': 'U',
    'S': 'L',
    'E': 'L',
    'W': 'R',
    'B': 'R'
}
counter = 0 
class TaskController:
    def __init__(self,object_plan, beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager, art_protocal, main_protocol, assist_protocol):
        # 注入对象
        self.my_beep = beep
        self.my_path = path
        self.my_uart = uart
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
        self.object_plan = object_plan
        # 状态映射表：将状态常量映射到对应的处理函�?
        self.handlers = {
            READY_NAVIGATE: self.handle_ready_navigate,
            NAVIGATE: self.handle_navigate,
            SCAN:     self.handle_scan,
            SERVO:    self.handle_servo,
            MOVE:     self.handle_move,
            CALIBRATE: self.handle_calibrate,
            ADJUST:   self.handle_adjust,
            RETURN:    self.handle_return,
            STOP:      self.handle_stop,
            RETREAT: self.handle_retreat,
            # ... 其他状�?
        }
        self.if_rogue_plan=self.data.if_rogue_plan
        self.navigate_message = []  # 导航信息：目标点坐标和朝�?
        self.slave_navigate_message = []  # 从车导航信息：目标点坐标和朝�?
        self.scan_message = []  # 扫描信息：目标物体位�?
        self.current_object = ''  # 当前目标物体种类
        # 标志�?
        self.if_transitioning = True  # 是否正在进行状态转�?
        self.if_send_path = False  # 是否已经发送路径规划信�?
        self.detected_num = 0
        self.if_send_detect_message = False
        self.last_side = 'D'
        self.retreat_message= (0,0)
        
        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作
        
    # 不同模式下的执行函数
    def run(self):
        if self.if_transitioning:
            self.enter()  # 进入新状态执行一次性的进入函数

        # 获取当前状态对应的函数并执�?
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函�?
    def enter(self):
        state = self.my_state.state
        self.if_transitioning = False  # 进入新状态，重置状态转换标志位

        if state == READY_NAVIGATE:
            # 进入准备导航状态，做好路径规划准备和导航信息准�?
            pass
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟�?
            pass
        elif state == SCAN:
            # 进入扫描状态，开始寻找目标物�?
            '''
            if self.my_vision.if_send_order == False:
                # 打开摄像�?
                self.my_order_manager.mode_target()
                self.my_vision.if_send_order = True
            '''
            self.my_vision.reset_analysed_objects()
            self.detected_num = 0
            self.my_order_manager.mode_detect()
            self.object_plan.reset_judge()
            if self.if_rogue_plan:
                self.my_art_protocol.send_object_kind(self.current_object)  # 发送目标物体种类信�?
            self.my_vision.reset_analysed_objects()
            #self.scan_message.append([self.my_car.x_current, self.my_car.y_current])  # 记录扫描状态开始时小车的位置，作为后续判断是否迷路的参�?
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物�?
            pass
        elif state == MOVE:
            # 进入搬运状态，开始搬运物�?
            self.my_plan.reset_navigate()
            self.my_moving.my_photo.reset_photo()
            pass
            # 测试
            # self.my_uart.write(f"state: {self.my_moving.current_state},moving_pt: {self.my_moving.moving_point},angle_buffer: {self.my_moving.angle_buffer}\n")
        elif state == CALIBRATE:
            # 进入校准状态，进行位置或传感器校准
            # 记录小车在哪个边�?
            self.my_vision.car_position = object_to_line_dict.get(self.current_object)
        elif state == ADJUST:
            # 进入调整状态，根据需要进行微�?
            pass
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务�?
            self.my_path.plan_path(self.data.fixed_point[3][0], self.data.fixed_point[3][1], ignore_center_rect=True)  # 规划回起始点的路�?
            self.my_path.ready_path[-1] = self.data.fixed_point[3]
            # 最后插入一个途径点便于计�?
            self.my_path.ready_path.insert(-1, [self.data.fixed_point[3][0], 10.0])
            # self.my_uart.write(f"Path: {self.my_path.ready_path}")  # 测试：打印路径点
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()
        elif state == RETREAT:
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            pass

    def exit(self):
        state = self.my_state.state
        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            self.my_state.state = NAVIGATE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0]) 
            # 退出导航状态，停止路径跟随
            self.if_send_path = False  # 重置路径发送标志位
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = SCAN  # 直接切换到扫描状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SCAN:
            # 退出扫描状态，停止寻找目标物体
            if not self.my_plan.if_finish_navigate:
                self.my_plan.reset_navigate()
                self.my_vision.reset_servo_angle()
                self.my_art_protocol.send_object_kind(self.current_object)
                self.my_state.state = MOVE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                # 如果小车并没有找到物体，直接return
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，直接回家
                '''
                self.data.current_index += 1  # 跳过当前物体，进入下一个物体的准备导航状�?
                if self.data.current_index >= self.data.total_objects_num:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                else:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                '''
        elif state == SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                self.my_vision.if_finish_servo = False
                self.my_state.state = MOVE
                self.if_transitioning = True
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                # 将openart置为等待模式
                # self.my_order_manager.finish()
                self.my_plan.reset_navigate()
                self.data.current_index += 1  # 跳过当前物体，进入下一个物体的准备导航状�?
                if self.data.current_index >= self.data.total_objects_num:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                else:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == MOVE:
            # 退出搬运状态，停止搬运动作 
            if self.current_object == 'T':
                self.last_side = 'U'
            elif self.current_object == 'S' or self.current_object == 'E':
                self.last_side = 'L'
            elif self.current_object == 'W' or self.current_object == 'B':
                self.last_side = 'R'
            else:
                self.my_plan.reset_navigate_angle()
                # 如果从车丢失物体直接返回发车区避免浪费时�?
                self.my_state.state = RETURN 
            # 若从车丢失物体，则跳过当前物�?     
            self.data.current_index += 1
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = RETREAT  # 直接切换到校准状�?
            # 此时从车丢失物体
            if self.my_moving.current_state == ADJUST:
                self.my_plan.reset_navigate_angle()
                # 如果从车丢失物体直接返回发车区避免浪费时�?
                self.my_state.state = RETURN 
            # 跳过当前物体
            self.my_moving.reset_move()  # 重置搬运标志
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更�?
            self.my_vision.reset_apriltag_calibrate()  # 重置校准标志
            if self.data.current_index >= self.data.total_objects_num:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
            else:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == ADJUST:
            # 退出调整状态，完成微调后进行必要的状态更�?
            self.my_vision.reset_orbit()
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == RETURN:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4]) 
        
            # 退出返回状态，完成返回后进行必要的状态更�?
            self.if_send_path = True
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = STOP  # 直接切换到停止状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状�?
            self.my_beep.test()  # 任务完成，发出提示音
        elif state == RETREAT:
            # 重置导航标志�?
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状�?
            if self.data.current_index >= self.data.total_objects_num:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
    
    def handle_ready_navigate(self):

        # 进入准备导航状态，做好路径规划准备和导航信息准�?
        target_x = 160
        target_y = 120
        slave_stop_threshold = 25.0
        scan_threshold = 20
        if self.data.current_index >= self.data.total_objects_num:
            self.my_state.state = RETURN
            self.if_transitioning = True
            return
        # 主车最终目标点
        main_final_pt = []#惯导先到的位�?
        if self.if_rogue_plan:
            target_x = self.data.rogue_planning[self.data.current_index][0][0]
            target_y = self.data.rogue_planning[self.data.current_index][0][1]
            self.current_object = self.data.rogue_planning[self.data.current_index][1]  # 提取当前物体种类信息
            # 便于边线处减�?
            self.my_plan.current_object = self.current_object  
            self.last_side = self.data.rogue_planning[self.data.current_index][2]
            scan_threshold=0
            # 小车导航到物体前的距�?
            stop_threshold = 20.0
            # 根据小车进入的边界信息选择合适的角度和扫描点信息
        
        if self.last_side == "L":
            target_angle = 90.0
            self.slave_navigate_message = [[self.data.fixed_point[1][0] - slave_stop_threshold, target_y+scan_threshold], target_angle]
            main_final_pt = [self.data.fixed_point[1][0], target_y+scan_threshold]
            if self.if_rogue_plan:
                self.scan_message = [[target_x - stop_threshold, target_y]]
            else:
                self.scan_message = [[self.data.fixed_point[1][0], target_y-scan_threshold]]
        elif self.last_side == "R":
            target_angle = -90.0
            self.slave_navigate_message = [[self.data.fixed_point[2][0] + slave_stop_threshold, target_y-scan_threshold], target_angle]
            main_final_pt = [self.data.fixed_point[2][0], target_y-scan_threshold]
            if self.if_rogue_plan:
                self.scan_message = [[target_x + stop_threshold, target_y]]
            else:
                self.scan_message = [[self.data.fixed_point[2][0], target_y+scan_threshold]]
        elif self.last_side == "U":
            target_angle = 180.0
            self.slave_navigate_message = [[target_x+scan_threshold, self.data.fixed_point[2][1] + slave_stop_threshold], target_angle]
            main_final_pt = [target_x+scan_threshold, self.data.fixed_point[2][1]]
            if self.if_rogue_plan:
                self.scan_message = [[target_x, target_y + stop_threshold]]
            else:
                self.scan_message = [[target_x-scan_threshold,self.data.fixed_point[2][1]]]
        else:
            target_angle = 0.0
            self.slave_navigate_message = [[target_x-scan_threshold, self.data.fixed_point[1][1] - slave_stop_threshold], target_angle]
            main_final_pt = [target_x-scan_threshold, self.data.fixed_point[1][1]-5]
            #main_final_pt = [target_x, self.data.fixed_point[1][1]-5]
            if self.if_rogue_plan:
                self.scan_message = [[target_x, target_y - stop_threshold]]
            else:
                self.scan_message = [[target_x+scan_threshold,self.data.fixed_point[1][1]-5]]
         # 进行路径规划
        self.my_path.plan_path(main_final_pt[0], main_final_pt[1])  
        self.navigate_message = [self.my_path.ready_path, target_angle]  # 准备导航信息
        self.exit()  # 退出当前状态，进入导航状�?

    def handle_navigate(self):
        # if state == NAVIGATE
        self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])
        # 主车行驶多远后给从车发送路径信�?
        dist_threshold = 20.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信�?

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状�?

    def handle_scan(self):
        def analyse_package(num):
            if not self.if_send_detect_message:
                self.if_send_detect_message = True
                self.my_order_manager.mode_detect()
            object_package=self.my_art_protocol.detect_objects_on_the_court()
            if object_package:
                self.my_vision.analyse_object_coordinate(object_package,if_cover = True)
                self.detected_num+=1
                if self.detected_num==num:
                    self.my_order_manager.finish()
                    self.if_send_detect_message = False
                    self.my_uart.write(f"1{self.my_vision.analysed_objects}\n")
        if self.detected_num < 2:
            analyse_package(2)
            self.my_plan.if_finish_navigate = False
        elif self.detected_num < 4:
            self.my_plan.navigate(path = self.scan_message)
            if self.my_plan.if_finish_navigate:
                analyse_package(4)
        elif self.detected_num == 4:
            if self.object_plan.judge_object_character(self.my_vision.analysed_objects,self.last_side):
                target = self.object_plan.plan_target
                self.my_uart.write(f"target{self.object_plan.target_objects}\n")
                self.my_uart.write(f"path{self.object_plan.path}\n")
                self.my_uart.write(f"score{self.object_plan.target_score}\n")
                if not target:
                    self.my_uart.write("False\n")
                    self.exit()
                else:
                    self.object_plan.barrier.pop(target[0])
                    self.my_moving.now_barriar=self.object_plan.barrier[:]
                    #self.my_uart.write(f"barriar{self.my_moving.now_barriar}\n")
                    self.current_object=target[1]
                    self.my_plan.current_object = self.current_object
                    self.my_vision.current_servo_object = self.current_object
                    rm = self.my_moving.ready_move([target[2],target[3]],new_side = self.last_side)
                    #self.my_uart.write(f"rm:{rm},nav_n:{len(self.my_moving.navigate_buffer)}\n")
                    if rm:self.my_plan.if_finish_navigate = False
                    self.exit()

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            x = self.my_car.x_current
            y = self.my_car.y_current
            self.my_plan.navigate(path = [[x+10.0, y], [x-10.0, y], self.navigate_message[0][-1]])
            
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.my_vision.current_servo_object:
                self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False

        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状�?

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()
        if self.my_moving.if_finish_move:
            current_object = self.current_object
            retreat_threhold = 5
            if current_object == 'T':
                if self.my_car.now_yaw<0:
                    self.retreat_message=[self.my_car.x_current+retreat_threhold, self.my_car.y_current]
                else:
                    self.retreat_message=[self.my_car.x_current-retreat_threhold, self.my_car.y_current]
            elif current_object in ['S', 'E']:
                if self.my_car.now_yaw<-PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
            elif current_object in ['B', 'W']:
                if self.my_car.now_yaw<PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
            self.exit()  # 退出当前状态，进入下一个状�?

    def handle_retreat(self):
        # if state == ADJUST
        self.my_plan.navigate(path = [self.retreat_message])

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状�?

    def handle_calibrate(self):
        # if state == CALIBRATE
        global counter
        self.my_vision.apriltag_calibrate_control()

        if self.my_vision.if_finish_calibrate:
            counter += 1
            # 延时100ms
            if counter >= 10:
                counter = 0
                self.exit()  # 退出当前状态，进入下一个状�?

    def handle_adjust(self):
        pass

    def handle_return(self):
        # if state == RETURN
        self.my_plan.navigate(path = self.my_path.ready_path)  # 返回起始�?
                
        # 主车行驶多远后给从车发送路径信�?
        dist_threshold = 50.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信�?

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状�?

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()             
