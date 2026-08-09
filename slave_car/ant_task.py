# 状态机类
from micropython import const
import gc
import math

PI = const(3.1415926)
READY_NAVIGATE = const(0) # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)         # 微调状态
RETURN = const(8)		  # 返回状态
STOP = const(9)           # 停止状态
RETREAT = const(10)       # 后退状态
KEEP_SPACE = const(11)    # 保持距离状态
counter = 0  # 计数器
object_to_line_dict = {
    'T': 'U',
    'S': 'L',
    'E': 'L',
    'W': 'R',
    'B': 'R'
}

class TaskController:
    def __init__(self,flash,beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager, art_protocal, slave_protocol):
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
        self.my_slave_protocol = slave_protocol
        self.my_flash_system = flash
        # 状态映射表：将状态常量映射到对应的处理函数
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
            # ... 其他状态
        }
        self.if_first_run = True
        self.num_clamp_factor = self.my_flash_system.find_value("NUM_CLAMP_FACTOR")
        T_dis = self.my_flash_system.find_value("TENNIS_cla_dis")
        S_dis = self.my_flash_system.find_value("SANDBAG_cla_dis")
        B_dis = self.my_flash_system.find_value("BEAR_cla_dis")
        self.clamp_distance = {'T':T_dis,'S':S_dis,'E':S_dis,'W':B_dis,'B':B_dis}
        self.navigate_message = []  # 导航信息：目标点坐标和朝向
        self.pt_buffer = []  # 目标点坐标缓冲区
        self.current_object = ''  # 当前目标物体种类
        # 标志位
        self.if_transitioning = True  # 是否正在进行状态转换
        self.current_pushed_num = 0
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

        if state == READY_NAVIGATE:
            # 进入准备导航状态，做好路径规划准备和导航信息准备
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟随
            pass
        elif state == SCAN:
            pass
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物体
            pass
        elif state == MOVE:
            num_compensation = self.current_pushed_num * self.num_clamp_factor
            self.my_moving.clamp_distance = self.clamp_distance[self.current_object]+num_compensation
            self.my_moving.ready_move(self.pt_buffer[1], self.pt_buffer[0], self.current_object)
            self.current_pushed_num += 1
            pass
        elif state == CALIBRATE:
            pass
        elif state == ADJUST:
            # 进入调整状态，根据需要进行微调
            pass
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务点
            self.my_path.plan_path(self.data.fixed_point[3][0], self.data.fixed_point[3][1], ignore_center_rect=True)  # 规划回起始点的路径
            self.my_path.ready_path[-1] = self.data.fixed_point[3]
            # 最后插入一个途径点便于计时
            self.my_path.ready_path.insert(-1, [self.data.fixed_point[3][0], 10.0])
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()

    def exit(self):
        global counter
        state = self.my_state.state
        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            if self.current_object == 'R':
                # 若当前物体信息为回程信息
                self.my_state.state = RETURN  # 直接切换到返回状态
            else:
                self.my_state.state = NAVIGATE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            # 退出导航状态，停止路径跟随
            if self.current_object == 'P':
                self.my_plan.reset_navigate_angle()  # 重置导航角度
                self.my_plan.reset_navigate()  # 重置导航标志
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.current_object == 'A':
                self.my_vision.reset_calibrate()
                if self.pt_buffer[1] == 90:
                    if self.my_car.x_current <= self.pt_buffer[0][0]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                elif self.pt_buffer[1] == -90:
                    if self.my_car.x_current >= self.pt_buffer[0][0]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                elif self.pt_buffer[1] == 0:
                    if self.my_car.y_current <= self.pt_buffer[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                elif self.pt_buffer[1] == 180:
                    if self.my_car.y_current >= self.pt_buffer[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                self.my_vision.calibrate_buffer = [[self.pt_buffer[0]],self.pt_buffer[1]]
                self.my_state.state = CALIBRATE
                self.if_transitioning=True
            else:
                #target_point = self.my_art_protocol.coordinate_receive()
                #if target_point and chr(target_point[2]) == self.current_object:
                # 计数器清零
                self.counter = 0
                self.my_vision.if_send_order = False  # 重置发送指令标志位
                self.my_vision.ready_servo_and_orbit(self.current_object, 'servo')
                self.my_vision.reset_servo_angle()
                self.my_plan.reset_navigate()  # 重置导航相关变量
                self.my_state.state = MOVE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SCAN:
            pass
        elif state == SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                # 重置环绕角度
                self.my_vision.reset_orbit_angle()
                self.my_vision.if_finish_servo = False  # 重置伺服完成标志
                self.my_state.state = MOVE  # 直接切换到搬运状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                # 将openart置为等待模式
                self.my_order_manager.finish()
                self.my_plan.reset_navigate()
                self.my_slave_protocol.send_slave_state("lost")  # 通知主车丢失物体
                self.my_plan.reset_navigate_angle()
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == MOVE:
                self.my_moving.if_finish_move = False  # 重置搬运完成标志
                self.my_plan.reset_navigate_angle()
                self.my_plan.reset_navigate()
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                if self.my_moving.current_state != NAVIGATE:
                    self.my_state.state = RETURN  # 直接切换到校准状态
                    return
                self.my_state.state = RETREAT  # 直接切换到校准状态
        elif state == CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更新
            self.my_vision.reset_calibrate()  # 重置校准标志
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物体
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == ADJUST:
            # 退出调整状态，完成微调后进行必要的状态更新
            pass
        elif state == RETURN:
            # 退出返回状态，完成返回后进行必要的状态更新
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = STOP  # 直接切换到停止状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状态
            self.my_beep.test()  # 任务完成，发出提示音
        elif state == RETREAT:
            # 重置导航标志位
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
    
    def handle_ready_navigate(self):
        global counter
        if self.if_first_run:
            counter += 1
            if counter <= 50:
                return
            
        # 进入准备导航状态，做好路径规划准备和导航信息准备
        path = self.my_slave_protocol.get_path_list()  # 从从车协议中获取路径信息
        if path:
            # 只有当路径信息为过渡或者回城时才记录目标点坐标
            horizon_stop_threshold = 10
            if path[0] not in ['P', 'R']:
                self.pt_buffer = [path[2], path[1]]  # 储存目标坐标
                self.navigate_message = []  # 收到物体坐标先不导航
            else:
                # 进行路径规划
                tx=path[2][0]
                ty=path[2][1]
                if abs(tx+1)<1e-3 and abs(ty+1)<1e-3:
                    self.navigate_message = [[-1,-1], path[1]]  # 只保留角度
                else:
                    #规划停在主车左/右侧
                    current_yaw_deg = self.my_car.now_yaw * 180.0 / PI
                    if current_yaw_deg > -45.0 and current_yaw_deg <= 45.0: 
                        current_turn_deg = 0.0
                    elif current_yaw_deg > 45.0 and current_yaw_deg <= 135.0:
                        current_turn_deg = 90.0
                    elif current_yaw_deg > 135.0 or current_yaw_deg <= -135.0:
                        current_turn_deg = 180.0
                    elif current_yaw_deg > -135.0 and current_yaw_deg <= -45.0:
                        current_turn_deg = -90.0
                    if self.if_first_run:
                        horizon_stop_threshold = 20
                    if current_turn_deg == 0 or current_turn_deg == 180:
                        tx=min(max(path[2][0]-horizon_stop_threshold,self.my_car.x_current),path[2][0]+horizon_stop_threshold)
                    else:
                        ty=min(max(path[2][1]-horizon_stop_threshold,self.my_car.y_current),path[2][1]+horizon_stop_threshold)
                    self.my_path.plan_path(tx, ty)  # 传入目标坐标进行路径规划
                    pathh = self.my_path.ready_path  # 获取规划好的路径
                    if self.if_first_run:
                        counter = 0
                        self.if_first_run = False
                        pathh.insert(0, [self.my_car.x_current, self.my_car.y_current+50.0])
                    self.navigate_message = [pathh, path[1]]  # 目标坐标和转向角度
            self.current_object = path[0]  # 当前物体种类
            self.my_plan.current_object = self.current_object  # 将当前物体种类传递给路径跟随模块
            # 测试
            # self.my_uart.write(f"Ready to navigate to {self.current_object} at {self.navigate_message[0]} with turn {self.navigate_message[1]}\r\n")  # 调试信息
            # self.my_uart.write(f"{self.my_path.ready_path}\r\n")
            self.exit()  # 退出当前状态，进入导航状态

    def handle_navigate(self):
        # if state == NAVIGATE
        if self.navigate_message:
            if self.navigate_message[0][0] == -1 and self.navigate_message[0][1] == -1:
                self.my_plan.navigate(target_turn_angle = self.navigate_message[1])
            else:
                self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])
        else:
            self.my_plan.if_finish_navigate=True#直接退出
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状态
   
    def handle_scan(self):
        # if state == SCAN
        pass

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            x = self.my_car.x_current
            y = self.my_car.y_current
            now_yaw = self.my_car.now_yaw  # 弧度，0=北(+Y)，90°=东(+X)
            # 车身右方(+X): (cos(now_yaw), -sin(now_yaw))
            # 车身左方(-X): (-cos(now_yaw), sin(now_yaw))
            right_x = x + 15.0 * math.cos(now_yaw)
            right_y = y - 15.0 * math.sin(now_yaw)
            left_x = x - 15.0 * math.cos(now_yaw)
            left_y = y + 15.0 * math.sin(now_yaw)
            self.my_plan.navigate(path = [[right_x, right_y], [left_x, left_y], self.pt_buffer[0]], target_turn_angle = self.pt_buffer[1])
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.current_object:
                self.my_vision.if_send_order = False
                self.my_vision.ready_servo_and_orbit(chr(target_point[2]), 'servo', target_point)
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False
                self.my_order_manager.mode_target() # 打开目标识别模式
        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()
        if self.my_moving.if_finish_move:
            current_object = self.current_object
            retreat_threhold = 10
            self.retreat_message = [self.my_car.x_current, self.my_car.y_current]
            if current_object == 'T':
                self.my_vision.car_position = 'U'
                if self.my_car.now_yaw<0:
                    self.retreat_message=[self.my_car.x_current+retreat_threhold, self.my_car.y_current]
                else:
                    self.retreat_message=[self.my_car.x_current-retreat_threhold, self.my_car.y_current]
            elif current_object in ['S', 'E']:
                self.my_vision.car_position = 'L'
                if self.my_car.now_yaw<-PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
            elif current_object in ['B', 'W']:
                self.my_vision.car_position = 'R'
                if self.my_car.now_yaw<PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_retreat(self):
        # if state == ADJUST
        self.my_plan.navigate(path = [self.retreat_message])
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状态
    
    def handle_calibrate(self):
        if self.my_vision.if_finish_calibrate:
            self.exit()
            return
        
        if self.my_vision.if_lost_object == False:
            self.my_vision.apriltag_calibrate_control()
        else:
            # 控制小车前后移动寻找apriltag码
            self.my_plan.navigate(path = self.my_vision.lost_path)

            target_point = self.my_art_protocol.apriltag_receive()
            if target_point:
                self.my_plan.reset_navigate()
                self.my_vision.counter = 0
                self.my_vision.calibrate_times = 0
                self.my_vision.if_lost_object, self.my_vision.if_gain_calibrate_angle = False, False
            
            if self.my_plan.if_finish_navigate:
                self.exit()
    
    def handle_adjust(self):
        # if state == ADJUST
        pass

    def handle_return(self):
        # if state == RETURN
        print("\nreturn\n")
        self.my_plan.navigate(path = self.my_path.ready_path)  # 返回起始点
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状态

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()
