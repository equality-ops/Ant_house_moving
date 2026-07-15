from micropython import const
import math
import gc

# 计数器
counter = 0

PI = const(3.1415926)
READY_NAVIGATE = const(0)   # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)           # 微调状态
RETURN = const(8)		    # 返回状态
STOP = const(9)           # 停止状态
OutLine = const(1)

# 搬运控制类
class MoveControl:
    def __init__(self,flash_sys, beep, photo, uart, car, plan, path_plan, plan_data, vision_manager, state, slave_protocol, art_protocol, order_manager):
        self.my_beep = beep
        self.my_photo = photo
        self.my_uart = uart
        self.vision_manager = vision_manager
        self.my_plan = plan
        self.my_path = path_plan
        self.plan_data = plan_data
        self.my_car = car
        self.my_state = state
        self.my_slave_protocol = slave_protocol
        self.my_art_protocol = art_protocol
        self.my_order_manager = order_manager
        self.flash_sys = flash_sys
        self.next_orbit_angle = 0.0  # 下一环绕角度
        self.move_pt_buffer = []     # 搬运目标点缓冲区
        self.next_point = []     # 下一目标点
        self.adjust_point = []   # 微调目标点
        self.now_object_pt = []
        self.clamp_distance = 3
        self.push_postion = [1,0]
        self.plan_path = []
        self.angle_T = self.flash_sys.find_value("angle_T")
        self.angle_S = self.flash_sys.find_value("angle_S")
        self.angle_B = self.flash_sys.find_value("angle_B")
        self.current_state = ORBIT  # 当前状态：0为环绕，1为视觉伺服，2为搬运， 3为微调
        self.if_send_to_main = False  # 是否向art发送完成信号
        self.if_finish_move = False  # 是否完成搬运
        self.if_get_orbit_angle = False  # 是否获取环绕角度
        self.if_to_the_top = False

        self.navigate_buffer ={
                        'SLA_P':[],
                        'ANGLE':[0,0],
        }
        self.navigate_distance=20
        self.__angle=30
        self.surrounding_points = {
            'LU': [],
            'LD': [],
            'RU': [],
            'RD': [],
            'LDD': [],
            'RDD': [],
        }

        gc.collect()

    def reset_orbit(self):
        self.if_get_orbit_angle = False
        self.vision_manager.if_orbit_ready = False
        self.vision_manager.if_finish_orbit = False

    def get_object_square_points(self,car_angle,L):#寻找物体周围点位
        a=self.navigate_distance
        if car_angle == 0:
            forward = (0, 1)
            right = (1, 0)
        elif car_angle == 90:
            forward = (1, 0)
            right = (0, -1)
        elif car_angle == 180:
            forward = (0, -1)
            right = (-1, 0)
        elif car_angle == -90:
            forward = (-1, 0)
            right = (0, 1)
        else:
            raise ValueError("car_angle must be one of 0, 90, 180, -90")
        fx, fy = forward
        rx, ry = right
        lx, ly = -rx, -ry
        LU = [self.now_object_pt[0] + lx * a + fx * a, self.now_object_pt[1] + ly * a + fy * a]
        LD = [self.now_object_pt[0] + lx * a - fx * a, self.now_object_pt[1] + ly * a - fy * a]
        RU = [self.now_object_pt[0] + rx * a + fx * a, self.now_object_pt[1] + ry * a + fy * a]
        RD = [self.now_object_pt[0] + rx * a - fx * a, self.now_object_pt[1] + ry * a - fy * a]

        # 在 LD/RD 基础上，继续向靠近小车方向移动 L，也就是 -forward
        LDD = [LD[0] - fx * L, LD[1] - fy * L]
        RDD = [RD[0] - fx * L, RD[1] - fy * L]
        self.surrounding_points =  {
            'LU': LU,
            'LD': LD,
            'RU': RU,
            'RD': RD,
            'LDD': LDD,
            'RDD': RDD,
        }
    def judge_next_turn(self,sp,ref_yaw=None):
        if ref_yaw is None:ref_yaw = self.record_angle
        else:ref_yaw = ref_yaw * PI / 180.0
        st = [120,160]
        if sp == 'T': next_pt = [st[0],240] 
        elif sp in ['S','E']: next_pt = [0,st[1]]
        elif sp in ['W','B']: next_pt = [320,st[1]]
        dx = next_pt[0] - st[0]
        dy = next_pt[1] - st[1]
        # 将世界坐标系下的差值投影到小车坐标系 (按 Y轴为车头前方，X轴为车身右侧 进行转换)
        # 根据世界坐标向北为0度，向东为90度的定义推导的旋转变换
        cy = dx * math.sin(ref_yaw) + dy * math.cos(ref_yaw)
        cx = dx * math.cos(ref_yaw) - dy * math.sin(ref_yaw)
        if abs(cx) > abs(cy):
            if cx > 0:return 90.0  # 车身右侧
            else:return -90.0  # 车身左侧
        else:
            if cy > 0:return 0.0  # 车头前方
            else:return 180.0  # 车尾后方

    def ready_move(self, current_ref_yaw_deg, point, sp):
        self.record_angle = self.my_car.now_yaw  # 保持弧度制供 judge_next_turn 默认使用
        current_yaw_deg = self.record_angle * 180.0 / PI
        if current_yaw_deg > -45.0 and current_yaw_deg <= 45.0: 
            current_turn_deg = 0.0
            now_side = 'D'
        elif current_yaw_deg > 45.0 and current_yaw_deg <= 135.0:
            current_turn_deg = 90.0
            now_side = 'L'
        elif current_yaw_deg > 135.0 or current_yaw_deg <= -135.0:
            current_turn_deg = 180.0
            now_side = 'U'
        elif current_yaw_deg > -135.0 and current_yaw_deg <= -45.0:
            current_turn_deg = -90.0
            now_side = 'R'
        # 初始参考偏航角就是当前小车所在方向（度数）
        turn_angle = 0.0
        turn_angle = self.judge_next_turn(sp,current_ref_yaw_deg)
        target_turn = current_ref_yaw_deg + turn_angle
        # 角度限幅到 [-180, 180)
        target_turn = (target_turn + 180.0) % 360.0 - 180.0
        car_postion = target_turn
        angle_l=(target_turn + self.__angle + 180.0) % 360.0 - 180.0
        angle_r=(target_turn - self.__angle + 180.0) % 360.0 - 180.0
        self.if_to_the_top = False
        if turn_angle == 0.0:
            self.if_to_the_top = True
            self.now_object_pt = point
            self.get_object_square_points(current_ref_yaw_deg, 18)
            sla_p = [self.surrounding_points['LD']]
            angle = angle_l
            car_postion += 90
        elif turn_angle == 90.0:
            sla_p = [point]
            angle = angle_r
            car_postion -= 90
        elif turn_angle == 180.0:
            sla_p = [point]
            angle = angle_r
            car_postion -= 90
        elif turn_angle == -90.0:
            sla_p = [point]
            angle = angle_l
            car_postion += 90
        car_postion = 180 - (180 - car_postion) % 360
        if car_postion<=90+0.01 and car_postion>=90-0.01:self.push_postion = [1,0]
        elif car_postion<=0.01 and car_postion>=-0.01:self.push_postion = [0,1]
        elif car_postion<=-90+0.01 and car_postion>=-90-0.01:self.push_postion = [-1,0]
        else :self.push_postion = [0,-1]
        if now_side =='D': self.my_path.plan_path(sla_p[0][0],min(self.my_plan.plan_data.center_rect[0][1],sla_p[0][1]))
        elif now_side =='U': self.my_path.plan_path(sla_p[0][0],max(self.my_plan.plan_data.center_rect[3][1],sla_p[0][1]))
        elif now_side =='L': self.my_path.plan_path(min(self.my_plan.plan_data.center_rect[0][0],sla_p[0][0]),sla_p[0][1])   
        else: self.my_path.plan_path(max(self.my_plan.plan_data.center_rect[3][0],sla_p[0][0]),sla_p[0][1])
        sla_p = self.my_path.ready_path + sla_p
        self.navigate_buffer={
                    'SLA_P':sla_p,
                    'ANGLE':[angle,current_turn_deg],
                }
        self.if_get_orbit_angle=True
        self.current_state = NAVIGATE
        self.if_finish_move = False
        self.if_send_to_main = False
        self.my_plan.reset_navigate()
        self.my_plan.reset_navigate_angle()

    # 重置小车里程计
    def reset_car_pos(self):
        current_object = self.vision_manager.current_servo_object
        # 经验修正值
        correction = 2.0
        if current_object == 'T':
            self.my_car.y_current = 240.0 - correction
        elif current_object in ['S', 'E']:
            self.my_car.x_current = 0.0 + correction
        elif current_object in ['B', 'W']:
            self.my_car.x_current = 320.0 - correction

    # 计算微调的目标点
    def calculate_adjustment_point(self, fixed_dist = 5.0):
        # 当前车头朝向 (弧度)
        now_yaw = self.my_car.now_yaw
        # 已知世界坐标系下向北(+Y)为0度，向东(+X)为90度
        # 车头指向的正方向向量为 (sin(now_yaw), cos(now_yaw))
        # 逆着车头方向，就是向车身正后方偏移 fixed_dist 的距离
        target_x = self.my_car.x_current - fixed_dist * math.sin(now_yaw)
        target_y = self.my_car.y_current - fixed_dist * math.cos(now_yaw)
        
        self.adjust_point = [target_x, target_y]

    # 处理主车发送的下一搬运途径点
    def handle_next_point(self, move_pt_buffer):
        direct = move_pt_buffer[2][0]
        coord_val = move_pt_buffer[2][1]
        # 根据指令计算下一目标点坐标，'x'表示在x轴方向上调整，'y'表示在y轴方向上调整
        if direct == float(ord('x')):
            self.next_point = [self.my_car.x_current + coord_val, self.my_car.y_current]
        elif direct == float(ord('y')):
            self.next_point = [self.my_car.x_current, self.my_car.y_current + coord_val]
    def caculate_move_path(self,path):
        try:
            twist_clamp_factor = 1.05
            dx1=path[2][0]
            dy1=path[2][1]
            if path[1] == 0:
                pl=[self.my_car.x_current+dx1,self.plan_data.FIELD_H+20]
            elif path[1] == 180:
                pl=[self.my_car.x_current+dx1,-20]
            elif path[1] == 90:
                pl=[self.plan_data.FIELD_W+20,self.my_car.y_current+dy1]
            elif path[1] == -90:
                pl=[-20,self.my_car.y_current+dy1]
            dy = abs(pl[1]-self.my_car.y_current)
            dx = abs(pl[0]-self.my_car.x_current)
            now_clamp = self.clamp_distance * 0.005 * (dx + dy)
            p2 = [pl[0] + self.push_postion[0] * now_clamp,
                  pl[1] + self.push_postion[1] * now_clamp,]
            p2_t = [pl[0] + self.push_postion[0] * now_clamp*twist_clamp_factor,
                    pl[1] + self.push_postion[1] * now_clamp*twist_clamp_factor,]
            p_m = [self.my_car.x_current + dx1,
                   self.my_car.y_current + dy1,]
            if self.push_postion[0] != 0:
                total_dy = abs(pl[1] - self.my_car.y_current)
                if total_dy <= 1e-6:return []
                ratio = abs(dy1) / total_dy
                self.my_plan.keep_x_or_y_v = False
            elif self.push_postion[1] != 0:
                # y 内收，保持 vx
                total_dx = abs(pl[0] - self.my_car.x_current)
                if total_dx <= 1e-6:return []
                ratio = abs(dx1) / total_dx
                self.my_plan.keep_x_or_y_v = True
            else:return []
            if dx1==0 and dy1==0:
                self.my_plan.fitting_path_ = [[self.my_car.x_current,self.my_car.y_current],p2]
                return [pl]
            p1 = [p_m[0] + ratio * self.push_postion[0] * now_clamp,
                  p_m[1] + ratio * self.push_postion[1] * now_clamp,]
            self.my_plan.fitting_path_ = [[self.my_car.x_current,self.my_car.y_current],p1,p2_t]
            return [p_m,pl]
        except:return []
    # 状态过渡函数
    def state_transition(self):
        global counter
        if self.current_state == NAVIGATE:
            counter += 1

            if self.vision_manager.if_send_order == False:
                self.my_order_manager.mode_target()
                self.my_art_protocol.send_object_kind(self.vision_manager.current_servo_object)
                self.my_art_protocol.clear_uart_buffer()
                self.vision_manager.if_send_order = True
            # 延时500ms
            if counter >= 50:
                # 重置计数器
                counter = 0
                self.my_plan.reset_navigate()
                self.vision_manager.reset_servo_angle()
                self.reset_orbit() # 重置环绕相关变量
                self.plan_path = []
                self.vision_manager.if_finish_servo = False
                self.vision_manager.if_lost_object = True
                self.current_state = SERVO
                return 
            target_point = self.my_art_protocol.coordinate_receive()
            #self.my_uart.write(f"wait:{self.vision_manager.current_servo_object}, tp:{target_point}, send:{self.vision_manager.if_send_order}\n")
            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object:
                self.vision_manager.ready_servo_and_orbit(chr(target_point[2]), 'servo',target_point)
                self.vision_manager.reset_servo_angle()
                self.my_plan.reset_navigate()
                self.reset_orbit() # 重置环绕相关变量
                self.plan_path = []
                self.current_state = SERVO
        elif self.current_state == ORBIT:
            if self.if_send_to_main == False:
                # 通知主车已完成当前环绕
                self.my_slave_protocol.send_slave_state("finish")
                self.if_send_to_main = True
                self.my_plan.fitting_path_ = []
            if self.vision_manager.if_send_order == True:
                self.my_order_manager.finish() # 关闭目标识别模式
                self.vision_manager.if_send_order = False
            rec_path = self.my_slave_protocol.get_path_list()
            if rec_path and rec_path[0] == 'R':return
            if rec_path and rec_path[0] == 'M':
                self.plan_path = self.caculate_move_path(rec_path)
                if not self.plan_path:return
                # 测试
                self.my_beep.test()
                self.vision_manager.if_finish_servo = False
                self.if_send_to_main = False
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.my_plan.move_state = MOVE
                self.current_state = MOVE
        elif self.current_state == MOVE:
            if self.my_plan.if_near_line or self.my_plan.if_finish_navigate:
                self.my_plan.reset_navigate()
                self.my_plan.if_near_line = False
                self.if_finish_move = True
                self.my_plan.fitting_path_ = []
                self.my_plan.move_state = NAVIGATE
                self.current_state = NAVIGATE
            else:
                target_point = self.my_art_protocol.coordinate_receive()
                if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and\
                target_point[1] >= 30.0:
                    self.vision_manager.ready_servo_and_orbit(chr(target_point[2]), 'servo',target_point)
                    self.vision_manager.reset_servo_angle()
                    self.my_plan.reset_navigate()  # 重置导航相关变量
                    self.current_state = SERVO
        elif self.current_state == ADJUST:
            pass
        elif self.current_state == SERVO:
            if self.vision_manager.if_lost_object:
                # 若丢失物体则给从车发送丢失消息
                self.my_slave_protocol.send_slave_state("lost")
                self.if_finish_move = True
                return
            self.vision_manager.if_finish_servo = False
            self.vision_manager.if_finish_orbit = False
            self.vision_manager.if_orbit_ready = False
            self.vision_manager.reset_orbit_angle()
            
            if self.if_to_the_top:
                self.vision_manager.if_finish_orbit = True#直接跳过旋转
            self.current_state = ORBIT


    # 搬运控制函数
    def moving(self):
        if self.if_finish_move:
            return
        if self.current_state == NAVIGATE:
            NAV_T=self.navigate_buffer
            if self.if_to_the_top:self.my_plan.navigate(NAV_T['SLA_P'],NAV_T['ANGLE'][0])
            else:self.my_plan.navigate(NAV_T['SLA_P'],NAV_T['ANGLE'][1])#保持现在角度
            if self.my_plan.if_finish_navigate == True:
                self.state_transition()
                return
        elif self.current_state == ORBIT:
            if self.vision_manager.if_finish_orbit:
                self.state_transition() # 退出当前状态，进入搬运状态
                return
            NAV_T=self.navigate_buffer
            self.vision_manager.orbit_control(NAV_T['ANGLE'][0])
        elif self.current_state == MOVE:
            self.my_photo.update_photo_state()
            if self.my_photo.current_state == OutLine:
                self.reset_car_pos()
                self.my_photo.reset_photo()
                self.my_beep.test()
                self.my_plan.if_near_line = True
                self.my_plan.if_finish_navigate = True
            self.my_plan.navigate(path = self.plan_path)
            if self.my_plan.if_finish_navigate == True:
                self.state_transition()
        elif self.current_state == ADJUST:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
        elif self.current_state == SERVO:
            if self.vision_manager.if_lost_object == False:
                self.vision_manager.visual_servo_control()
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
                self.my_plan.navigate(path = [[right_x, right_y], [left_x, left_y]])
                target_point = self.my_art_protocol.coordinate_receive()
                if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object:
                    self.vision_manager.ready_servo_and_orbit(chr(target_point[2]), 'servo',point = [target_point[0],target_point[1]])
                    self.my_plan.reset_navigate()
                    self.vision_manager.if_lost_object = False

            if self.vision_manager.if_finish_servo or self.my_plan.if_finish_navigate:
                self.state_transition()  # 退出当前状态
