from micropython import const
import math
import gc

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
InField = const(-1)
OnLine = const(0)
OutLine = const(1)
# 多路复用器计数器
counter = 0
class MoveControl:
    def __init__(self, beep, photo, car, plan,path, plan_data,move_plan, vision_manager, state, main_protocol, art_protocol, order_manager, assist_protocol):
        self.my_beep = beep
        self.my_photo = photo
        self.vision_manager = vision_manager
        self.my_plan = plan
        self.my_path = path
        self.plan_data = plan_data
        self.my_car = car
        self.my_state = state
        self.my_main_protocol = main_protocol
        self.my_art_protocol = art_protocol
        self.my_order_manager = order_manager
        self.my_assist_protocol = assist_protocol
        self.move_plan = move_plan
        self.now_object_pt = [0.0, 0.0]
        self.record_angle = 0.0  # 记录的角度(记录小车的最初的角度)

        self.navigate_buffer = []
        self.navigate_distance=18
        self.__angle=30
        self.surrounding_points = {
            'LU': [],
            'LD': [],
            'RU': [],
            'RD': [],
            'LDD': [],
            'RDD': [],
        }
        self.now_barriar = []
        self.moving_point = []   # 搬运途径点
        self.angle_buffer = []   # 角度缓冲区
        self.next_point = []     # 下一目标点
        self.adjust_point = []   # 微调目标点
        self.moving_idx = 0      # 搬运途径点索引
        self.move_dir = 0

        self.num_send_orbit_point=0

        self.current_state = ORBIT  # 当前状态：0为环绕，1为视觉伺服，2为搬运， 3为微调
        self.if_send_orbit_command = False  # 是否发送过环绕控制指令
        self.if_start_orbit = False  # 是否开始环绕
        self.if_finish_move = False  # 是否完成搬运
        self.plan_path = []
        self.send_point = []
        gc.collect()

    # 更新物体当前坐标，已知物体在小车正前方的距离 dist
    def update_object_pos(self):
        # 当前车头朝向 (弧度)
        now_yaw = self.my_car.now_yaw
        dist = self.vision_manager.final_dist + self.vision_manager.car_radius
        # 已知世界坐标系下向北(+Y)为0度，向东(+X)为90度
        # 车头指向的正方向向量为 (sin(now_yaw), cos(now_yaw))
        self.now_object_pt = [
            self.my_car.x_current + dist * math.sin(now_yaw),
            self.my_car.y_current + dist * math.cos(now_yaw)
        ]
    def calculate_object_pos(self,point):#用扫描的一帧计算位置
        self.now_object_pt = self.vision_manager.calc_object_global_pos(point[0],point[1])
    # 构建搬运途径点列表
    def build_moving_point(self,point):
        current_index = self.plan_data.current_index
        current_object = self.vision_manager.current_servo_object
        self.moving_point.clear()
        self.moving_point.append(self.now_object_pt[:])  # 物体位置（使用切片拷贝，避免引用污染）
        if self.plan_data.if_rogue_plan:
            object_message = self.plan_data.rogue_planning[current_index]
            move_step=object_message[3]
        else:
            move_step=[]
        for item in move_step: # 搬运途径点
            if item[0] == 'x':
                self.moving_point.append([item[1], self.moving_point[-1][1]])
            elif item[0] == 'y':
                self.moving_point.append([self.moving_point[-1][0], item[1]])
        if current_object == 'T':
            self.moving_point.append([self.moving_point[-1][0], 240.0])
            self.move_dir = 0
        elif current_object in ['S', 'E']:
            self.moving_point.append([0.0, self.moving_point[-1][1]])
            self.move_dir = -90
        elif current_object in ['B', 'W']:
            self.moving_point.append([320.0, self.moving_point[-1][1]])
            self.move_dir = 90
    # 判断小车编队到下一目标点时的转向（返回基于小车坐标系的相对朝向）
    def judge_next_turn(self, current_pt, next_pt, ref_yaw=None):
        if ref_yaw is None:ref_yaw = self.record_angle
        else:ref_yaw = ref_yaw * PI / 180.0
        dx = next_pt[0] - current_pt[0]
        dy = next_pt[1] - current_pt[1]
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
        else:raise ValueError("car_angle must be one of 0, 90, 180, -90")
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
    # 搬运前的准备
    def ready_move(self,point,new_side = None):
        if not point or len(point) < 2:return False
        #self.now_object_pt = self.vision_manager.calc_object_global_pos(point[0],point[1])
        self.now_object_pt = point[:]
        if not self.vision_manager.if_in_rect(self.now_object_pt[0],self.now_object_pt[1]):
            return False
        self.moving_idx = 0
        self.current_state = ORBIT
        self.if_finish_move = False
        self.if_start_orbit = False
        self.navigate_buffer.clear()
        self.my_plan.reset_navigate()
        self.my_plan.reset_navigate_angle()
        # 重置搬运点索引
        self.moving_idx = 0
        # 构建搬运途径点列表
        self.build_moving_point(point)
        # 记录小车当前角度
        self.record_angle = self.my_car.now_yaw  # 保持弧度制供 judge_next_turn 默认使用
        current_yaw_deg = self.record_angle * 180.0 / PI
        if not new_side:
            if current_yaw_deg > -45.0 and current_yaw_deg <= 45.0: current_turn_deg = 0.0
            elif current_yaw_deg > 45.0 and current_yaw_deg <= 135.0:current_turn_deg = 90.0
            elif current_yaw_deg > 135.0 or current_yaw_deg <= -135.0:current_turn_deg = 180.0
            elif current_yaw_deg > -135.0 and current_yaw_deg <= -45.0:current_turn_deg = -90.0
        else:
            if new_side =='D':current_turn_deg = 0.0
            elif new_side =='U':current_turn_deg = 180
            elif new_side =='L':current_turn_deg = 90       
            else:current_turn_deg = -90
        self.angle_buffer.clear()
        self.get_object_square_points(current_turn_deg,15)
        # 初始参考偏航角就是当前小车所在方向（度数）
        current_ref_yaw_deg = current_turn_deg
        for i in range(len(self.moving_point) - 1):
            # 获取基于 current_ref_yaw_deg 作为参照方向时的相对转向角度
            # turn_angle 可能是返回 0.0 (前方), 90.0 (右), -90.0 (左), 180.0 (后)
            turn_angle = self.judge_next_turn(self.moving_point[i], self.moving_point[i + 1], ref_yaw=current_ref_yaw_deg)
            # 世界坐标系下小车在下一步运动期望到达的实际偏航角
            target_turn = current_ref_yaw_deg + turn_angle
            # 角度限幅到 [-180, 180)
            target_turn = (target_turn + 180.0) % 360.0 - 180.0
            angle_l=(target_turn + self.__angle + 180.0) % 360.0 - 180.0
            angle_r=(target_turn - self.__angle + 180.0) % 360.0 - 180.0
            M_PAth = []
            if turn_angle == 0.0:
                m_PAth = [self.surrounding_points['LD']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_l,current_ref_yaw_deg]
            elif turn_angle == 90.0:
                m_PAth = [self.surrounding_points['LD'],self.surrounding_points['LU']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_l,current_ref_yaw_deg]
            elif turn_angle == 180.0:
                m_PAth = [self.surrounding_points['RD'],self.surrounding_points['RU']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_l,current_ref_yaw_deg]
            elif turn_angle == -90.0:
                m_PAth = [self.surrounding_points['RD'],self.surrounding_points['RU']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_r,current_ref_yaw_deg]
            if new_side:
                if new_side =='D':self.my_path.plan_path(m_PAth[0][0],self.my_plan.plan_data.center_rect[0][1])
                elif new_side =='U':self.my_path.plan_path(m_PAth[0][0],self.my_plan.plan_data.center_rect[3][1])
                elif new_side =='L':self.my_path.plan_path(self.my_plan.plan_data.center_rect[0][0],m_PAth[0][1])   
                else:self.my_path.plan_path(self.my_plan.plan_data.center_rect[3][0],m_PAth[0][1])
                M_PAth = self.my_path.ready_path + m_PAth
            else:
                M_PAth = m_PAth
            self.navigate_buffer.append({
                            'MAIN_P':M_PAth,
                            'SLA_P':S_PAth,
                            'ANGLE':ANGle,
                        })
        self.moving_point.pop(0)  # 移除起点
        if self.vision_manager.current_servo_object == 'T':
            self.my_plan.error_x = self.my_plan.error_x_T
            self.final_dist = self.vision_manager.servo_pid.target_y_T
            self.object_radius = self.vision_manager.radius_T
            self.orbit_angle = self.vision_manager.angle_T
            self.my_plan.move_v_max = self.my_plan.move_v_max_T
        elif self.vision_manager.current_servo_object == 'S' or self.vision_manager.current_servo_object == 'E':
            self.my_plan.error_x = self.my_plan.error_x_S
            self.final_dist = self.vision_manager.servo_pid.target_y_S
            self.object_radius = self.vision_manager.radius_S
            self.orbit_angle = self.vision_manager.angle_S
            self.my_plan.move_v_max = self.my_plan.move_v_max_S
        elif self.vision_manager.current_servo_object == 'B' or self.vision_manager.current_servo_object == 'W':
            self.my_plan.error_x = self.my_plan.error_x_B
            self.final_dist = self.vision_manager.servo_pid.target_y_B
            self.object_radius = self.vision_manager.radius_B
            self.orbit_angle = self.vision_manager.angle_B
            self.my_plan.move_v_max = self.my_plan.move_v_max_B
        return True
    # 重置环绕控制标志位
    def reset_orbit(self):
        self.if_send_orbit_command = False
        self.if_start_orbit = False
        self.vision_manager.if_orbit_ready = False
        self.vision_manager.if_finish_orbit = False
        self.num_send_orbit_point=0
        self.surrounding_points.clear()
        self.navigate_buffer.clear()
        self.my_plan.reset_navigate()

    # 重置搬运控制相关变量
    def reset_move(self):
        self.moving_idx = 0
        self.moving_point.clear()
        self.angle_buffer.clear()
        self.next_point.clear()
        self.adjust_point.clear()
        self.current_state = ORBIT
        self.reset_orbit()
        self.if_finish_move = False
        gc.collect()

    # 重置小车里程计
    def reset_car_pos(self):
        current_object = self.vision_manager.current_servo_object
        light_to_center = 8.5  # 光电管到车体中心的距离
        COS = 0.707
        if current_object == 'T':
            self.my_car.y_current = 240.0 - light_to_center * COS
        elif current_object in ['S', 'E']:
            self.my_car.x_current = 0.0 + light_to_center * COS
        elif current_object in ['B', 'W']:
            self.my_car.x_current = 320.0 - light_to_center * COS
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
    def calculate_move_path(self):
        objects=self.now_barriar
        if self.move_dir==0 or self.move_dir==180:
            if self.my_car.now_yaw>0:swell_dir=-90
            else:swell_dir=90
        elif self.move_dir==-90 or self.move_dir==90:
            if self.my_car.now_yaw>-PI/2 and self.my_car.now_yaw<PI/2:swell_dir=180
            else:swell_dir=0
        else: return False
        plan_path = self.move_plan.plan_move(self.move_dir,swell_dir,objects)
        if len(plan_path) == 2:
            self.send_point=[0,0]
        elif len(plan_path) == 3:
            self.send_point=[plan_path[1][0]-self.my_car.x_current,plan_path[1][1]-self.my_car.y_current]
        else: return False
        self.plan_path = plan_path[1:]
        return True 
    # 状态过渡函数
    def state_transition(self):
        global counter
        if self.current_state == ORBIT:
            if not self.if_send_orbit_command:#若还未发消息
                self.if_send_orbit_command = True
                NAV_T=self.navigate_buffer[self.moving_idx]
                self.my_main_protocol.send_path(NAV_T['SLA_P'][0],NAV_T['ANGLE'][1],NAV_T['SLA_P'][1])
            if self.vision_manager.if_send_order == False:#若还未打开摄像头
                # 打开摄像头
                self.my_order_manager.mode_target()
                self.vision_manager.if_send_order = True#从车完成后开始视觉
            
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and\
            target_point[1] >= 40.0:
                self.vision_manager.ready_servo_and_orbit(target_point, 'adjust')
                
                self.vision_manager.reset_servo_angle()
                self.current_state = ADJUST

                self.reset_orbit()  # 重置环绕相关变量
                self.vision_manager.if_send_order = False
        elif self.current_state == ADJUST:
            # 延时50ms再进行状态过渡，确保小车已经稳定在视觉伺服的起始位置，避免过早进入搬运状态导致丢失目标
            if counter >= 5:
                order = self.my_main_protocol.get_slave_state()
                if order == "finish":
                    self.my_beep.test()
                    counter = 0
                    self.vision_manager.if_finish_servo = False
                    #self.handle_next_point()
                    if self.calculate_move_path():
                        self.my_main_protocol.send_path('M',self.move_dir,self.send_point)
                        # 在最后一个搬运点前给辅助车发送具体坐标
                        self.my_plan.reset_navigate_angle()
                        self.my_plan.reset_navigate()
                        self.current_state = MOVE
                    else:
                        self.if_finish_move = True#直接退出return
                elif order == "lost":
                    counter = 0
                    self.if_finish_move = True
            else:
                counter += 1
        elif self.current_state == MOVE:
            # 如果当前搬运点是最后一个额外增加的终点指令，说明已经完成搬运
            #self.moving_idx += 1
            #if self.moving_idx >= len(self.moving_point):
            self.if_finish_move = True
            return
            '''
            if self.vision_manager.if_send_order == False:
                # 打开摄像头
                self.my_order_manager.mode_target()
                self.vision_manager.if_send_order = True
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and\
            target_point[1] >= 40.0:
                self.vision_manager.ready_servo_and_orbit(target_point, 'servo')
                self.vision_manager.if_send_order = False
                self.my_plan.reset_navigate()   # 重置导航相关变量
                
                self.vision_manager.reset_servo_angle()
                self.current_state = SERVO
            '''
        elif self.current_state == SERVO:
            self.vision_manager.if_finish_servo = False
            self.vision_manager.reset_orbit_angle()
            self.current_state = ORBIT

    # 搬运控制函数
    def moving(self):
        if self.if_finish_move:
            return
        if self.current_state == ORBIT:
            if self.if_start_orbit == False:
                NAV_T=self.navigate_buffer[self.moving_idx]
                if NAV_T:
                    self.if_start_orbit = True
                    self.if_send_orbit_command = False
                    self.my_plan.navigate(NAV_T['MAIN_P'],NAV_T['ANGLE'][0])
            else:
                if not self.if_send_orbit_command and self.my_plan.finished_dist >= 15:
                    NAV_T=self.navigate_buffer[self.moving_idx]
                    self.my_main_protocol.send_path(NAV_T['SLA_P'][0],NAV_T['ANGLE'][1],NAV_T['SLA_P'][1])
                    self.if_send_orbit_command = True
                self.my_plan.navigate(self.navigate_buffer[self.moving_idx]['MAIN_P'],self.navigate_buffer[self.moving_idx]['ANGLE'][0])
                if self.my_plan.if_finish_navigate == True:
                    self.state_transition()
        elif self.current_state == ADJUST:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
        elif self.current_state == MOVE:
            #self.my_plan.navigate(path = [self.next_point])
            self.my_plan.navigate(path = self.plan_path)
            self.my_photo.update_photo_state()
            if self.my_photo.current_state == OutLine:
                self.reset_car_pos()
                self.my_photo.reset_photo()
                self.my_beep.test()
                self.my_plan.if_finish_navigate = True
            if self.my_plan.if_finish_navigate == True:
                self.state_transition()
        elif self.current_state == SERVO:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
