import math

# 状态机制
class StateMachine:
    def __init__(self):
        # state 模式：
        self.READY_NAVIGATE = 0  # 准备导航状态（主车等待从车准备好）
        self.NAVIGATE = 1       # 导航状态
        self.SCAN = 2           # 扫描状态
        self.SERVO = 3          # 视觉伺服状态
        self.ORBIT = 4          # 环绕状态
        self.MOVE = 5           # 搬运状态
        self.CALIBRATE = 6      # 校准状态
        self.RETURN = 7		    # 返回状态
        self.STOP = 8           # 停止状态
        self.REVERSE_ORBIT  = 9 # 反向环绕状态
        
        self.if_move_easy_object = False   # 是否搬运过易搬运物体的标志位（搬运过易搬运物体后在返回起点时不避开矩形区域）
        self.state = self.READY_NAVIGATE  # 初始状态为准备导航状态
        self.state_work = -1 # 阶段变量

# 路径和速度规划相关常量
class Plan_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 地图固定点坐标
        # fixed_point[0]为主车起点，fixed_point[1][2][3][4]分别为矩形区域下、左、上、右扫描起始点，[5][6][7][8]分别为矩形区域下、左、上、右扫描结束点
        self.fixed_point = [[0.0, 0.0], [110.0, 50.0], [90.0, 90.0], [210.0, 190.0], [230.0, 150.0], [210.0, 50.0], [90.0, 150.0], [110.0, 190.0], [230.0, 90.0]]  # type: list
        # 矩形区域四角点坐标
        self.rectangle_corners = [[100.0, 60.0], [100.0, 180.0], [220.0, 180.0], [220.0, 60.0]]
        # 目标物品坐标及种类信息
        # 'T'为网球， 'S'为沙袋，'B'为玩具熊
        self.object = []    # type: list
        # 实际的物体坐标
        self.object_real = []   # type: list    
        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 当前避障路径中的目标点索引
        self.current_aimed_point_index = 0    # type: int
        # 时间计数器
        self.time_counter = 0          # type: int
        # 路径点切换时间阈值（用于过渡）
        self.plan_point_transition_T = self.flash_sys.find_value("plan_point_transition_T")
        # 长短距离标志
        self.LONG_DISTANCE = 1
        self.MID_DISTANCE = 2
        self.SHORT_DISTANCE = 3

class Plan:
    def __init__(self, flash_sys, plan_data: Plan_data, math, car, state: StateMachine, order_manager, my_uart3, beep, art_protocol, sin_diff_fil, cos_diff_fil):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入数学常量对象
        self.MATH = math
        # 注入小车位置对象
        self.my_car = car
        # 注入状态机对象
        self.my_state = state
        # 注入无线通信对象
        self.my_uart3 = my_uart3
        # 注入指令管理对象
        self.my_order_manager = order_manager
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入openart串口解析对象
        self.my_art_protocol = art_protocol
        # 注入主车正余弦滑动平均滤波器对象
        self.sin_diff_fil = sin_diff_fil
        self.cos_diff_fil = cos_diff_fil

        # 速度规划相关常量
        self.min_start_v = self.flash_sys.find_value("min_start_v")           # type: int  # 最小制动速度
        self.dead_zone_v = self.flash_sys.find_value("dead_zone_v")         # type: int  # 死区启动速度
        self.long_v_max = self.flash_sys.find_value("long_v_max")           # type: int  # 长距离时的最大速度
        self.short_v_max = self.flash_sys.find_value("short_v_max")          # type: int  # 短距离时的最大速度
        self.transit_v = self.flash_sys.find_value("transit_v")              # type: int  # 过渡阶段速度
        self.move_v_max = self.flash_sys.find_value("move_v_max")          # type: int  # 搬运物品时的最大速度
        self.scan_v_max = self.flash_sys.find_value("scan_v_max")          # type: int  # 扫描时的最大速度
        self.BOOST = 1                  # type: int  # 死区启动标志位
        self.TRANSIT = 2                # type: int  # 过渡阶段标志位
        self.DEC = 3                    # type: int  # 减速阶段标志位
        self.STOP = 4                   # type: int  # 停止标志位
        self.v_target = 0               # type: int  # 目标速度
        # 速度规划阶段变量
        self.v_max = 0                  # type: int    # 本次移动规划的最大速度
        self.j = 0                      # type: float  # 加加速度    
        self.dec_distance = 0.0         # type: float  # 减速距离
        self.dec_steps = 0              # type: int    # 减速距离对应的步数
        self.stage = self.STOP          # type: int    # 速度规划阶段标志位
        self.finish_building = False    # type: int    # 检验减速速度表是否构建完成的标志位
        # 死区启动相关变量
        self.elapsed_time = 0           # type: int   # 死区启动已用时间计数器
        self.boost_duration = 0         # type: int   # 死区启动持续时间计数器
        self.boost_time_threshold = 60  # type: int   # 死区启动时间阈值
        self.dec_speed_index = 0        # type: int   # 减速速度表索引
        # 路径规划相关变量
        self.ideal_target_x = 0.0        # type: float
        self.ideal_target_y = 0.0        # type: float
        self.real_target_x = 0.0         # type: float
        self.real_target_y = 0.0         # type: float
        self.target_yaw = 0.0            # type: float
        # 测试，开始时给一点扰乱角度
        # self.turn_angle_target = 20.0     # type: float
        self.turn_angle_target = 0.0     # type: float
        self.error_correct_x = 0.0       # type: float
        self.error_correct_y = 0.0       # type: float
        self.calibrate_angle = 0.0       # type: float # 摄像头识别到的矫正角度
        # 判断小车是否到达目标点的阈值
        self.plan_arrive_threshold = self.flash_sys.find_value("plan_arrive_threshold")  # type: float
        self.total_distance = 0.0       # type: float
        self.finished_distance = 0.0    # type: float
        # 到终点的剩余距离
        self.rest_distance = 0.0        # type: float
        # 当前与下一避障目标点的距离
        self.current_rest_dis = 0.0     # type: float
        # 到过度点的剩余距离
        self.rest_transition_distance = 0.0       # type: float
        # 目标路径
        self.path_points = []      # type: list
        # 当前路径（用于避开矩形区域）
        self.current_path = []     # type: list
        # 距离长短标志位
        self.dis_flag = None
        # 绕行障碍物方向指示
        self.direct = 0
        # 标志位
        self.arrive_flag = False            # type: bool  # 判断是否到达目标点标志位
        self.if_pass_transit_point = False  # type: bool  # 判断是否到达过渡点标志位
        self.transition_flag = False        # type: bool  # 判断是否过渡完成标志位
        self.if_finish_turn = False         # type: bool  # 判断是否完成转角调整标志位
        self.if_send_path = False           # type: bool  # 判断是否向从车发送路径标志位
        self.if_set_path = False            # type: bool  # 判断是否设置路径标志位
        self.finish_navigate = False        # type: bool  # 判断是否完成导航标志位
        self.return_to_scan_point = False   # type: bool  # 判断是否返回扫描点标志位

    def _ease_out_quad(self, t):
        """二次缓出曲线，用于快速启动"""
        return t ** 2
    
    # 构建减速速度表
    def build_dec_speed_list(self, i):
        if self.finish_building == False:
            # 计算加加速度
            temp_dec_distance = self.dec_distance / self.my_car.speed_conversion_gamma
            self.j = (self.v_max ** 3) / (temp_dec_distance ** 2) 
            # 计算减速总时间
            if self.j == 0:
                self.half_time = 0
            else:
                self.half_time = math.sqrt(self.v_max / self.j)
            self.total_time = 2 * self.half_time
            # 计算减速距离对应的速度点个数
            self.dec_lenth = int(temp_dec_distance) * 10 + 1
            # 将标志位设为True
            self.finish_building = True
        else:
            i = i / self.dec_lenth * self.total_time
            if i >= self.total_time / 2:
                v = int(-0.5 * self.j * (i ** 2) + 2 * self.j * i * self.half_time - self.j * (self.half_time ** 2))
            else:
                v = int(0.5 * self.j * (i ** 2))
            return v

    # 速度规划函数
    def planning_speed(self):
        if self.arrive_flag == False:
            if self.stage == self.STOP:
                self.stage = self.BOOST
            elif self.stage == self.BOOST:
                self.elapsed_time += 1
                if self.elapsed_time <= self.boost_time_threshold:
                    # 计算目标速度
                    self.v_target = self.dead_zone_v+ int(self._ease_out_quad(self.elapsed_time / self.boost_time_threshold) * (self.v_max - self.dead_zone_v))
                else:
                    self.v_target = self.v_max
                    self.stage = self.TRANSIT
                    self.elapsed_time = 0
                    # 测试
                    # self.my_uart3.write("boost_finish\n")
            elif self.stage == self.TRANSIT:
                if self.current_rest_dis < 30.0 and self.if_pass_transit_point == False:
                    self.v_target = int(self.current_rest_dis/ 20.0 * (self.v_max - self.transit_v) + self.transit_v)
                else:
                    if self.v_target < self.v_max:
                        if self.elapsed_time <= self.boost_time_threshold:
                        # 缓慢恢复到巡航速度
                            self.v_target = self.transit_v + int(self._ease_out_quad(self.elapsed_time / self.boost_time_threshold) * (self.v_max - self.transit_v))
                            self.elapsed_time += 1
                        else:
                            self.v_target = self.v_max
                            self.elapsed_time = 0
                    else:
                        self.v_target = self.v_max
                        self.elapsed_time = 0   
                if self.rest_distance < self.dec_distance:
                    self.stage = self.DEC
            elif self.stage == self.DEC:
                if self.rest_distance < self.dec_distance:
                    self.dec_speed_index = int((self.rest_distance / self.dec_distance) * self.dec_lenth)
                    self.v_target = self.build_dec_speed_list(self.dec_speed_index)
                    
                if self.v_target <= self.min_start_v:
                    self.v_target = self.min_start_v
                    self.dec_speed_index = 0
        else:
            self.v_target = 0
            self.stage = self.STOP
            self.finish_building = False

    # 路径规划函数，用于避开中心矩形区域
    def path_planning(self, target_x: float, target_y: float):
        # 避免除0错误
        if target_x - self.my_car.x_current == 0:
            k = float('inf')  # 处理垂直线的情况，斜率趋近于无穷大
        else:
            k = (target_y - self.my_car.y_current) / (target_x - self.my_car.x_current)
        # 记录在直线上方的矩形角点
        record_corners = []
        # 记录角点坐标与直线上坐标的纵坐标的总和
        sum = 0.0
        for point in self.plan_data.rectangle_corners:
            if k == float('inf'):
                diff = point[0] - self.my_car.x_current
            else:
                diff = k * (point[0] - self.my_car.x_current) + self.my_car.y_current - point[1]
            
            sum += diff
            if diff < 0:
                record_corners.append(point)
        
        final_path = []
        # 根据record_corners的值来判断目标点相对于矩形的位置关系，从而选择合适的避障路径
        if len(record_corners) == 0 or len(record_corners) == 4:
            # 目标点在矩形的同一侧，直接规划为目标点
            pass
        elif len(record_corners) == 1:
            if record_corners[0][0] > max(self.my_car.x_current, target_x) or record_corners[0][0] < min(self.my_car.x_current, target_x):
                pass
            else:
                final_path = record_corners
        elif len(record_corners) == 3:
            record_corners = [x for x in self.plan_data.rectangle_corners if x not in record_corners]
            if record_corners[0][0] > max(self.my_car.x_current, target_x) or record_corners[0][0] < min(self.my_car.x_current, target_x):
                pass
            else:
                final_path = record_corners
        elif len(record_corners) == 2:
            if k == float('inf'):
                if (self.my_car.y_current <= self.plan_data.rectangle_corners[0][1] and target_y <= self.plan_data.rectangle_corners[0][1]) or (self.my_car.y_current >= self.plan_data.rectangle_corners[1][1] and target_y >= self.plan_data.rectangle_corners[1][1]):
                    pass
                else:
                    if sum > 0.0:
                        final_path =  record_corners
                    else:
                        final_path = [x for x in self.plan_data.rectangle_corners if x not in record_corners]
            else:
                if max(self.my_car.x_current, target_x) < self.plan_data.rectangle_corners[0][0] or min(self.my_car.x_current, target_x) > self.plan_data.rectangle_corners[2][0] or max(self.my_car.y_current, target_y) < self.plan_data.rectangle_corners[0][1] or min(self.my_car.y_current, target_y) > self.plan_data.rectangle_corners[2][1]:
                    pass
                else:
                    if sum > 0.0:
                        final_path =  record_corners
                    else:
                        final_path = [x for x in self.plan_data.rectangle_corners if x not in record_corners]

        if len(final_path) > 1:
            final_path.sort(key=lambda p: (p[0] - self.my_car.x_current)**2 + (p[1] - self.my_car.y_current)**2)
        return final_path

    # 设置目标点坐标
    def set_target_point(self, x: float, y: float):
        # 重置当前路径和相关索引
        self.plan_data.current_aimed_point_index = 0
        
        # 搬运，扫描，视觉伺服，apriltag矫正，环绕，或返回模式下不需要避开矩形区域行驶
        if self.my_state.state == self.my_state.NAVIGATE and self.return_to_scan_point == False:
            # 进行避障路径规划
            self.current_path = self.path_planning(x, y)
        else:   
            self.current_path = []
            self.return_to_scan_point = False

        # 理想条件下的目标坐标
        self.ideal_target_x = x
        self.ideal_target_y = y

        # 根据当前规划好的避障路径来估计航向
        if len(self.current_path) > 0:
            dx = self.ideal_target_x - self.current_path[-1][0]
            dy = self.ideal_target_y - self.current_path[-1][1]
        else:
            dx = self.ideal_target_x - self.my_car.x_current
            dy = self.ideal_target_y - self.my_car.y_current
        
        blurry_yaw = -math.atan2(-dx, dy) * 180.0 / self.MATH.PI - self.my_car.now_yaw * 180.0 / self.MATH.PI  # 计算大致航向角，单位：度
        if blurry_yaw > 180.0:
            blurry_yaw -= 360.0
        elif blurry_yaw < -180.0:
            blurry_yaw += 360.0
        # 根据大致航向角选择合适的坐标修正量（解决因惯性造成的打滑问题）
        if blurry_yaw >= -30.0 and blurry_yaw < 30.0:
            self.error_correct_x = 0.0
            self.error_correct_y = 0.9
        elif blurry_yaw >= 30.0 and blurry_yaw < 60.0:
            self.error_correct_x = 0.5
            self.error_correct_y = 0.7
        elif blurry_yaw >= 60.0 and blurry_yaw < 120.0:
            self.error_correct_x = 0.9
            self.error_correct_y = 0.4
        elif blurry_yaw >= 120.0 and blurry_yaw < 150.0:
            self.error_correct_x = 0.4
            self.error_correct_y = -0.8
        elif blurry_yaw >= 150.0 and blurry_yaw <= 180.0 or blurry_yaw >= -180.0 and blurry_yaw < -150.0:
            self.error_correct_x = 0.0
            self.error_correct_y = 0.0
        elif blurry_yaw >= -150.0 and blurry_yaw < -120.0:
            self.error_correct_x = -0.6
            self.error_correct_y = -0.7
        elif blurry_yaw >= -120.0 and blurry_yaw < -60.0:
            self.error_correct_x = -0.9
            self.error_correct_y = -1.0
        elif blurry_yaw >= -60.0 and blurry_yaw < -30.0:
            self.error_correct_x = -0.6
            self.error_correct_y = 0.7
        
        # 实际条件下的目标坐标
        self.real_target_x = self.ideal_target_x + self.error_correct_x
        self.real_target_y = self.ideal_target_y + self.error_correct_y

        # 将终点加入避障路径
        self.current_path.append((self.real_target_x, self.real_target_y))

        # 实际距离坐标点的直线距离
        total_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)

        total_transit_dis = math.sqrt((self.my_car.x_current - self.current_path[self.plan_data.current_aimed_point_index][0]) ** 2 + (self.my_car.y_current - self.current_path[self.plan_data.current_aimed_point_index][1]) ** 2)
        # 依据到过渡点的距离计算里程计系数
        if total_transit_dis >= 300.0:
            self.my_car.alpha_x = 0.959375
        elif total_transit_dis >= 200.0:
            self.my_car.alpha_x = 0.968275
        elif total_transit_dis >= 100.0:
            self.my_car.alpha_x = 0.966933
        elif total_transit_dis >= 55.0:
            self.my_car.alpha_x = 0.975
        else:
            self.my_car.alpha_x = 1.02

        if total_transit_dis >= 220.0:
            self.my_car.alpha_y = 0.957758
        elif total_transit_dis >= 160.0:
            self.my_car.alpha_y = 0.96
        elif total_transit_dis >= 100.0:
            self.my_car.alpha_y = 0.962
        elif total_transit_dis >= 55.0:
            self.my_car.alpha_y = 0.9625
        else:
            self.my_car.alpha_y = 1.0

        # 计算减速距离（长距离或者搬运、扫描模式时减速距离为20，短距离时为0且短距离时速度恒定）
        if total_distance >= 55.0 or self.my_state.state == self.my_state.MOVE or self.my_state.state == self.my_state.SCAN:
            # 设置减速距离和加速时间阈值
            self.dec_distance = 25.0
            self.boost_time_threshold = 60
            if self.my_state.state == self.my_state.MOVE:
                self.v_max = self.move_v_max
            elif self.my_state.state == self.my_state.SCAN:
                self.v_max = self.scan_v_max
            else:
                self.v_max = self.long_v_max
            # 创建s型曲线减速速度表
            self.build_dec_speed_list(0)
            self.dis_flag = self.plan_data.LONG_DISTANCE
        # 中距离移动
        elif total_distance >= 10.0 and total_distance < 55.0:
            # 设置减速距离和加速时间阈值
            self.dec_distance = 8.0
            self.boost_time_threshold = 30
            self.v_max = self.short_v_max
            # 创建s型曲线减速速度表
            self.build_dec_speed_list(0)
            self.dis_flag = self.plan_data.MID_DISTANCE
        else:
            self.v_target = self.min_start_v
            self.dis_flag = self.plan_data.SHORT_DISTANCE

        self.if_pass_transit_point = False
        self.arrive_flag = False

    # 更新已完成和剩余距离并判断是否到达目标点
    def update_distance(self):
        if self.plan_data.current_aimed_point_index < len(self.current_path) - 1:
            self.current_rest_dis = math.sqrt((self.my_car.x_current - self.current_path[self.plan_data.current_aimed_point_index][0]) ** 2 + (self.my_car.y_current - self.current_path[self.plan_data.current_aimed_point_index][1]) ** 2)
            if self.current_rest_dis < 2.0:
                self.plan_data.current_aimed_point_index += 1
                total_transit_dis = math.sqrt((self.my_car.x_current - self.current_path[self.plan_data.current_aimed_point_index][0]) ** 2 + (self.my_car.y_current - self.current_path[self.plan_data.current_aimed_point_index][1]) ** 2)
                # 依据到过渡点的距离计算里程计系数
                if total_transit_dis >= 300.0:
                    self.my_car.alpha_x = 0.959375
                elif total_transit_dis >= 200.0:
                    self.my_car.alpha_x = 0.968275
                elif total_transit_dis >= 100.0:
                    self.my_car.alpha_x = 0.966933
                elif total_transit_dis >= 55.0:
                    self.my_car.alpha_x = 0.975
                else:
                    self.my_car.alpha_x = 1.02

                if total_transit_dis >= 220.0:
                    self.my_car.alpha_y = 0.957758
                elif total_transit_dis >= 160.0:
                    self.my_car.alpha_y = 0.96
                elif total_transit_dis >= 100.0:
                    self.my_car.alpha_y = 0.962
                elif total_transit_dis >= 55.0:
                    self.my_car.alpha_y = 0.9625
                else:
                    self.my_car.alpha_y = 1.0
        else:
            self.if_pass_transit_point = True

        self.rest_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)
        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold and abs(self.my_car.angle_pid.nowError) <= 0.5 and self.if_pass_transit_point == True:
            self.arrive_flag = True
            self.transition_flag = False
            # self.my_uart3.write("arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))
            self.finished_distance = 0.0
            self.rest_distance = 0.0
            self.dec_distance = 0.0

        if self.dis_flag != self.plan_data.SHORT_DISTANCE:
            # 每次更新距离后进行速度规划计算
            self.planning_speed()

    # 计算目标航向角
    def compute_target_yaw(self, target_x, target_y):
        # 只有在需要避障时开启航向角滤波以平滑过渡避障点
        if len(self.current_path) > 1:
            dx = self.sin_diff_fil.filtering(target_x - self.my_car.x_current)
            dy = self.cos_diff_fil.filtering(target_y - self.my_car.y_current)
        else:
            dx = target_x - self.my_car.x_current
            dy = target_y - self.my_car.y_current
        # 计算目标角度，单位：度（注意避免除以0）
        self.target_yaw = -math.atan2(-dx, dy) * 180.0 / self.MATH.PI        
            
    # 计算小车需要转向的角度（一般为0）
    def compute_turn_angle_target(self, turn_angle_target: float):
        self.turn_angle_target = turn_angle_target

    # 用于路径之间的过渡，保证小车平稳
    def path_transition(self):
        self.v_target = 0
        self.plan_data.time_counter += 1
        # 最终的过渡时间为 plan_point_transition_T * plan_calculate_T(单位：ms)
        if self.plan_data.time_counter >= self.plan_data.plan_point_transition_T:
            self.plan_data.time_counter = 0
            # self.my_uart3.write("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
            # 进行里程计的硬复位
            if self.if_finish_turn == True:
                self.my_car.x_current = self.ideal_target_x
                self.my_car.y_current = self.ideal_target_y	
            self.transition_flag = True

    # 停止小车运动
    def stop(self):
        self.v_target = 0
        self.target_yaw = 0.0

    # 按照传入路径及进行惯性导航
    # 如果传入的目标转角不为none，则进行转角规划，否则不进行转角规划（用于路径点之间的过渡）
    def navigate(self, path: list, target_turn_angle = None):
        # 先进行转角调整使得路径规划与导航更稳定
        if self.if_finish_turn == False and self.finish_navigate == False:
            if target_turn_angle is not None:
                self.v_target = 0
                self.turn_angle_target = target_turn_angle
                # 通过角度环限幅削弱转角调整的力度，帮助小车稳定完成转角调整
                self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.low_pwmout_limitmax
            else:
                self.turn_angle_target = self.my_car.now_yaw * 180.0 / self.MATH.PI
                self.if_finish_turn = True  # 如果没有目标转角，直接认为转角调整完成
                self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax
            # self.my_uart3.write(f"{abs(self.turn_angle_target - self.my_car.now_yaw * 180.0 / self.MATH.PI)}\n")
            # 在未完成转角调整时，持续进行转角调整
            diff = abs(self.turn_angle_target - self.my_car.now_yaw * 180 / self.MATH.PI)
            if diff > 180.0:
                diff = 360.0 - diff
            if diff <= 1.0:
                if self.transition_flag == False:
                    self.path_transition()
                else:
                    self.if_finish_turn = True
                    # 恢复正常的角度环限幅
                    self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax

        if self.if_set_path == False and self.finish_navigate == False and self.if_finish_turn == True:
            # 路径初始化
            self.path_points = path
            self.if_set_path = True
            # 设置第一个目标点
            self.set_target_point(self.path_points[0][0], self.path_points[0][1])
            self.compute_target_yaw(self.current_path[self.plan_data.current_aimed_point_index][0], self.current_path[self.plan_data.current_aimed_point_index][1])  
        # 规划好路径后再进行导航
        if self.if_set_path == True:
            # 判断是否还有未到达的目标点
            if self.plan_data.aimed_point_index < len(self.path_points):
                # 判断是否到达下一个目标点
                if self.arrive_flag == False:
                    self.update_distance()
                    if self.arrive_flag == True:
                        # 到达目标点后，更新目标点索引
                        self.plan_data.aimed_point_index += 1
                        # 进行路径过渡
                        self.path_transition()   
                    else:
                        # 计算目标航向角
                        self.compute_target_yaw(self.current_path[self.plan_data.current_aimed_point_index][0], self.current_path[self.plan_data.current_aimed_point_index][1])       
                else:
                    # 判断此时是否完成路径过渡
                    if self.transition_flag == False:
                        self.path_transition()
                    else:
                        # 如果还有下一个目标点，设置下一个目标点坐标
                        if self.plan_data.aimed_point_index < len(self.path_points):
                            self.set_target_point(self.path_points[self.plan_data.aimed_point_index][0], self.path_points[self.plan_data.aimed_point_index][1])
                            # 计算目标航向角
                            self.compute_target_yaw(self.current_path[self.plan_data.current_aimed_point_index][0], self.current_path[self.plan_data.current_aimed_point_index][1])
                        else:
                            self.stop()
            else:
                self.stop()
                # 测试
                # self.my_uart3.write("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
                self.my_car.x_current = self.ideal_target_x
                self.my_car.y_current = self.ideal_target_y
                self.if_finish_turn = False
                self.plan_data.aimed_point_index = 0
                self.dec_speed_index = 0
                self.path_points.clear()
                self.if_set_path = False
                self.transition_flag = False
                self.finish_navigate = True
                
 # 视觉伺服控制类(PD控制器)
class VisionManager:
    def __init__(self, flash_sys, beep, math, angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, tof, tof_distance_fil, car, protocol, order_manager, plan: Plan, state: StateMachine):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入数学常量对象
        self.MATH = math
        # 注入角度环pid对象
        self.angle_pid = angle_pid
        # 注入伺服PD控制器对象
        self.servo_pid = servo_pid
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入正弦滑动平均滤波器对象
        self.sin_servo_fil = sin_servo_fil
        # 注入余弦滑动平均滤波器对象
        self.cos_servo_fil = cos_servo_fil
        # 注入无线串口对象，用于调试
        self.my_uart3 = my_uart3
        # 注入TOF测距对象，用于测距
        self.my_tof = tof
        self.tof_distance = 0       # type: float # TOF测距值
        self.tof_buffer = []        # type: list  # TOF测距缓存列表
        self.tof_distance_fil = tof_distance_fil     # TOF测距滤波器对象
        # 注入小车姿态控制对象
        self.my_car = car
        # 注入通信协议对象
        self.my_art_protocol = protocol
        # 注入指令管理对象
        self.my_order_manager = order_manager
        # 注入路径规划对象
        self.my_plan = plan
        # 注入状态机对象
        self.my_state = state

        # 当前伺服的物品种类
        # 'T'为网球， 'S'为沙袋，'B'为玩具熊
        self.current_servo_object = ''
        # 当前伺服连续丢失物体的帧数
        self.servo_lost_count = 0
        # PD控制相关变量
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")  # type: float  # 视觉伺服控制距离阈值
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")  # type: float  # 视觉伺服控制距离阈值
        self.target_rel_speed_x = 0          # type: int   # 伺服控制目标x速度
        self.target_rel_speed_y = 0          # type: int   # 伺服控制目标y速度
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")  # type: int   # 视觉伺服控制最大速度
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")  # type: int   # 视觉伺服控制最小速度 
        self.target_point = []                      # type: list   # 目标点像素坐标
        self.target_rel_speed = 0                   # type: int     # 目标速度
        self.target_rel_yaw = 0.0                   # type: float   # 目标航向角
        self.target_rel_turn_angle = 0.0            # type: float   # 目标转角

        # 环绕控制相关变量
        self.orbit_radius = 0.0            # type: float   # 环绕半径
        self.orbit_speed = 0               # type: int     # 环绕速度
        self.orbit_yaw = 0.0               # type: float   # 环绕航向角
        self.orbit_turn_angle = 0.0        # type: float   # 环绕转角
        self.current_dis = 0.0             # type: float   # 当前距离
        self.target_angle = 0.0            # type: float   # 目标角度
        self.orbit_v = self.flash_sys.find_value("orbit_v")   # type: int   # 环绕速度
        self.object_radius = 0.0           # type: float   # 物体半径
        self.orbit_angle = 0.0             # type: float   # 环绕角度
        self.record_angle = 0.0            # type: float   # 记录的角度(记录小车的最初的角度)
        self.radius_T = self.flash_sys.find_value("radius_T")   # type: float   # 网球半径
        self.radius_S = self.flash_sys.find_value("radius_S")   # type: float   # 沙袋半径
        self.radius_B = self.flash_sys.find_value("radius_B")   # type: float   # 玩具熊半径
        self.direct = 0     # 0为顺时针，1为逆时针

        # 延时计数器
        self.counter = 0       # type: int     # 延时计数器
        # 目标角度缓冲区
        self.angle_buffer = []     # type: list    # 目标角度缓冲区
        # 微调阶段
        self.adjust_stage = 1      
        # 边线矫正时小车位置
        self.car_position = 0
        # 矫正次数
        self.calibrate_times = 0       # type: int     # 矫正次数
        # 标志位
        self.if_lost_object = False       # type: bool   # 是否丢失目标物体标志位
        self.if_send_servo_command = False   # type: bool   # 是否发送视觉伺服控制指令标志位
        self.finish_servo = False      # 是否完成视觉伺服控制标志位
        self.if_gain_dis = False       # type: bool   # 是否获取目标距离标志位
        self.finish_orbit = False      # type: bool   # 是否完成环绕控制标志位
        self.if_ready_calibrate = False       # type: bool  # 判断是否准备好进行校准标志位
        self.if_gain_calibrate_angle = False   # type: bool  # 判断是否获取校准角度标志位
        self.if_finish_calibrate = False       # type: bool  # 判断是否完成校准标志位

    # 计算目标航向角
    def compute_target_rel_yaw(self):
        # 计算目标角度，单位：度（注意避免除以0）
        self.target_rel_yaw = -math.atan2(-self.target_rel_speed_x, self.target_rel_speed_y) * 180.0 / self.MATH.PI + self.target_rel_turn_angle
        if self.target_rel_yaw > 180.0:
            self.target_rel_yaw -= 360.0
        elif self.target_rel_yaw < -180.0:
            self.target_rel_yaw += 360.0
    
    # 视觉伺服控制
    def visual_servo_control(self):
        # 单独测试该模式时需要解开这段注释
        '''
        # 通过标志位控制只向openart发送一次视觉伺服控制指令
        if self.if_send_servo_command == False:
            self.my_order_manager.mode_target()
            self.if_send_servo_command = True
            # 控制小车面向物体进行视觉伺服控制
            self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / self.MATH.PI
        else:
        '''
        if self.finish_servo == False:
            self.target_point = self.my_art_protocol.coordinate_receive()
            if self.target_point:
                self.servo_lost_count = 0
                self.servo_pid.compute_pid(self.target_point[0], self.target_point[1])
                self.target_rel_speed_x = self.servo_pid.pwm_output_x
                self.target_rel_speed_y = self.servo_pid.pwm_output_y

                if self.finish_servo == False:
                    # 判断是否完成视觉伺服控制
                    if abs(self.servo_pid.nowError_x) <= self.finish_threshold_x and abs(self.servo_pid.nowError_y) <= self.finish_threshold_y:
                        self.target_rel_speed = 0
                        self.target_rel_yaw = 0.0
                        self.my_order_manager.finish()
                        # 测试
                        self.my_beep.test()
                        self.finish_servo = True
                    else:
                        # 计算综合目标速度和航向角
                        # 滤波
                        self.target_rel_speed_x = self.sin_servo_fil.filtering(self.target_rel_speed_x)
                        self.target_rel_speed_y = self.cos_servo_fil.filtering(self.target_rel_speed_y)                                            
                        # 固定伺服速度
                        self.target_rel_speed = int(math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2))
                        self.compute_target_rel_yaw()
                        # 当横移角度过大时，速度折半
                        if self.target_rel_yaw > 45.0 or self.target_rel_yaw < -45.0:
                            self.target_rel_speed = int(self.target_rel_speed * 0.5)
                        self.target_rel_speed = max(self.min_rel_speed, min(self.target_rel_speed, self.max_rel_speed))
            else:
                self.servo_lost_count += 1
                # 连续丢失150帧物体坐标后（在1.5s内不再收到物体坐标信息），认为物体丢失，停止小车运动
                if self.servo_lost_count >= 150:
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.servo_lost_count = 0
                    self.if_lost_object = True


    # 环绕控制函数，传入环绕物体旋转的目标角度（单位：度），顺时针为正，逆时针为负
    def orbit_control(self, target_angle: float):
        if self.if_gain_dis == False:
             # 保持静止采集tof数据
            self.orbit_speed = 0
            if self.my_state.state == self.my_state.ORBIT:
                if len(self.tof_buffer) <= 35:          
                    # 获取TOF测距值，并添加到缓冲区
                    self.tof_buffer.append(self.tof_distance_fil.update(self.my_tof.get()))
                    # 测试
                    # self.my_uart3.write("tof_distance: {:<f}\n".format(self.tof_buffer[-1]))
                else:
                    # 计算最终的TOF测距值（去除前5个的平均值）
                    self.tof_distance = sum(self.tof_buffer[5:]) / len(self.tof_buffer[5:])
                    # 10.5为tof传感器到车身中心的距离，可以根据物体种类选择合适的旋转半径（object_radius）
                    self.orbit_radius = ((self.tof_distance - 36.0) / 10 + 10.5 + self.object_radius) / 5
                    self.record_angle = self.my_car.now_yaw * 180 / self.MATH.PI
                    self.target_angle = self.record_angle + target_angle
                    # 限制目标角度在-180到180度之间
                    if self.target_angle > 180.0:
                        self.target_angle -= 360.0
                    elif self.target_angle < -180.0:
                        self.target_angle += 360.0
                    # 确定旋转方向（顺时针还是逆时针）
                    if target_angle >= 0.0:
                        self.direct = 0
                    else:
                        self.direct = 1 
                    self.current_dis = 0.0
                    self.if_gain_dis = True
                    self.tof_buffer.clear()
                    # 测试
                    # self.my_beep.test()
                    # self.my_uart3.write("final_tof: {:<f}, orbit_radius: {:<f}\n".format(self.tof_distance, self.orbit_radius))
            elif self.my_state.state == self.my_state.REVERSE_ORBIT:
                    # 若此时模式为反向环绕，则环绕半径不变
                    self.record_angle = self.my_car.now_yaw * 180 / self.MATH.PI
                    self.target_angle = self.record_angle + target_angle
                    # 限制目标角度在-180到180度之间
                    if self.target_angle > 180.0:
                        self.target_angle -= 360.0
                    elif self.target_angle < -180.0:
                        self.target_angle += 360.0
                    # 确定旋转方向（顺时针还是逆时针）
                    if target_angle >= 0.0:
                        self.direct = 0
                    else:
                        self.direct = 1 
                    self.current_dis = 0.0
                    self.if_gain_dis = True
        else:
            if self.finish_orbit == False:
                # 更新当前小车的行驶距离
                self.current_dis += self.my_car.car_speed_x
                # 更新当前小车的目标转角及目标航向角
                self.orbit_turn_angle = -self.current_dis / self.orbit_radius * 180.0 / self.MATH.PI + self.record_angle
                if self.orbit_turn_angle >= 180.0:
                    self.orbit_turn_angle -= 360.0
                elif self.orbit_turn_angle <= -180.0:
                    self.orbit_turn_angle += 360.0
                    
                if self.direct == 0:
                    self.orbit_yaw = -90.0 + self.orbit_turn_angle
                elif self.direct == 1:
                    self.orbit_yaw = 90.0 + self.orbit_turn_angle
                    
                if self.orbit_yaw >= 180.0:
                    self.orbit_yaw -= 360.0
                elif self.orbit_yaw <= -180.0:
                    self.orbit_yaw += 360.0
                # 更新当前小车的速度
                self.orbit_speed = self.orbit_v
                # 判断是否完成环绕
                diff = abs(self.target_angle - self.my_car.now_yaw * 180 / self.MATH.PI)
                if diff > 180.0:
                    diff = 360.0 - diff
                if diff <= 0.5:	
                    self.orbit_speed = 0
                    self.orbit_turn_angle = self.my_car.now_yaw * 180 / self.MATH.PI
                    self.finish_orbit = True

    # apriltag辅助校准校准控制函数
    def apriltag_calibrate_control(self):
        """0代表下边线左侧,1代表下边线右侧, 2代表上边线左侧, 3代表上边线右侧"""
        if self.if_ready_calibrate == False:
            # 判断小车处于上下左右哪个边线，并微调小车位置使其更靠近边线（避免因惯性过大导致无法识别边线）
            # 进行两阶段的微调
            if self.adjust_stage == 1:
                if self.car_position == 1 or self.car_position == 3:
                    self.my_plan.navigate([[190.0, self.my_car.y_current]], self.my_car.now_yaw * 180.0 / self.MATH.PI)
                elif self.car_position == 0 or self.car_position == 2:
                    self.my_plan.navigate([[130.0, self.my_car.y_current]], self.my_car.now_yaw * 180.0 / self.MATH.PI)
                if self.my_plan.finish_navigate == True:
                    self.adjust_stage = 2
                    self.my_plan.finish_navigate = False
            elif self.adjust_stage == 2:
                if self.car_position == 1:
                    self.my_plan.navigate([[190.0, 0.0]], -90.0)
                elif self.car_position == 0:
                    self.my_plan.navigate([[130.0, 0.0]], 90.0)
                elif self.car_position == 3:
                    self.my_plan.navigate([[190.0, 240.0]], -90.0)
                elif self.car_position == 2:
                    self.my_plan.navigate([[130.0, 240.0]], 90.0)
                if self.my_plan.finish_navigate == True:
                    # 伺服apriltag时固定目标点坐标（单位：像素），并且固定目标转角为0（即小车面向apriltag）
                    self.servo_pid.target_y = 10.0
                    self.counter = 0
                    self.calibrate_times = 0
                    # 重置阶段标志
                    self.adjust_stage = 1
                    self.if_ready_calibrate = True
                    self.my_plan.finish_navigate = False
                    self.target_rel_turn_angle = self.my_plan.turn_angle_target
                    # 测试
                    self.my_beep.test()
                    self.my_order_manager.mode_apriltag()
        else:
            if self.calibrate_times == 1:
                if self.counter != -1:
                    self.counter += 1
                if self.counter >= 50:
                    self.counter = -1
                    self.if_gain_calibrate_angle = False
                    # 测试
                    self.my_beep.test()
                elif self.counter != -1:
                    return
            target_point = self.my_art_protocol.apriltag_receive()
            if target_point:
                self.servo_lost_count = 0
                if self.if_gain_calibrate_angle == False:
                    # 计算目标转角
                    now_yaw = self.my_car.now_yaw * 180.0 / self.MATH.PI
                    if self.car_position == 0 or self.car_position == 2:
                        self.target_rel_turn_angle = now_yaw + target_point[2]
                    elif self.car_position == 1 or self.car_position == 3:
                        self.target_rel_turn_angle = now_yaw - target_point[2]
                    self.if_gain_calibrate_angle = True

                self.servo_pid.compute_pid(target_point[0], target_point[1])
                self.target_rel_speed_x = self.servo_pid.pwm_output_x
                self.target_rel_speed_y = self.servo_pid.pwm_output_y
                
                if self.if_finish_calibrate == False:
                    # 判断是否完成视觉伺服控制
                    diff = abs(self.target_rel_turn_angle - self.my_car.now_yaw * 180.0 / self.MATH.PI)
                    if diff > 180.0:
                        diff = 360.0 - diff
                    if ((abs(self.servo_pid.nowError_x) <= self.finish_threshold_x and abs(self.servo_pid.nowError_y) <= self.finish_threshold_y) or self.calibrate_times == 1) and diff <= 0.5:
                        self.target_rel_speed = 0
                        self.target_rel_yaw = 0.0
                        # 测试
                        self.my_beep.test()
                        self.calibrate_times += 1
                        # 完成两次矫正才算结束
                        if self.calibrate_times >= 2:
                            self.calibrate_times = 0
                            self.counter = 0
                            # 里程计和姿态角硬复位
                            if self.car_position == 0:
                                self.my_car.now_yaw = self.MATH.PI / 2
                                self.my_car.x_current = 140.0
                                self.my_car.y_current = 0.0
                            elif self.car_position == 1:
                                self.my_car.now_yaw = -self.MATH.PI / 2
                                self.my_car.x_current = 180.0
                                self.my_car.y_current = 0.0
                            elif self.car_position == 2:
                                self.my_car.now_yaw = self.MATH.PI / 2
                                self.my_car.x_current = 140.0
                                self.my_car.y_current = 240.0
                            elif self.car_position == 3:
                                self.my_car.now_yaw = -self.MATH.PI / 2
                                self.my_car.x_current = 180.0
                                self.my_car.y_current = 240.0
                            # 在切换模式前保持当前转角
                            self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / self.MATH.PI
                            self.my_order_manager.finish()
                            self.if_finish_calibrate = True
                    else:
                        # 计算综合目标速度和航向角
                        # 滤波
                        self.target_rel_speed_x = self.sin_servo_fil.filtering(self.target_rel_speed_x)
                        self.target_rel_speed_y = self.cos_servo_fil.filtering(self.target_rel_speed_y)                                            
                        self.compute_target_rel_yaw()

                        if self.calibrate_times == 1:
                            self.target_rel_speed = 0
                        else:
                            # 计算伺服速度
                            self.target_rel_speed = int(math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2))
                            # 当横移角度过大时，速度折半
                            if self.target_rel_yaw > 45.0 or self.target_rel_yaw < -45.0:
                                self.target_rel_speed = int(self.target_rel_speed * 0.5)
                            self.target_rel_speed = max(self.min_rel_speed, min(self.target_rel_speed, self.max_rel_speed))
            else:
                self.servo_lost_count += 1
                # 连续丢失100帧apriltag坐标后（在1s内不再收到物体坐标信息），认为apriltag丢失，停止小车运动
                if self.servo_lost_count >= 100:
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.servo_lost_count = 0
                    self.if_lost_object = True