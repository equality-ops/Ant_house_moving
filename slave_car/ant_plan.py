import math

# 状态机制
class StateMachine:
    def __init__(self):
        self.NAVIGATE = 1    # 导航状态
        self.SERVO = 2       # 视觉伺服状态
        self.ORBIT = 3       # 环绕状态
        self.MOVE = 4        # 搬运状态
        self.CALIBRATE = 5   # 校准状态
        self.RETURN = 6		 # 返回状态
        self.STOP = 7        # 停止状态
        self.state = self.NAVIGATE  # 初始状态为导航状态


# 路径和速度规划相关常量
class Plan_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys

        # 地图固定点坐标
        self.fixed_point = [[0.0, 0.0], [0.0, 150.0], [150.0, 0.0], [150.0, 150.0]]  # type: list
        # 路径1
        self.path_1 = [[[8.9, 8.0]]]     # type: list
        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 坐标误差修正量
        self.error_correct_x_50_1 = self.flash_sys.find_value("error_correct_x_50_1") # type: float
        self.error_correct_y_50_1 = self.flash_sys.find_value("error_correct_y_50_1")  # type: float
        self.error_correct_x_50_2 = self.flash_sys.find_value("error_correct_x_50_2") # type: float
        self.error_correct_y_50_2 = self.flash_sys.find_value("error_correct_y_50_2")  # type: float
        self.error_correct_x_50_3 = self.flash_sys.find_value("error_correct_x_50_3") # type: float
        self.error_correct_y_50_3 = self.flash_sys.find_value("error_correct_y_50_3")  # type: float
        self.error_correct_x_50_4 = self.flash_sys.find_value("error_correct_x_50_4") # type: float  
        self.error_correct_y_50_4 = self.flash_sys.find_value("error_correct_y_50_4")  # type: float
        self.error_correct_x_50_5 = self.flash_sys.find_value("error_correct_x_50_5") # type: float
        self.error_correct_y_50_5 = self.flash_sys.find_value("error_correct_y_50_5")  # type: float
        self.error_correct_x_50_6 = self.flash_sys.find_value("error_correct_x_50_6") # type: float
        self.error_correct_y_50_6 = self.flash_sys.find_value("error_correct_y_50_6")  # type: float
        self.error_correct_x_50_7 = self.flash_sys.find_value("error_correct_x_50_7") # type: float
        self.error_correct_y_50_7 = self.flash_sys.find_value("error_correct_y_50_7")  # type: float
        self.error_correct_x_50_8 = self.flash_sys.find_value("error_correct_x_50_8") # type: float
        self.error_correct_y_50_8 = self.flash_sys.find_value("error_correct_y_50_8")  # type: float
        # 时间计数器
        self.time_counter = 0          # type: int
        # 路径点切换时间阈值（用于过渡）
        self.plan_point_transition_T = self.flash_sys.find_value("plan_point_transition_T")
        # 上下左右边线
        self.BOUNDARY_UP = 1
        self.BOUNDARY_DOWN = 2
        self.BOUNDARY_LEFT = 3
        self.BOUNDARY_RIGHT = 4
        # 长短距离标志
        self.LONG_DISTANCE = 1
        self.SHORT_DISTANCE = 2

class Plan:
    def __init__(self, flash_sys, plan_data: Plan_data, math, car, order_manager, my_uart3, beep, art_protocol):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入数学常量对象
        self.MATH = math
        # 注入小车位置对象
        self.my_car = car
        # 注入无线通信对象
        self.my_uart3 = my_uart3
        # 注入指令管理对象
        self.my_order_manager = order_manager
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入openart串口解析对象
        self.my_art_protocol = art_protocol

        # 速度规划相关常量
        self.min_start_v = self.flash_sys.find_value("min_start_v")           # type: int  # 最小制动速度
        self.dead_zone_v = self.flash_sys.find_value("dead_zone_v")         # type: int  # 死区启动速度
        self.long_v_max = self.flash_sys.find_value("long_v_max")           # type: int  # 长距离时的最大速度
        self.short_v_max = self.flash_sys.find_value("short_v_max")          # type: int  # 短距离时的最大速度
        self.BOOST = 1                  # type: int  # 死区启动标志位
        self.TRANSIT = 2                # type: int  # 过渡阶段标志位
        self.DEC = 3                    # type: int  # 减速阶段标志位
        self.STOP = 4                   # type: int  # 停止标志位
        self.dec_ratio = self.flash_sys.find_value("dec_ratio")	# type: float  # 减速段占据的比例
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
        self.boost_time_threshold = self.flash_sys.find_value("boost_time_threshold")  # type: int  # 死区启动时间阈值
        self.dec_speed_index = 0        # type: int   # 减速速度表索引
        # 路径规划相关变量
        self.last_target_x = 0.0         # type: float
        self.last_target_y = 0.0         # type: float
        self.last_target_yaw = 0.0       # type: float
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
        self.rest_distance = 0.0        # type: float
        # 目标路径
        self.path_points = []      # type: list
        # 边线矫正时小车位置
        self.car_position = None
        # 距离长短标志位
        self.dis_flag = None
        # 标志位
        self.arrive_flag = False        # type: bool  # 判断是否到达目标点标志位
        self.transition_flag = True     # type: bool  # 判断是否过渡完成标志位
        self.if_set_path = False        # type: bool  # 判断是否设置路径标志位
        self.finish_navigate = False    # type: bool  # 判断是否完成导航标志位
        self.if_ready_calibrate = False       # type: bool  # 判断是否准备好进行校准标志位
        self.if_gain_calibrate_angle = True   # type: bool  # 判断是否获取校准角度标志位
        self.if_finish_calibrate = True       # type: bool  # 判断是否完成校准标志位

    def _ease_out_quad(self, t):
        """二次缓出曲线，用于快速启动"""
        return -t * (t - 2)
    
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
                    self.v_target = self.min_start_v + int(self._ease_out_quad(self.elapsed_time / self.boost_time_threshold) * (self.v_max - self.min_start_v))
                else:
                    self.v_target = self.v_max
                    self.stage = self.TRANSIT
                    self.elapsed_time = 0
                    self.my_uart3.write("boost_finish\n")
            elif self.stage == self.TRANSIT:
                self.v_target = self.v_max
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


    # 设置目标点坐标
    def set_target_point(self, x: float, y: float):
        self.last_target_x = self.real_target_x
        self.last_target_y = self.real_target_y
        self.last_target_yaw = self.target_yaw
        # 理想条件下的目标坐标
        self.ideal_target_x = x
        self.ideal_target_y = y
        # 计算大致航向
        dx = self.ideal_target_x - self.my_car.x_current
        dy = self.ideal_target_y - self.my_car.y_current
        # 计算目标角度，单位：度（注意避免除以0）
        if dy == 0.0:
            if dx > 0.0:
                blurry_yaw = 90.0
            elif dx < 0.0:
                blurry_yaw = -90.0
        elif dx == 0.0:
            if dy > 0.0:
                blurry_yaw = 0.0
            elif dy < 0.0:
                blurry_yaw = 180.0
        else:  
            if dx > 0.0 and dy < 0.0:
                blurry_yaw = math.atan(dx / dy) * 180.0 / self.MATH.PI + 180.0
            elif dx < 0.0 and dy < 0.0:
                blurry_yaw = math.atan(dx / dy) * 180.0 / self.MATH.PI - 180.0
            else:
                blurry_yaw = math.atan(dx / dy) * 180.0 / self.MATH.PI
                
        # 根据大致航向角选择合适的坐标修正量（解决因惯性造成的打滑问题）
        if blurry_yaw >= -30.0 and blurry_yaw < 30.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_1
            self.error_correct_y = self.plan_data.error_correct_y_50_1
        elif blurry_yaw >= 30.0 and blurry_yaw < 60.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_2
            self.error_correct_y = self.plan_data.error_correct_y_50_2
        elif blurry_yaw >= 60.0 and blurry_yaw < 120.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_3
            self.error_correct_y = self.plan_data.error_correct_y_50_3
        elif blurry_yaw >= 120.0 and blurry_yaw < 150.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_4
            self.error_correct_y = self.plan_data.error_correct_y_50_4
        elif blurry_yaw >= 150.0 and blurry_yaw <= 180.0 or blurry_yaw >= -180.0 and blurry_yaw < -150.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_5
            self.error_correct_y = self.plan_data.error_correct_y_50_5
        elif blurry_yaw >= -150.0 and blurry_yaw < -120.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_6
            self.error_correct_y = self.plan_data.error_correct_y_50_6
        elif blurry_yaw >= -120.0 and blurry_yaw < -60.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_7
            self.error_correct_y = self.plan_data.error_correct_y_50_7
        elif blurry_yaw >= -60.0 and blurry_yaw < -30.0:
            self.error_correct_x = self.plan_data.error_correct_x_50_8
            self.error_correct_y = self.plan_data.error_correct_y_50_8
        
        # 实际条件下的目标坐标
        self.real_target_x = self.ideal_target_x + self.error_correct_x
        self.real_target_y = self.ideal_target_y + self.error_correct_y
        # 实际距离坐标点的总距离
        self.total_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)
        # 根据总距离设置最大速度
        if self.total_distance >= 30.0:
           self.v_max = self.long_v_max
        else:
          self.v_max = self.short_v_max
        # 依据总距离计算里程计系数
        if self.total_distance >= 150.0:
            self.my_car.alpha_x = 0.967048
        elif self.total_distance >= 100.0:
            self.my_car.alpha_x = 0.954026
        elif self.total_distance >= 45.0:
            self.my_car.alpha_x = 0.944249
        else:
            self.my_car.alpha_x = 0.94

        if self.total_distance >= 280.0:
            self.my_car.alpha_y = 0.950677
        elif self.total_distance >= 230.0:
            self.my_car.alpha_y = 0.951949
        elif self.total_distance >= 130.0:
            self.my_car.alpha_y = 0.946843
        elif self.total_distance >= 45.0:
            self.my_car.alpha_y = 0.937625
        else:
            self.my_car.alpha_y = 0.933

        # 计算减速距离（长距离时减速距离为20，短距离时为0且短距离时速度恒定）
        if self.total_distance >= 45.0:
            self.dec_distance = 20.0
            self.build_dec_speed_list(0)
            self.dis_flag = self.plan_data.LONG_DISTANCE
        else:
            self.v_target = 60
            self.dis_flag = self.plan_data.SHORT_DISTANCE
        self.arrive_flag = False
        # 测试
        # self.v_target = 320

    # 更新已完成和剩余距离并判断是否到达目标点
    def update_distance(self):
        # 暂不需要计算已完成距离，直接用总距离和剩余距离判断是否到达目标点
        # self.finished_distance = math.sqrt((self.my_car.x_current - self.last_target_x) ** 2 + (self.my_car.y_current - self.last_target_y) ** 2)
        self.rest_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)
        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold and abs(self.my_car.angle_pid.nowError) <= 1.0:
            self.arrive_flag = True
            self.transition_flag = False
            self.my_uart3.write("arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))
            self.finished_distance = 0.0
            self.rest_distance = 0.0
            self.dec_distance = 0.0
        # 每次更新距离后进行速度规划计算
        # 测试
        if self.dis_flag == self.plan_data.LONG_DISTANCE:
            self.planning_speed()

    # 计算目标航向角
    def compute_target_yaw(self):
        dx = self.real_target_x - self.my_car.x_current
        dy = self.real_target_y - self.my_car.y_current
        # 计算目标角度，单位：度（注意避免除以0）
        if dy == 0.0:
            if dx > 0.0:
                self.target_yaw = 90.0
            elif dx < 0.0:
                self.target_yaw = -90.0
        elif dx == 0.0:
            if dy > 0.0:
                self.target_yaw = 0.0
            elif dy < 0.0:
                self.target_yaw = 180.0
        else:  
            if dx > 0.0 and dy < 0.0:
                self.target_yaw = math.atan(dx / dy) * 180.0 / self.MATH.PI + 180.0
            elif dx < 0.0 and dy < 0.0:
                self.target_yaw = math.atan(dx / dy) * 180.0 / self.MATH.PI - 180.0
            else:
                self.target_yaw = math.atan(dx / dy) * 180.0 / self.MATH.PI
                
    # 计算小车需要转向的角度（一般为0）
    def compute_turn_angle_target(self, turn_angle_target: float):
        self.turn_angle_target = turn_angle_target

    # 更新战术矢量，master_pos为主车位置（如果是从车），obstacles为传感器探测到的障碍物坐标列表（如果有）
    # 小车的航向角会受到目标点引力和障碍物斥力的影响
    def update_tactical_vector(self, master_pos=None, obstacles=[]):
        """
        master_pos: (x, y) 如果是从车，引力来源于主车后方的跟随点
        obstacles: [(x, y, save_dist), ...] 传感器探测到的障碍物坐标及安全距离
        """
        # 1. 计算引力向量 (dx, dy)
        if master_pos:
            # 协同逻辑：跟随点定在主车后方 30cm
            target_x, target_y = master_pos[0] - 30, master_pos[1] 
        else:
            target_x, target_y = self.real_target_x, self.real_target_y
    
        f_att_x = target_x - self.my_car.x_current
        f_att_y = target_y - self.my_car.y_current

        # 2. 计算斥力向量 (避障逻辑)
        f_rep_x, f_rep_y = 0.0, 0.0
    
        for ob_x, ob_y, safe_dist in obstacles:
            dist = math.sqrt((self.my_car.x_current - ob_x)**2 + (self.my_car.y_current - ob_y)**2)
            if dist < safe_dist:
                # 距离越近，排斥力指数级增长，500.0为斥力强度调节系数（需要根据实际情况调整）
                force = 500.0 * (1.0/dist - 1.0/safe_dist)
                f_rep_x += force * (self.my_car.x_current - ob_x) / dist
                f_rep_y += force * (self.my_car.y_current - ob_y) / dist

                # --- 新增的侧向拨力 (切向力) ---
                # 此时切向力向右侧拨动，强度与斥力成正比，方向垂直于法向斥力方向
                # 强度可以稍微小一点，比如是正向斥力的 0.2 倍，该系数需要根据实际调整
                f_tan_x = 0.2 * force * ((self.my_car.y_current - ob_y) / dist)   # 利用 (dy, -dx) 旋转逻辑
                f_tan_y = 0.2 * force * (-(self.my_car.x_current - ob_x) / dist)

                # 最终斥力 = 法向斥力 + 切向拨力
                total_f_rep_x += (f_rep_x + f_tan_x)
                total_f_rep_y += (f_rep_y + f_tan_y)

        # 3. 合成最终矢量
        total_dx = f_att_x + total_f_rep_x
        total_dy = f_att_y + total_f_rep_y

        # # 更新 target_yaw 引导转向，单位：度（注意避免除以0）
        if total_dy == 0.0:
            if total_dx > 0.0:
                self.target_yaw = 90.0
            elif total_dx < 0.0:
                self.target_yaw = -90.0
        elif total_dx == 0.0:
            if total_dy > 0.0:
                self.target_yaw = 0.0
            elif total_dy < 0.0:
                self.target_yaw = 180.0
        else:  
            if total_dx > 0.0 and total_dy < 0.0:
                self.target_yaw = math.atan(total_dx / total_dy) * 180.0 / self.MATH.PI + 180.0
            elif total_dx < 0.0 and total_dy < 0.0:
                self.target_yaw = math.atan(total_dx / total_dy) * 180.0 / self.MATH.PI - 180.0
            else:
                self.target_yaw = math.atan(total_dx / total_dy) * 180.0 / self.MATH.PI

            # 更新 rest_distance 引导速度规划
            # 这样你的 S 曲线减速逻辑 (planning_speed) 依然能完美生效
            self.rest_distance = math.sqrt(total_dx**2 + total_dy**2)

        # 当剩余距离小于阈值并且完成目标转角时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold and abs(self.my_car.angle_pid.nowError) <= 1.0:
            self.arrive_flag = True
            self.transition_flag = False
            # 测试
            self.my_uart3.write("arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))
            self.finished_distance = 0.0
            self.rest_distance = 0.0
            self.dec_distance = 0.0
        # 每次更新距离后进行速度规划计算
        # 测试
        if self.dis_flag == self.plan_data.LONG_DISTANCE:
            self.planning_speed()

    # 用于路径之间的过渡，保证小车平稳
    def path_transition(self):
        self.v_target = 0
        self.plan_data.time_counter += 1
        # 最终的过渡时间为 plan_point_transition_T * plan_calculate_T(单位：ms)
        if self.plan_data.time_counter >= self.plan_data.plan_point_transition_T:
            self.plan_data.time_counter = 0
            self.my_uart3.write("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
            self.my_car.x_current = self.ideal_target_x
            self.my_car.y_current = self.ideal_target_y	
            self.transition_flag = True

    # 停止小车运动
    def stop(self):
        self.v_target = 0
        self.target_yaw = 0.0
        # 保持当前小车转角停下
        self.turn_angle_target = self.my_car.now_yaw * 180.0 / self.MATH.PI

    # 按照传入路径及进行惯性导航
    # 如果传入的目标转角不为none，则进行转角规划，否则不进行转角规划（用于路径点之间的过渡）
    def navigate(self, path: list, target_turn_angle = None):
        if self.if_set_path == False and self.finish_navigate == False:
            # 路径初始化
            self.path_points = path
            self.if_set_path = True
            self.plan_data.aimed_point_index = 0
            # 设置第一个目标点
            self.set_target_point(self.path_points[0][0], self.path_points[0][1])
            self.compute_target_yaw()
            if target_turn_angle is not None:
                self.compute_turn_angle_target(target_turn_angle)
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
                    self.compute_target_yaw()
            else:
                # 判断此时是否完成路径过渡
                if self.transition_flag == False:
                    self.path_transition()
                else:
                    # 如果还有下一个目标点，设置下一个目标点坐标
                    if self.plan_data.aimed_point_index < len(self.path_points):
                        self.set_target_point(self.path_points[self.plan_data.aimed_point_index][0], self.path_points[self.plan_data.aimed_point_index][1])
                        # 计算目标航向角
                        self.compute_target_yaw()
                    else:
                        self.stop()
        else:
            self.stop()
            # 测试
            # self.my_uart3.write("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
            self.my_car.x_current = self.ideal_target_x
            self.my_car.y_current = self.ideal_target_y
            self.dec_speed_index = 0
            self.path_points.clear()
            self.if_set_path = False
            self.finish_navigate = True
    
    # 从车战术导航
    def slave_tactical_navigate(self, master_pos, obstacles = [], target_turn_angle = None):
        if self.if_set_path == False and self.finish_navigate == False:
            # 路径初始化
            self.path_points = path
            self.if_set_path = True
            self.plan_data.aimed_point_index = 0
            # 设置第一个目标点
            self.set_target_point(self.path_points[0][0], self.path_points[0][1])
            if target_turn_angle is not None:
                self.compute_turn_angle_target(target_turn_angle)
        # 判断是否还有未到达的目标点
        if self.plan_data.aimed_point_index < len(self.path_points):
            # 判断是否到达下一个目标点
            if self.arrive_flag == False:
                self.update_tactical_vector(master_pos=None, obstacles=obstacles)
                if self.arrive_flag == True:
                    # 到达目标点后，更新目标点索引
                    self.plan_data.aimed_point_index += 1
                    # 进行路径过渡
                    self.path_transition()   
            else:
                # 判断此时是否完成路径过渡
                if self.transition_flag == False:
                    self.path_transition()
                else:
                    # 如果还有下一个目标点，设置下一个目标点坐标
                    if self.plan_data.aimed_point_index < len(self.path_points):
                        self.set_target_point(self.path_points[self.plan_data.aimed_point_index][0], self.path_points[self.plan_data.aimed_point_index][1])
                        # 计算目标航向角
                        self.compute_target_yaw()
                    else:
                        self.stop()
        else:
            self.stop()
            # 测试
            # self.my_uart3.write("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
            self.my_car.x_current = self.ideal_target_x
            self.my_car.y_current = self.ideal_target_y
            self.dec_speed_index = 0
            self.path_points.clear()
            self.if_set_path = False
            self.finish_navigate = True

        # 主车战术导航
    def main_tactical_navigate(self, path = [], obstacles = [], target_turn_angle = None):
        if self.if_set_path == False and self.finish_navigate == False:
            # 路径初始化
            self.path_points = path
            self.if_set_path = True
            self.plan_data.aimed_point_index = 0
            # 设置第一个目标点
            self.set_target_point(self.path_points[0][0], self.path_points[0][1])
            if target_turn_angle is not None:
                self.compute_turn_angle_target(target_turn_angle)
        # 判断是否还有未到达的目标点
        if self.plan_data.aimed_point_index < len(self.path_points):
            # 判断是否到达下一个目标点
            if self.arrive_flag == False:
                self.update_tactical_vector(master_pos=None, obstacles=obstacles)
                if self.arrive_flag == True:
                    # 到达目标点后，更新目标点索引
                    self.plan_data.aimed_point_index += 1
                    # 进行路径过渡
                    self.path_transition()   
            else:
                # 判断此时是否完成路径过渡
                if self.transition_flag == False:
                    self.path_transition()
                else:
                    # 如果还有下一个目标点，设置下一个目标点坐标
                    if self.plan_data.aimed_point_index < len(self.path_points):
                        self.set_target_point(self.path_points[self.plan_data.aimed_point_index][0], self.path_points[self.plan_data.aimed_point_index][1])
                        # 计算目标航向角
                        self.compute_target_yaw()
                    else:
                        self.stop()
        else:
            self.stop()
            # 测试
            # self.my_uart3.write("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
            self.my_car.x_current = self.ideal_target_x
            self.my_car.y_current = self.ideal_target_y
            self.dec_speed_index = 0
            self.path_points.clear()
            self.if_set_path = False
            self.finish_navigate = True


    def boundary_calibrate_control(self):
        if self.if_ready_calibrate == False:
            # 判断小车处于上下左右哪个边线，并微调小车位置使其更靠近边线（避免因惯性过大导致无法识别边线）
            now_yaw = self.my_car.now_yaw * 180.0 / self.MATH.PI
            if self.my_car.x_current <= 10.0 and self.my_car.y_current >= 30.0 and self.my_car.y_current <= 270.0:
                if now_yaw <= -90.0 or now_yaw > 90.0:
                    self.navigate([[-10.0, self.my_car.y_current + 5.0]], 180.0)
                else:
                    self.navigate([[-10.0, self.my_car.y_current - 5.0]], 0.0)
                self.car_position = self.plan_data.BOUNDARY_LEFT
                # 重置标志位，准备进行边线校准
                self.if_finish_calibrate = False
                self.if_gain_calibrate_angle = False
            elif self.my_car.x_current >= 290.0 and self.my_car.y_current >= 30.0 and self.my_car.y_current <= 270.0:
                if now_yaw <= -90.0 or now_yaw > 90.0:
                    self.navigate([[330.0, self.my_car.y_current + 5.0]], 180.0)
                else:
                    self.navigate([[330.0, self.my_car.y_current - 5.0]], 0.0)
                self.car_position = self.plan_data.BOUNDARY_RIGHT
                # 重置标志位，准备进行边线校准
                self.if_finish_calibrate = False
                self.if_gain_calibrate_angle = False
            elif self.my_car.y_current <= 10.0 and self.my_car.x_current >= 30.0 and self.my_car.x_current <= 270.0:
                if now_yaw <= 0.0 and now_yaw > -180.0:
                    self.navigate([[self.my_car.x_current + 5.0, -10.0]], -90.0)
                else:
                    self.navigate([[self.my_car.x_current - 5.0, -10.0]], 90.0)
                self.car_position = self.plan_data.BOUNDARY_UP
                # 重置标志位，准备进行边线校准
                self.if_finish_calibrate = False
                self.if_gain_calibrate_angle = False
            elif self.my_car.y_current >= 290.0 and self.my_car.x_current >= 30.0 and self.my_car.x_current <= 270.0:
                if now_yaw <= 0.0 and now_yaw > -180.0:
                    self.navigate([[self.my_car.x_current + 5.0, 250.0]], -90.0)
                else:
                    self.navigate([[self.my_car.x_current - 5.0, 250.0]], 90.0)
                self.car_position = self.plan_data.BOUNDARY_DOWN
                # 重置标志位，准备进行边线校准
                self.if_finish_calibrate = False
                self.if_gain_calibrate_angle = False
            else:
                self.car_position = None
                self.if_finish_calibrate = True
                self.if_gain_calibrate_angle = True

            if self.finish_navigate == True:
                self.if_ready_calibrate = True
                self.finish_navigate = False
                # 测试
                self.my_beep.test()
                # 判断是进行左右边线矫正还是上下边沿矫正
                if self.car_position == self.plan_data.BOUNDARY_LEFT or self.car_position == self.plan_data.BOUNDARY_RIGHT:
                # 向openart发送左右边线校准指令获取校准角度
                    self.my_order_manager.mode_boundary_lf()
                elif self.car_position == self.plan_data.BOUNDARY_UP or self.car_position == self.plan_data.BOUNDARY_DOWN:
                    # 向openart发送上下边线校准指令获取校准角度
                    self.my_order_manager.mode_boundary_ud()
        else:
            if self.car_position == self.plan_data.BOUNDARY_LEFT:
                self.navigate([[10.0, self.my_car.y_current]])
            elif self.car_position == self.plan_data.BOUNDARY_RIGHT:
                self.navigate([[310.0, self.my_car.y_current]])
            elif self.car_position == self.plan_data.BOUNDARY_UP:
                self.navigate([[self.my_car.x_current, 10.0]])
            elif self.car_position == self.plan_data.BOUNDARY_DOWN:
                self.navigate([[self.my_car.x_current, 230.0]])
            if self.finish_navigate == True:
                # 此时仍未获得角度信息，直接退出该模式
                self.finish_navigate = False
                self.if_gain_calibrate_angle = True
                self.if_finish_calibrate = True
                # 向openart发送停止校准指令
                self.my_order_manager.finish()
                return 
            
            # 判断是否获取到校准角度
            if self.if_gain_calibrate_angle == False:
                self.my_art_protocol.angle_receive()
                if len(self.my_art_protocol.angle_list) >= 3:
                    # 进行边线校准处理
                    self.calibrate_angle = sum(self.my_art_protocol.angle_list) / len(self.my_art_protocol.angle_list)
                    self.turn_angle_target += self.calibrate_angle * 2 / 3
                    # 进行里程计矫正处理
                    if self.my_car.x_current <= 50.0 and self.car_position == self.plan_data.BOUNDARY_LEFT:
                        self.my_car.x_current = 0.0
                    elif self.my_car.x_current >= 270.0 and self.car_position == self.plan_data.BOUNDARY_RIGHT:
                        self.my_car.x_current = 300.0
                    elif self.my_car.y_current <= 50.0 and self.car_position == self.plan_data.BOUNDARY_UP:
                        self.my_car.y_current = 0.0
                    elif self.my_car.y_current >= 190.0 and self.car_position == self.plan_data.BOUNDARY_DOWN:
                        self.my_car.y_current = 240.0
                    self.my_order_manager.finish()
                    self.if_gain_calibrate_angle = True 
                    # 若获得角度则跳过定位过渡阶段直接进行转角调整
                    self.arrive_flag = True
                    self.my_car.x_current = self.ideal_target_x
                    self.my_car.y_current = self.ideal_target_y
                    self.dec_speed_index = 0
                    self.path_points.clear()
                    self.if_set_path = False

                    # 测试
                    self.my_beep.test()
                    for i in range(0, len(self.my_art_protocol.angle_list)):
                        self.my_uart3.write(f"{self.my_art_protocol.angle_list[i]}\n")
                    # self.my_uart3.write(f"average_angle: {self.turn_angle_target}\n")

                    self.my_art_protocol.angle_list.clear()

            if self.if_finish_calibrate == False:
                # 判断是否完成校准（校准误差不超过2度）
                if self.if_gain_calibrate_angle == True and abs(self.my_car.angle_pid.nowError) <= 1.0:
                    self.if_finish_calibrate = True
                    # 向openart发送停止校准指令
                    self.my_order_manager.finish()
                    # 根据小车位置重置小车角度及目标转角
                    now_yaw = self.my_car.now_yaw * 180.0 / self.MATH.PI
                    if now_yaw >= -45.0 and now_yaw < 45.0:
                        self.my_car.now_yaw = 0.0
                        self.turn_angle_target = 0.0
                    elif now_yaw >= 45.0 and now_yaw < 135.0:
                        self.my_car.now_yaw = self.MATH.PI / 2
                        self.turn_angle_target = 90.0
                    elif now_yaw >= -135.0 and now_yaw < -45.0:
                        self.my_car.now_yaw = -self.MATH.PI / 2
                        self.turn_angle_target = -90.0
                    else:
                        self.my_car.now_yaw = self.MATH.PI
                        self.turn_angle_target = 180.0
                    # 测试
                    self.my_beep.test()
                    


 # 视觉伺服控制类(PD控制器)
class VisionManager:
    def __init__(self, flash_sys, beep, math, servo_pid, servo_yaw_fil, my_uart3, tof, tof_distance_fil, car, protocol, order_manager):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入数学常量对象
        self.MATH = math
        # 注入伺服PD控制器对象
        self.servo_pid = servo_pid
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入航向角滤波器对象
        self.servo_yaw_fil = servo_yaw_fil
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

        # PD控制相关变量
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")  # type: float  # 视觉伺服控制距离阈值
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")  # type: float  # 视觉伺服控制距离阈值
        self.target_rel_speed_x = 0          # type: int   # 伺服控制目标x速度
        self.target_rel_speed_y = 0          # type: int   # 伺服控制目标y速度
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")   # type: int   # 最小视觉伺服速度
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")   # type: int   # 最小视觉伺服速度
        self.target_point = []                      # type: list   # 目标点像素坐标
        self.target_rel_speed = 0                   # type: int     # 目标速度
        self.target_rel_yaw = 0.0                   # type: float   # 目标航向角
        self.target_rel_yaw_fil = 0.0				# type: float   # 滤波后的目标航向角
        self.target_rel_turn_angle = 0.0            # type: float   # 目标转角

        # 环绕控制相关变量
        self.orbit_radius = 0.0            # type: float   # 环绕半径
        self.orbit_speed = 0               # type: int     # 环绕速度
        self.orbit_yaw = 0.0               # type: float   # 环绕航向角
        self.orbit_turn_angle = 0.0        # type: float   # 环绕转角
        self.current_dis = 0.0             # type: float   # 当前距离
        self.total_dis = 0.0                 # type: float   # 总距离
        self.max_orbit_speed = self.flash_sys.find_value("max_orbit_speed")   # type: int   # 最大环绕速度
        self.min_orbit_speed = self.flash_sys.find_value("min_orbit_speed")   # type: int   # 最小环绕速度

        # 标志位
        self.if_send_servo_command = False   # type: bool   # 是否发送视觉伺服控制指令标志位
        self.finish_servo = False      # type: bool   # 是否完成视觉伺服控制标志位
        self.if_gain_dis = False       # type: bool   # 是否获取目标距离标志位
        self.finish_orbit = False      # type: bool   # 是否完成环绕控制标志位

        # 计算目标航向角
    def compute_target_rel_yaw(self):
        # 计算目标角度，单位：度（注意避免除以0）
        if self.target_rel_speed_y == 0.0:
            if self.target_rel_speed_x > 0.0:
                self.target_rel_yaw = 90.0
            elif self.target_rel_speed_x < 0.0:
                self.target_rel_yaw = -90.0
        elif self.target_rel_speed_x == 0.0:
            if self.target_rel_speed_y > 0.0:	
                self.target_rel_yaw = 0.0
            elif self.target_rel_speed_y < 0.0:
                self.target_rel_yaw = 180.0
        else:  
            if self.target_rel_speed_x > 0.0 and self.target_rel_speed_y < 0.0:
                self.target_rel_yaw = math.atan(self.target_rel_speed_x / self.target_rel_speed_y) * 180.0 / self.MATH.PI + 180.0
            elif self.target_rel_speed_x < 0.0 and self.target_rel_speed_y < 0.0:
                self.target_rel_yaw = math.atan(self.target_rel_speed_x / self.target_rel_speed_y) * 180.0 / self.MATH.PI - 180.0
            else:
                self.target_rel_yaw = math.atan(self.target_rel_speed_x / self.target_rel_speed_y) * 180.0 / self.MATH.PI

    # 计算小车需要转向的角度（一般为0）
    def compute_target_rel_turn_angle(self, turn_angle_target: float):
        self.target_rel_turn_angle = turn_angle_target


    # 传入物体中心点的实际像素坐标，计算目标速度
    def visual_servo_control(self):
        # 通过标志位控制只向openart发送一次视觉伺服控制指令
        if self.if_send_servo_command == False:
            self.my_order_manager.mode_target()
            self.if_send_servo_command = True
        else:
            self.target_point = self.my_art_protocol.coordinate_receive()
            if self.target_point:
                self.servo_pid.compute_pid(self.target_point[0], self.target_point[1])
                # 测试，可能阻塞，记得删去
                # self.my_uart3.write(f"x: {self.target_point[0]}, y: {self.target_point[1]}, target_yaw: {self.target_rel_yaw}, {self.servo_pid.current_y}\r\n")
                self.target_rel_speed_x = self.servo_pid.pwm_output_x * 1.2
                self.target_rel_speed_y = self.servo_pid.pwm_output_y

                if self.finish_servo == False:
                    # 判断是否完成视觉伺服控制
                    if abs(self.servo_pid.nowError_x) <= self.finish_threshold_x and abs(self.servo_pid.nowError_y) <= self.finish_threshold_y:
                        self.target_rel_speed = 0
                        self.target_rel_yaw = 0.0
                        self.my_order_manager.finish()
                        # 测试
                        self.my_beep.test()
                        self.my_uart3.write("x: %d, y: %d, target_yaw: %.2f, current_x: %.2f, current_y: %.2f\r\n" % (self.target_point[0], self.target_point[1], self.target_rel_yaw, self.servo_pid.current_x, self.servo_pid.current_y))
                        self.finish_servo = True
                    else:
                        # 计算综合目标速度和航向角
                        # 目标速度放大两倍
                        self.target_rel_speed = int(math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2)) * 2
                        # 伺服速度限幅
                        if self.target_rel_speed < self.min_rel_speed:
                            self.target_rel_speed = self.min_rel_speed
                        elif self.target_rel_speed > self.max_rel_speed:
                            self.target_rel_speed = self.max_rel_speed
                        # 测试
                        self.compute_target_rel_yaw()
                        # self.target_rel_speed = 0
                        self.target_rel_yaw = self.servo_yaw_fil.update(self.target_rel_yaw)
                        # 后续需要调整该转角的计算方式，让小车面向物体进行视觉伺服控制
                        self.compute_target_rel_turn_angle(0.0)	

    # 环绕控制函数，传入环绕物体旋转的目标角度（单位：度），顺时针为正，逆时针为负
    def orbit_control(self, target_angle: float):
        if self.if_gain_dis == False:
            if len(self.tof_buffer) <= 35:          
                # 获取TOF测距值，并添加到缓冲区
                self.tof_buffer.append(self.tof_distance_fil.update(self.my_tof.get()))
                # 测试
                self.my_uart3.write("tof_distance: {:<f}\n".format(self.tof_buffer[-1]))
            else:
                # 计算最终的TOF测距值（去除前5个的平均值）
                self.tof_distance = sum(self.tof_buffer[5:]) / len(self.tof_buffer[5:])
                # 3.0为网球半径，8.0为tod传感器到车身中心的距离，可以根据物体种类选择合适的旋转半径
                self.orbit_radius = self.tof_distance + 3.0 + 8.0
                # 限制目标角度在-180到180度之间
                if target_angle > 180.0:
                    target_angle -= 360.0
                elif target_angle < -180.0:
                    target_angle += 360.0
                # 确定旋转方向（顺时针还是逆时针）
                if target_angle >= 0.0:
                    self.orbit_yaw = -90.0
                else:
                    self.orbit_yaw = 90.0   
                self.current_dis = 0.0
                self.total_dis = self.orbit_radius * abs(target_angle) * self.MATH.PI / 180.0
                self.if_gain_dis = True
                self.tof_buffer.clear()
                # 测试
                self.my_beep.test()
                self.my_uart3.write("final_tof: {:<f}, orbit_radius: {:<f}, total_dis: {:<f}\n".format(self.tof_distance, self.orbit_radius, self.total_dis))
        else:
            # 更新当前小车的目标转角
            self.orbit_turn_angle = self.my_car.car_speed_x / self.orbit_radius * 180.0 / self.MATH.PI
            # 更新当前小车的行驶距离
            self.current_dis += self.my_car.car_speed_x
            # 更新当前小车的速度
            self.orbit_speed = int(self.max_orbit_speed - (self.max_orbit_speed - self.min_orbit_speed) * (self.current_dis / self.total_dis))
            # 速度限幅
            if self.orbit_speed < self.min_orbit_speed:
                self.orbit_speed = self.min_orbit_speed
            elif self.orbit_speed > self.max_orbit_speed:
                self.orbit_speed = self.max_orbit_speed
            # 判断是否完成环绕
            if self.current_dis >= self.total_dis:
                self.finish_orbit = True

"""
# 视觉伺服测试函数
def test_vision_servo_1():
    if my_state.state == my_state.NAVIGATE:
        # 按照路径1进行导航
        my_plan.navigate(plan_data.path_1)
        if my_plan.finish_navigate == True:
            my_state.state = my_state.SERVO
            my_plan.finish_navigate = False
    elif my_state.state == my_state.SERVO:
        # 接收openart发送的目标点坐标
        target_point = ant_else.uart_receive()
        if target_point:
            my_vision_manager_1.visual_servo_control(target_point[0], target_point[1])
    elif my_state.state == my_state.STOP:
        pass
"""