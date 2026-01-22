import math

# 状态机制
class StateMachine:
    def __init__(self):
        self.NAVIGATE = 1    # 导航状态
        self.SERVO = 2       # 视觉伺服状态
        self.MOVE = 3        # 搬运状态
        self.RETURN = 4      # 返回状态
        self.STOP = 5        # 停止状态
        self.state = self.NAVIGATE  # 初始状态为导航状态


# 路径和速度规划相关常量
class Plan_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys

        # 地图固定点坐标
        self.fixed_point = [[0.0, 0.0], [0.0, 150.0], [150.0, 0.0], [150.0, 150.0], [8.9, 8.0], [-8.8, -9.1]]  # type: list
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


class Plan:
    def __init__(self, flash_sys, plan_data: Plan_data, math, car, wireless):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入数学常量对象
        self.MATH = math
        # 注入小车位置对象
        self.my_car = car
        # 注入无线通信对象
        self.my_wireless = wireless

        # 速度规划相关常量
        self.min_start_v = self.flash_sys.find_value("min_start_v")           # type: int  # 最小启动速度
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
        self.turn_angle_target = 0       # type: float
        self.error_correct_x = 0.0       # type: float
        self.error_correct_y = 0.0       # type: float
        # 判断小车是否到达目标点的阈值
        self.plan_arrive_threshold = self.flash_sys.find_value("plan_arrive_threshold")  # type: float
        self.total_distance = 0.0       # type: float
        self.finished_distance = 0.0    # type: float
        self.rest_distance = 0.0        # type: float
        # 目标路径
        self.path_points = []      # type: list
        # 标志位
        self.arrive_flag = False        # type: bool  # 判断是否到达目标点标志位
        self.transition_flag = True     # type: bool  # 判断是否过渡完成标志位
        self.if_set_path = False        # type: bool  # 判断是否设置路径标志位
        self.finish_navigate = False    # type: bool  # 判断是否完成导航标志位
        

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
                    self.v_target = self.min_start_v + int(self._ease_out_quad(self.elapsed_time / self.boost_time_threshold) * (self.long_v_max - self.min_start_v))
                else:
                    self.v_target = self.long_v_max
                    self.stage = self.TRANSIT
                    self.elapsed_time = 0
            elif self.stage == self.TRANSIT:
                self.v_target = self.v_max
                #if self.rest_distance < self.dec_distance:
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
        if self.total_distance >= 8.0:
           self.v_max = self.long_v_max
        else:
          self.v_max = self.short_v_max
        # 计算减速距离
        self.dec_distance = self.total_distance * self.dec_ratio
        self.build_dec_speed_list(0)
        self.arrive_flag = False
        # 测试
        self.v_target = 50

    # 更新已完成和剩余距离并判断是否到达目标点
    def update_distance(self):
        self.finished_distance = math.sqrt((self.my_car.x_current - self.last_target_x) ** 2 + (self.my_car.y_current - self.last_target_y) ** 2)
        self.rest_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)
        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold:
            self.arrive_flag = True
            self.transition_flag = False
            # 将当前位置修正为目标点位置
            self.my_wireless.send_str("arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))
            self.finished_distance = 0.0
            self.rest_distance = 0.0
            self.dec_distance = 0.0
        # 每次更新距离后进行速度规划计算
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

    # 用于路径之间的过渡，保证小车平稳
    def path_transition(self):
        self.v_target = 0
        self.plan_data.time_counter += 1
        # 最终的过渡时间为 plan_point_transition_T * plan_calculate_T(单位：ms)
        if self.plan_data.time_counter >=  self.plan_data.plan_point_transition_T:
            self.plan_data.time_counter = 0
            self.my_wireless.send_str("real_arrive_point: {:<f},{:<f}\n".format(self.my_car.x_current, self.my_car.y_current))	
            self.my_car.x_current = self.ideal_target_x
            self.my_car.y_current = self.ideal_target_y	
            self.transition_flag = True

    # 停止小车运动
    def stop(self):
        self.v_target = 0
        self.target_yaw = 0.0
        self.turn_angle_target = self.my_car.now_yaw

    # 按照传入路径及进行惯性导航
    def navigate(self, path: list):
        if self.if_set_path == False:
            # 路径初始化
            self.path_points = path
            self.if_set_path = True
            self.finish_navigate = False
            self.plan_data.aimed_point_index = 0
            # 设置第一个目标点
            first_point = self.path_points[0]
            self.set_target_point(first_point[0], first_point[1])
            self.compute_target_yaw()
            self.compute_turn_angle_target(0)

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
                    self.compute_turn_angle_target(90.0)
            else:
                # 判断此时是否完成路径过渡
                if self.transition_flag == False:
                    self.path_transition()
                else:
                    # 如果还有下一个目标点，设置下一个目标点坐标
                    if self.plan_data.aimed_point_index < len(self.path_points):
                        next_point = self.path_points[self.plan_data.aimed_point_index]
                        self.set_target_point(next_point[0], next_point[1])
                        # 计算目标航向角
                        self.compute_target_yaw()
                        self.compute_turn_angle_target(90.0)
                    else:
                        self.stop()
        else:
            self.stop()
            self.my_car.x_current = self.ideal_target_x
            self.my_car.y_current = self.ideal_target_y
            self.dec_speed_index = 0
            self.path_points.clear()
            # 用于测试
            # self.if_set_path = False
            self.finish_navigate = True


"""
# 视觉伺服控制类1
class VisionManager_1:
    def __init__(self):
        # 位置校准相关变量
        self.convert_matrix = []  # type: list  # 单应性矩阵，用于将像素坐标转化为世界坐标
        self.x_rel_target = 0.0          # type: float  # 世界坐标系下的物体坐标
        self.y_rel_target = 0.0          # type: float  # 世界坐标系下的物体坐标
        self.target_offset_x = find_value(config, "target_offset_x")  # type: float  # 目标相对于小车中心的x偏移量
        self.target_offset_y = find_value(config, "target_offset_y")  # type: float  # 目标相对于小车中心的y偏移量
        self.finish_servo = False        # type: bool   # 是否完成视觉伺服控制标志位
        self.rel_dis_threshold = find_value(config, "rel_dis_threshold")  # type: float  # 视觉伺服控制距离阈值
        self.target_rel_yaw = 0.0           # type: float   # 目标航向角
        self.target_rel_turn_angle = 0.0    # type: float   # 目标转角
        # PD控制相关变量
        self.kp_rel = find_value(config, "kp_rel")         # type: float   # 视觉伺服控制x轴比例系数
        self.kd_rel = find_value(config, "kd_rel")         # type: float   # 视觉伺服控制x轴微分系数
        self.last_rel_dis = 0.0            # type: float   # 上一次目标距离
        self.rel_dis = 0.0                 # type: float   # 当前目标距离
        self.target_rel_speed = 0         # type: int   # 目标速度
        self.min_rel_speed = find_value(config, "min_rel_speed")   # type: int   # 最小视觉伺服速度

    def cord_trans(self, x, y): 
        a = self.convert_matrix
        denom = a[2][0] * x + a[2][1] * y + a[2][2]
        x_world = (a[0][0] * x + a[0][1] * y + a[0][2]) / denom
        y_world = (a[1][0] * x + a[1][1] * y + a[1][2]) / denom
        return (x_world, y_world)
    
    # 计算目标航向角
    def compute_target_rel_yaw(self):
        dx = self.x_rel_target - ant_motor.my_car.x_current
        dy = self.y_rel_target - ant_motor.my_car.y_current
        # 计算目标角度，单位：度（注意避免除以0）
        if dy == 0.0:
            if dx > 0.0:
                self.target_rel_yaw = 90.0
            elif dx < 0.0:
                self.target_rel_yaw = -90.0
        elif dx == 0.0:
            if dy > 0.0:
                self.target_rel_yaw = 0.0
            elif dy < 0.0:
                self.target_rel_yaw = 180.0
        else:  
            if dx > 0.0 and dy < 0.0:
                self.target_rel_yaw = math.atan(dx / dy) * 180.0 / MATH.PI + 180.0
            elif dx < 0.0 and dy < 0.0:
                self.target_rel_yaw = math.atan(dx / dy) * 180.0 / MATH.PI - 180.0
            else:
                self.target_rel_yaw = math.atan(dx / dy) * 180.0 / MATH.PI
    
    # 更新目标距离
    def update_target_rel_dis(self):
        self.last_rel_dis = self.rel_dis
        self.rel_dis= math.sqrt((self.x_rel_target - ant_motor.my_car.x_current) ** 2 + (self.y_rel_target - ant_motor.my_car.y_current) ** 2)
        # 判断是否完成视觉伺服控制
        if self.rel_dis <= self.rel_dis_threshold:
            self.finish_servo = True
    
    # 计算小车需要转向的角度（一般为0）
    def compute_target_rel_turn_angle(self, turn_angle_target: float):
        self.target_rel_turn_angle = turn_angle_target
    
    # 视觉伺服控制
    def visual_servo_control(self, x: float, y: float):
        # 将像素坐标转换为世界坐标
        (x_world, y_world) = self.cord_trans(x, y)
        # 计算目标相对于坐标系原点的世界坐标
        self.x_rel_target = ant_motor.my_car.x_current + x_world + self.target_offset_x
        self.y_rel_target = ant_motor.my_car.y_current + y_world + self.target_offset_y
        self.update_target_rel_dis()
        if self.finish_servo == False:
            self.compute_target_rel_yaw()
            self.compute_target_rel_turn_angle(0.0)
            # PD控制计算目标速度
            self.target_rel_speed = int(self.kp_rel * self.rel_dis + self.kd_rel * (self.rel_dis - self.last_rel_dis))
            if self.target_rel_speed < self.min_rel_speed:
                self.target_rel_speed = self.min_rel_speed
        else:
            self.target_rel_speed = 0
            self.target_rel_yaw = 0.0
            ant_else.finish_servo()
            self.finish_servo = False
            # 测试
            my_state.state = my_state.STOP

# 创建视觉伺服管理对象
my_vision_manager_1 = VisionManager_1()
"""

 # 视觉伺服控制类2(PD控制器)
class VisionManager_2:
    def __init__(self, flash_sys, beep, math, servo_pid_x, servo_pid_y, servo_yaw_fil):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入数学常量对象
        self.MATH = math
        # 注入伺服PD控制器对象
        self.servo_pid_x = servo_pid_x
        self.servo_pid_y = servo_pid_y
        # 注入蜂鸣器对象
        self.beep = beep
        # 注入航向角滤波器对象
        self.servo_yaw_fil = servo_yaw_fil

        # PD控制相关变量
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")  # type: float  # 视觉伺服控制距离阈值
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")  # type: float  # 视觉伺服控制距离阈值
        self.target_rel_speed_x = 0          # type: int   # 伺服控制目标x速度
        self.target_rel_speed_y = 0          # type: int   # 伺服控制目标y速度
        self.target_x = self.flash_sys.find_value("target_x")         # type: int   # 物体中心点的目标像素x坐标
        self.target_y = self.flash_sys.find_value("target_y")         # type: int   # 物体中心点的目标像素y坐标
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")   # type: int   # 最小视觉伺服速度
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")   # type: int   # 最小视觉伺服速度
        self.target_point = []					# 目标像素点
        self.target_rel_speed = 0                   # type: int     # 目标速度
        self.target_rel_yaw = 0.0                   # type: float   # 目标航向角
        self.target_rel_yaw_fil = 0.0				# type: float   # 滤波后的目标航向角
        self.target_rel_turn_angle = 0.0            # type: float   # 目标转角

        # 是否完成视觉伺服标志位
        self.finish_servo = False        # type: bool   # 是否完成视觉伺服控制标志位

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
                self.target_rel_yaw = math.atan(self.target_rel_speed_x / self.target_rel_speed_y) * 180.0 / MATH.PI + 180.0
            elif self.target_rel_speed_x < 0.0 and self.target_rel_speed_y < 0.0:
                self.target_rel_yaw = math.atan(self.target_rel_speed_x / self.target_rel_speed_y) * 180.0 / MATH.PI - 180.0
            else:
                self.target_rel_yaw = math.atan(self.target_rel_speed_x / self.target_rel_speed_y) * 180.0 / MATH.PI

    # 计算小车需要转向的角度（一般为0）
    def compute_target_rel_turn_angle(self, turn_angle_target: float):
        self.target_rel_turn_angle = turn_angle_target


    # 传入物体中心点的实际像素坐标，计算目标速度
    def visual_servo_control(self, x: int, y: int):
        self.target_rel_speed_x = -self.servo_pid_x.compute_pid(self.target_x, x)
        self.target_rel_speed_y = self.servo_pid_y.compute_pid(self.target_y, y) * 1.414
        # 判断是否完成视觉伺服控制
        if abs(self.servo_pid_x.nowError) <= self.finish_threshold_x and abs(self.servo_pid_y.nowError) <= self.finish_threshold_y:
            self.target_rel_speed = 0
            self.target_rel_yaw = 0.0
            self.beep.finish_servo()
            self.finish_servo = True
        else:
            # 计算综合目标速度和航向角
            self.target_rel_speed = int(math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2))
            # 伺服速度限幅
            if self.target_rel_speed < self.min_rel_speed:
                self.target_rel_speed = self.min_rel_speed
            elif self.target_rel_speed > self.max_rel_speed:
                self.target_rel_speed = self.max_rel_speed
            # 测试
            #self.target_rel_speed = 0
            self.compute_target_rel_yaw()
            self.target_rel_yaw = self.servo_yaw_fil.update(self.target_rel_yaw)
            self.compute_target_rel_turn_angle(0.0)
    
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