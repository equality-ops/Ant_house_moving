from machine import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker
import ant_motor
import math
import ant_flash
from ant_math import MATH as MATH
from ant_flash import find_aimed_value as find_value
import ant_pose
import ant_uart

# 路径和速度规划相关常量
class Plan_data:
    def __init__(self):
        # 地图固定点坐标
        self.fixed_point = [[0.0, 0.0], [0.0, 13.5], [13, 0.0], [13, 13.5], [8.8, 9.1], [-8.8, -9.1]]  # type: list
        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 坐标误差修正量
        self.error_correct_x_50_1 = find_value(ant_motor.config, "error_correct_x_50_1") # type: float
        self.error_correct_y_50_1 = find_value(ant_motor.config, "error_correct_y_50_1")  # type: float
        self.error_correct_x_50_2 = find_value(ant_motor.config, "error_correct_x_50_2") # type: float
        self.error_correct_y_50_2 = find_value(ant_motor.config, "error_correct_y_50_2")  # type: float
        self.error_correct_x_50_3 = find_value(ant_motor.config, "error_correct_x_50_3") # type: float
        self.error_correct_y_50_3 = find_value(ant_motor.config, "error_correct_y_50_3")  # type: float
        self.error_correct_x_50_4 = find_value(ant_motor.config, "error_correct_x_50_4") # type: float  
        self.error_correct_y_50_4 = find_value(ant_motor.config, "error_correct_y_50_4")  # type: float
        self.error_correct_x_50_5 = find_value(ant_motor.config, "error_correct_x_50_5") # type: float
        self.error_correct_y_50_5 = find_value(ant_motor.config, "error_correct_y_50_5")  # type: float
        self.error_correct_x_50_6 = find_value(ant_motor.config, "error_correct_x_50_6") # type: float
        self.error_correct_y_50_6 = find_value(ant_motor.config, "error_correct_y_50_6")  # type: float
        self.error_correct_x_50_7 = find_value(ant_motor.config, "error_correct_x_50_7") # type: float
        self.error_correct_y_50_7 = find_value(ant_motor.config, "error_correct_y_50_7")  # type: float
        self.error_correct_x_50_8 = find_value(ant_motor.config, "error_correct_x_50_8") # type: float
        self.error_correct_y_50_8 = find_value(ant_motor.config, "error_correct_y_50_8")  # type: float
        # 时间计数器
        self.time_counter = 0          # type: int
        # 路径点切换时间阈值（用于过渡）
        self.plan_point_transition_T = find_value(ant_motor.config, "plan_point_transition_T")

plan_data = Plan_data()

class Plan:
    def __init__(self):
        # 速度规划相关常量
        self.min_start_v = 30           # type: int  # 最小启动速度
        self.long_v_max = 80            # type: int  # 长距离时的最大速度
        self.short_v_max = 60           # type: int  # 短距离时的最大速度
        self.BOOST = 1                  # type: int  # 死区启动标志位
        self.TRANSIT = 2                # type: int  # 过渡阶段标志位
        self.DEC = 3                    # type: int  # 减速阶段标志位
        self.STOP = 4                   # type: int  # 停止标志位
        self.dec_ratio = find_value(ant_motor.config, "dec_ratio")	# type: float  # 减速段占据的比例
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
        self.boost_time_threshold = find_value(ant_motor.config, "boost_time_threshold")  # type: int  # 死区启动时间阈值
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
        self.turn_angle_target = 0       # type: int
        self.error_correct_x = 0.0       # type: float
        self.error_correct_y = 0.0       # type: float
        # 判断小车是否到达目标点的阈值
        self.plan_arrive_threshold = find_value(ant_motor.config, "plan_arrive_threshold")  # type: float
        # 判断是否到达目标点标志位
        self.arrive_flag = False        # type: bool
        # 判断是否过渡完成标志位
        self.transition_flag = True     # type: bool
        self.total_distance = 0.0       # type: float
        self.finished_distance = 0.0    # type: float
        self.rest_distance = 0.0        # type: float

        # 目标路径
        self.path_points = []      # type: list

        # 速度规划相关变量
        self.v_target = 0       # type: int

    def _ease_out_quad(self, t):
        """二次缓出曲线，用于快速启动"""
        return -t * (t - 2)
    
    # 构建减速速度表
    def build_dec_speed_list(self, i):
        if self.finish_building == False:
            real_dec_distance = self.dec_distance / ant_motor.my_car.position_conversion_gamma
            # 计算加加速度
            self.j = (self.v_max ** 3) / (real_dec_distance ** 2) 
            # 计算减速总时间
            if self.j == 0:
                self.half_time = 0
            else:
                self.half_time = math.sqrt(self.v_max / self.j)
            self.total_time = 2 * self.half_time
            # 计算减速距离对应的速度点个数
            self.dec_lenth = int(real_dec_distance) + 1
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
        dx = self.ideal_target_x - ant_motor.my_car.x_current
        dy = self.ideal_target_y - ant_motor.my_car.y_current
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
                blurry_yaw = math.atan(dx / dy) * 180.0 / MATH.PI + 180.0
            elif dx < 0.0 and dy < 0.0:
                blurry_yaw = math.atan(dx / dy) * 180.0 / MATH.PI - 180.0
            else:
                blurry_yaw = math.atan(dx / dy) * 180.0 / MATH.PI
                
        # 根据大致航向角选择合适的坐标修正量（解决因惯性造成的打滑问题）
        if blurry_yaw >= -30.0 and blurry_yaw < 30.0:
            self.error_correct_x = plan_data.error_correct_x_50_1
            self.error_correct_y = plan_data.error_correct_y_50_1
        elif blurry_yaw >= 30.0 and blurry_yaw < 60.0:
            self.error_correct_x = plan_data.error_correct_x_50_2
            self.error_correct_y = plan_data.error_correct_y_50_2
        elif blurry_yaw >= 60.0 and blurry_yaw < 120.0:
            self.error_correct_x = plan_data.error_correct_x_50_3
            self.error_correct_y = plan_data.error_correct_y_50_3
        elif blurry_yaw >= 120.0 and blurry_yaw < 150.0:
            self.error_correct_x = plan_data.error_correct_x_50_4
            self.error_correct_y = plan_data.error_correct_y_50_4
        elif blurry_yaw >= 150.0 and blurry_yaw <= 180.0 or blurry_yaw >= -180.0 and blurry_yaw < -150.0:
            self.error_correct_x = plan_data.error_correct_x_50_5
            self.error_correct_y = plan_data.error_correct_y_50_5
        elif blurry_yaw >= -150.0 and blurry_yaw < -120.0:
            self.error_correct_x = plan_data.error_correct_x_50_6
            self.error_correct_y = plan_data.error_correct_y_50_6
        elif blurry_yaw >= -120.0 and blurry_yaw < -60.0:
            self.error_correct_x = plan_data.error_correct_x_50_7
            self.error_correct_y = plan_data.error_correct_y_50_7
        elif blurry_yaw >= -60.0 and blurry_yaw < -30.0:
            self.error_correct_x = plan_data.error_correct_x_50_8
            self.error_correct_y = plan_data.error_correct_y_50_8
        
        # 实际条件下的目标坐标
        self.real_target_x = self.ideal_target_x + self.error_correct_x
        self.real_target_y = self.ideal_target_y + self.error_correct_y
        # 实际距离坐标点的总距离
        self.total_distance = math.sqrt((self.real_target_x - ant_motor.my_car.x_current) ** 2 + (self.real_target_y - ant_motor.my_car.y_current) ** 2)
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
        self.finished_distance = math.sqrt((ant_motor.my_car.x_current - self.last_target_x) ** 2 + (ant_motor.my_car.y_current - self.last_target_y) ** 2)
        self.rest_distance = math.sqrt((self.real_target_x - ant_motor.my_car.x_current) ** 2 + (self.real_target_y - ant_motor.my_car.y_current) ** 2)
    
        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold:
            self.arrive_flag = True
            self.transition_flag = False
            # 将当前位置修正为目标点位置
            ant_uart.wireless.send_str("arrive_point: {:<f},{:<f}\n".format(ant_motor.my_car.x_current, ant_motor.my_car.y_current))
            self.finished_distance = 0.0
            self.rest_distance = 0.0
            self.dec_distance = 0.0

        # 每次更新距离后进行速度规划计算
        self.planning_speed()

    # 计算目标航向角
    def compute_target_yaw(self):
        dx = self.real_target_x - ant_motor.my_car.x_current
        dy = self.real_target_y - ant_motor.my_car.y_current
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
                self.target_yaw = math.atan(dx / dy) * 180.0 / MATH.PI + 180.0
            elif dx < 0.0 and dy < 0.0:
                self.target_yaw = math.atan(dx / dy) * 180.0 / MATH.PI - 180.0
            else:
                self.target_yaw = math.atan(dx / dy) * 180.0 / MATH.PI

    # 计算小车需要转向的角度（一般为0）
    def compute_turn_angle_target(self, turn_angle_target: int):
        self.turn_angle_target = turn_angle_target

    # 用于路径之间的过渡，保证小车平稳
    def path_transition(self):
        self.v_target = 0
        plan_data.time_counter += 1
        # 最终的过渡时间为 plan_point_transition_T * plan_calculate_T(单位：ms)
        if plan_data.time_counter >=  plan_data.plan_point_transition_T:
            plan_data.time_counter = 0
            ant_motor.my_car.x_current = self.ideal_target_x
            ant_motor.my_car.y_current = self.ideal_target_y	
            self.transition_flag = True

    def stop(self):
        self.v_target = 0
        self.target_yaw = 0.0


# 创建规划（路径和速度）对象
my_plan = Plan()

# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
    # 测试MCU与openart通信
    #target_point = ant_uart.uart_receive()
    #if target_point:
    #    ant_uart.wireless.send_str("x: {:<f}, y: {:<f}\n".format(target_point[0], target_point[1]))
    
    #ant_uart.my_uart6.write("hello\r\n")
    # 判断是否还有未到达的目标点
    if plan_data.aimed_point_index < len(my_plan.path_points):
        # 判断是否到达下一个目标点
        if my_plan.arrive_flag == False:
            my_plan.update_distance()
            if my_plan.arrive_flag == True:
                # 到达目标点后，更新目标点索引
                plan_data.aimed_point_index += 1
                # 进行路径过渡
                my_plan.path_transition()
            else:
                # 计算目标航向角
                my_plan.compute_target_yaw()
                my_plan.compute_turn_angle_target(0)
        else:
            # 判断此时是否完成路径过渡
            if my_plan.transition_flag == False:
                my_plan.path_transition()
            else:
                # 如果还有下一个目标点，设置下一个目标点坐标
                if plan_data.aimed_point_index < len(my_plan.path_points):
                    next_point = my_plan.path_points[plan_data.aimed_point_index]
                    my_plan.set_target_point(next_point[0], next_point[1])
                else:
                    my_plan.stop()
                
    
    else:
        my_plan.stop()
        ant_motor.my_car.x_current = my_plan.ideal_target_x
        ant_motor.my_car.y_current = my_plan.ideal_target_y
        


