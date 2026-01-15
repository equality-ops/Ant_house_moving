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
        self.fixed_point = [[0.0, 0.0], [0.0, 63.0], [63.0, 0.0], [63.0, 63.0], [36.5, 36.5]]  # type: list
        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 坐标误差修正量
        self.error_correct_x_400 = 2.7  # type: float
        self.error_correct_y_400 = 2.7  # type: float
        # 时间计数器
        self.time_counter = 0          # type: int
        # 路径点切换时间阈值（用于过渡）
        self.plan_point_transition_T = find_value(ant_motor.config, "plan_point_transition_T")

plan_data = Plan_data()

class Plan:
    def __init__(self):
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
        self.transition_flag = True    # type: bool
        self.total_distance = 0.0       # type: float
        self.finished_distance = 0.0    # type: float
        self.rest_distance = 0.0        # type: float

        # 目标路径
        self.path_points = []      # type: list

        # 速度规划相关变量
        self.v_target = 0       # type: int

    # 设置目标点坐标
    def set_target_point(self, x: float, y: float):
        self.last_target_x = self.real_target_x
        self.last_target_y = self.real_target_y
        self.last_target_yaw = self.target_yaw
        # 理想条件下的目标坐标
        self.ideal_target_x = x
        self.ideal_target_y = y
        # 实际条件下的目标坐标
        self.real_target_x = self.ideal_target_x + plan_data.error_correct_x_400
        self.real_target_y = self.ideal_target_y + plan_data.error_correct_y_400
        # 实际距离坐标点的总距离
        self.total_distance = math.sqrt((self.real_target_x - ant_motor.my_car.x_current) ** 2 + (self.real_target_y - ant_motor.my_car.y_current) ** 2)
        self.arrive_flag = False

    # 更新已完成和剩余距离并判断是否到达目标点
    def update_distance(self):
        self.finished_distance = math.sqrt((ant_motor.my_car.x_current - self.last_target_x) ** 2 + (ant_motor.my_car.y_current - self.last_target_y) ** 2)
        self.rest_distance = math.sqrt((self.real_target_x - ant_motor.my_car.x_current) ** 2 + (self.real_target_y - ant_motor.my_car.y_current) ** 2)

        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold:
            self.arrive_flag = True
            self.transition_flag = False
            # 将当前位置修正为目标点位置
            ant_motor.my_car.x_current = self.ideal_target_x
            ant_motor.my_car.y_current = self.ideal_target_y
            self.finished_distance = 0.0
            self.rest_distance = 0.0

    # 计算目标航向角
    def compute_target_yaw(self):
        dx = self.real_target_x - ant_motor.my_car.x_current
        dy = self.real_target_y - ant_motor.my_car.y_current
        # 计算目标角度，单位：度（注意避免除以0）
        if dy == 0:
            if dx > 0:
                self.target_yaw = 90.0
            elif dx < 0:
                self.target_yaw = -90.0
        if dx == 0:
            if dy > 0:
                self.target_yaw = 0.0
            elif dy < 0:
                self.target_yaw = 180.0
        else:  
            if dx > 0 and dy < 0:
                self.target_yaw = math.atan2(dx, dy) * 180 / MATH.PI + 180.0
            elif dx < 0 and dy < 0:
                self.target_yaw = math.atan2(dx, dy) * 180 / MATH.PI - 180.0
            else:
                self.target_yaw = math.atan2(dx, dy) * 180 / MATH.PI

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
            self.arrive_flag = True

    def stop(self):
        self.v_target = 0
        self.target_yaw = 0.0



# 创建规划（路径和速度）对象
my_plan = Plan()

# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
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
