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


class Plan:
    def __init__(self):
        self.last_target_x = 0.0   # type: float
        self.last_target_y = 0.0   # type: float
        self.last_target_yaw = 0.0 # type: float
        self.target_x = 0.0        # type: float
        self.target_y = 0.0        # type: float
        self.target_yaw = 0.0      # type: float
        # 判断小车是否到达目标点的阈值
        self.arrive_threshold = find_value(ant_motor.config, "plan_arrive_threshold")  # type: float
        self.arrive_flag = False   # type: bool
        self.total_distance = 0.0  # type: float
        self.finished_distance = 0.0  # type: float
        self.rest_distance = 0.0   # type: float

        self.v_target = 0.0       # type: float

    def set_target_point(self, x: float, y: float):
        self.last_target_x = ant_motor.my_car.x_current
        self.last_target_y = ant_motor.my_car.y_current
        self.last_target_yaw = self.target_yaw
        self.target_x = x
        self.target_y = y
        self.total_distance = math.sqrt((x - ant_motor.my_car.x_current) ** 2 + (y - ant_motor.my_car.y_current) ** 2)
        self.arrive_flag = False

    def update_distance(self):
        self.finished_distance = math.sqrt((ant_motor.my_car.x_current - self.last_target_x) ** 2 + (ant_motor.my_car.y_current - self.last_target_y) ** 2)
        self.rest_distance = math.sqrt((self.target_x - ant_motor.my_car.x_current) ** 2 + (self.target_y - ant_motor.my_car.y_current) ** 2)

        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.arrive_threshold:
            self.arrive_flag = True


    def compute_target_yaw(self):
        dx = self.target_x - ant_motor.my_car.x_current
        dy = self.target_y - ant_motor.my_car.y_current
        # 计算目标角度，单位：弧度（注意避免除以0）
        if dy == 0:
            if dx > 0:
                self.target_yaw = MATH.PI / 2
            elif dx < 0:
                self.target_yaw = -MATH.PI / 2
        if dx == 0:
            if dy > 0:
                self.target_yaw = 0.0
            elif dy < 0:
                self.target_yaw = MATH.PI
        else:  
            if dx > 0 and dy < 0:
                self.target_yaw = math.atan2(dx, dy) + MATH.PI
            elif dx < 0 and dy < 0:
                self.target_yaw = math.atan2(dx, dy) - MATH.PI
            else:
                self.target_yaw = math.atan2(dx, dy)


# 创建规划（路径和速度）对象
my_plan = Plan()

# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(timer) -> None:
    pass