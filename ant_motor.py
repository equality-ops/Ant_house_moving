from machine import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker, encoder
import math
import ant_flash
from ant_math import MATH as MATH
from ant_flash import find_aimed_value as find_value
import ant_pose


###################################【文件读取】###################################
# 从config.txt中读取保存所有的参数并保存到config字典中
config = ant_flash.phase_config("/flash/config.txt")

# 滑动平均滤波器
class SlipAveragingFilter:
    # 构造对象时传入滤波窗口大小
    def __init__(self, filter_size: int):
        self.filter_size = filter_size
        self.index = 0
        self.buffer = [0] * filter_size

    # 滤波时传入一个新的数据，返回滤波后的结果(float)
    def filtering(self, data: int) -> float:
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.filter_size
        return sum(self.buffer) / self.filter_size
    
    
# 一维卡尔曼滤波器
class KalmanFilter:
    def __init__(self, P=1.0, Q=0.01, R=0.1, initial_output=0.0):
        self.P = P
        self.Q = Q
        self.R = R
        self.Output = initial_output

    def update(self, input_value):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.Output += K * (input_value - self.Output)
        self.P = (1 - K) * self.P
        return self.Output


# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = SlipAveragingFilter(5)    # 滤波窗口为5个
diff_filter_ur = SlipAveragingFilter(5)    # 滤波窗口为5个
diff_filter_md = SlipAveragingFilter(5)    # 滤波窗口为5个
diff_filter_gyroz = SlipAveragingFilter(5)  # 滤波窗口为5个

# 创建姿态数据对象
pose_data = ant_pose.PoseData(diff_filter_gyroz)


# 定义一个抽象类用于顶层设计
# 该类能够存储pid参数并计算得到当前应该输出的pwm值
class ControlPID:
    def compute_pid(self, target: int, actual: int) -> None:
        pass

# 速度环PID
class SpeedPositionPID(ControlPID):
    def __init__(self, kp: float, ki: float, kd: float, pwmout_limitmax: int, diff_filter: SlipAveragingFilter):
        self.kp = kp        # type: float
        self.ki = ki        # type: float
        self.kd = kd        # type: float
        self.nowError = 0   # type: int
        self.preError = 0   # type: int
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.__pwmout_limitmax = pwmout_limitmax    # type: float
        self.diff_filter = diff_filter
        self.__A = 800      # 变速积分上限
        self.__B = 200      # 变速积分下限

    def compute_pid(self, target: int, actual: int):
        self.preError = self.nowError
        self.nowError = target - actual
        self.integral += self.nowError
        # 对微分项进行滑动平均滤波
        self.derivative = self.diff_filter.filtering(self.nowError - self.preError)

        abs_integral = abs(self.integral)
        coefficient = 1.0   # type: float
        if self.__A == self.__B:
            # 避免除以0
            coefficient = 0.0 if (abs_integral > self.__A) else 1.0
        else:
            if abs_integral > self.__A:
                coefficient = 0.0
            elif abs_integral > self.__B:
                coefficient = (self.__A - abs_integral) / (self.__A - self.__B)
            else:
                coefficient = 1.0

        self.integral = self.integral * coefficient

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError + self.ki * self.integral * coefficient + self.kd * self.derivative

        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))


# 角度环PID
class AnglePositionPID(ControlPID):
    def __init__(self, kp: float, ki: float, kd: float, pwmout_limitmax: int):
        self.kp = kp        # type: float
        self.ki = ki        # type: float
        self.kd = kd        # type: float
        self.nowError = 0   # type: int
        self.preError = 0   # type: int
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.__pwmout_limitmax = pwmout_limitmax    # type: float

    def compute_pid(self, target: int, actual: int):
        self.preError = self.nowError
        self.nowError = target - actual
        self.integral += self.nowError
        self.derivative = self.nowError - self.preError

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError + self.ki * self.integral + self.kd * self.derivative

        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))

# 创建电机pid对象
motor_ul_pid = SpeedPositionPID(kp = find_value(config, "ul_normal_kp"), 
                                ki = find_value(config, "ul_normal_ki"), 
                                kd = find_value(config, "ul_normal_kd"),  
                                pwmout_limitmax = 6000, 
                                diff_filter = diff_filter_ul)

motor_ur_pid = SpeedPositionPID(kp = find_value(config, "ur_normal_kp"), 
                                ki = find_value(config, "ur_normal_ki"), 
                                kd = find_value(config, "ur_normal_kd"),  
                                pwmout_limitmax = 6000, 
                                diff_filter = diff_filter_ur)

motor_md_pid = SpeedPositionPID(kp = find_value(config, "md_normal_kp"), 
                                ki = find_value(config, "md_normal_ki"), 
                                kd = find_value(config, "md_normal_kd"),  
                                pwmout_limitmax = 6000, 
                                diff_filter = diff_filter_md)

angle_pid = AnglePositionPID(kp = find_value(config, "angle_normal_kp"), 
                            ki = find_value(config, "amgle_normal_ki"), 
                            kd = find_value(config, "amgle_normal_kd"),  
                            pwmout_limitmax = 4000)

# 创建MOTOR_CONTROLLER对象
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty = 0, invert = False)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty = 0, invert = True)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)

# 小车姿态控制
class CarPose:
    def __init__(self):
        # 机械参数
        self.wheel_radius = find_value(config, "wheel_radius")  # type: float  # 轮半径，单位：cm
        self.car_radius = find_value(config, "car_radius")          # type: float  # 车体半径，单位：cm
        # 上一次速度
        self.last_car_speed_x = 0.0  # type: float
        self.last_car_speed_y = 0.0  # type: float
        self.last_car_speed_w = 0.0  # type: float
        # 小车当前速度
        self.car_speed_x = 0.0  # type: float
        self.car_speed_y = 0.0  # type: float
        self.car_speed_w = 0.0  # type: float
        # 小车在世界坐标系下的速度
        self.real_speed_x = 0.0  # type: float
        self.real_speed_y = 0.0  # type: float
        self.real_speed_w = 0.0  # type: float
        # 目标速度
        self.target_speed_x = 0.0  # type: float
        self.target_speed_y = 0.0  # type: float
        self.target_speed_w = 0.0  # type: float
        # 速度系数
        self.conversion_gamma = find_value(config, "conversion_gamma")    # 一个脉冲在一个周期(0.005s)内的速度转换系数，单位：cm/s
        self.gkd = find_value(config, "gkd")  # type: float  # 角速度补偿系数
        self.fuse_ratio = find_value(config, "fuse_ratio")  # type: float  # 编码器和陀螺仪融合系数
        self.alpha_x = 1.0  # type: float
        self.alpha_y = 1.0  # type: float
        self.alpha_w = 1.0  # type: float
        self.beta_x = 1.0  # type: float
        self.beta_y = 1.0  # type: float
        self.beta_z = 1.0  # type: float
        # 位置
        self.x_last = 0.0   # type: float
        self.y_last = 0.0   # type: float
        self.x_current = 0.0   # type: float
        self.y_current = 0.0   # type: float
        self.x_next = 0.0   # type: float
        self.y_next = 0.0   # type: float
        self.now_yaw = 0.0  # type: float
        # 采集周期
        self.collect_dt = find_value(config, "collect_dt")     # type: float  # 单位：秒

    # 小车姿态更新
    def update_pose(self, pose_data: ant_pose.PoseData):
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        self.last_car_speed_w = self.car_speed_w
        # 计算小车当前x,y速度
        # car_speed_x, car_speed_y单位：cm/s
        self.car_speed_x = (MATH.SIN30 * (pose_data.encoder_data_ur + pose_data.encoder_data_ul) - pose_data.encoder_data_md) * self.conversion_gamma
        self.car_speed_y = (MATH.COS30 * (pose_data.encoder_data_ul - pose_data.encoder_data_ur)) * self.conversion_gamma
        # 计算小车当前角速度
        # car_speed_w单位：角度每秒
        self.car_speed_w = self.fuse_ratio * (pose_data.encoder_data_ur + pose_data.encoder_data_ul + pose_data.encoder_data_md) * self.conversion_gamma / self.car_radius * MATH.PI / 180 + (1 - self.fuse_ratio) * pose_data.gyro_z
        # 计算小车在世界坐标系下的偏航角
        # now_yaw单位：角度
        self.now_yaw += self.car_speed_w * self.collect_dt 
        # 限定now_yaw在-180到180度之间
        if self.now_yaw >= 180:  self.now_yaw -= 360
        elif self.now_yaw <= -180:  self.now_yaw += 360
        # 计算小车在世界坐标系下的速度
        self.real_speed_x = self.car_speed_x * math.cos(math.radians(self.now_yaw)) - self.car_speed_y * math.sin(math.radians(self.now_yaw))
        self.real_speed_y = self.car_speed_x * math.sin(math.radians(self.now_yaw)) + self.car_speed_y * math.cos(math.radians(self.now_yaw))
        self.real_speed_w = self.car_speed_w

    # 全向移动控制函数
    def move_ctrl(self, move_speed_target: int, move_angle_target: int, turn_angle_target: int):
        # 计算各个电机的目标速度
        speed_w = turn_angle_target - self.now_yaw
        if speed_w > 180:  speed_w -= 360
        elif speed_w < -180:  speed_w += 360

        self.target_speed_w = speed_w
        self.target_speed_x = move_speed_target * math.cos(math.radians(move_angle_target))
        self.target_speed_y = move_speed_target * math.sin(math.radians(move_angle_target))
    
        motor_ul_speed_target = (self.target_speed_w + self.target_speed_x) * MATH.OneThird + self.target_speed_y / MATH.SQRT3 + self.car_speed_w * self.gkd
        motor_ur_speed_target = (self.target_speed_w + self.target_speed_x) * MATH.OneThird - self.target_speed_y / MATH.SQRT3 + self.car_speed_w * self.gkd
        motor_md_speed_target = self.target_speed_w * MATH.OneThird - self.target_speed_x * MATH.TwoThirdS + self.car_speed_w * self.gkd
        
        # 设置各个电机的目标速度
        motor_ul_pid.compute_pid(int(motor_ul_speed_target), pose_data.encoder_data_ul)
        motor_ur_pid.compute_pid(int(motor_ur_speed_target), pose_data.encoder_data_ur)
        motor_md_pid.compute_pid(int(motor_md_speed_target), pose_data.encoder_data_md)

    def set_motor_pwm(self):
        motor_ul.duty(int(motor_ul_pid.pwm_output))
        motor_ur.duty(int(motor_ur_pid.pwm_output))
        motor_md.duty(int(motor_md_pid.pwm_output))


my_car = CarPose()

# 定时器1中断回调函数
def time_pit1_handler(time):
    pose_data.update_data()
    my_car.update_pose(pose_data)
    my_car.move_ctrl(50, 0, 0)
    my_car.set_motor_pwm()