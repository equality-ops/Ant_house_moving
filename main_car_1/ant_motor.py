from machine import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker
import math
import ant_flash
from ant_math import MATH as MATH
from ant_flash import find_aimed_value as find_value
import ant_uart
import time

###################################【文件读取】###################################
# 从config.txt中读取保存所有的参数并保存到config字典中
config = ant_flash.phase_config("/flash/config.txt")

# 读取完再导入
import ant_plan
import ant_pose

class PID_data:
    def __init__(self):
        self.ul_normal_kp = find_value(config, "ul_normal_kp")  # type: float
        self.ul_normal_ki = find_value(config, "ul_normal_ki")  # type: float
        self.ul_normal_kd = find_value(config, "ul_normal_kd")  # type: float
        self.ur_normal_kp = find_value(config, "ur_normal_kp")  # type: float
        self.ur_normal_ki = find_value(config, "ur_normal_ki")  # type: float
        self.ur_normal_kd = find_value(config, "ur_normal_kd")  # type: float
        self.md_normal_kp = find_value(config, "md_normal_kp")  # type: float
        self.md_normal_ki = find_value(config, "md_normal_ki")  # type: float
        self.md_normal_kd = find_value(config, "md_normal_kd")  # type: float
        # 电机补偿系数
        self.compensate_param_ul_right = find_value(config, "compensate_param_ul_right")	# type: float
        self.compensate_param_ur_right = find_value(config, "compensate_param_ur_right")	# type: float
        self.compensate_param_md_right = find_value(config, "compensate_param_md_right")	# type: float
        self.compensate_param_ul_left = find_value(config, "compensate_param_ul_left")	# type: float
        self.compensate_param_ur_left = find_value(config, "compensate_param_ur_left")	# type: float
        self.compensate_param_md_left = find_value(config, "compensate_param_md_left")	# type: float
        self.compensate_param_ul_up = find_value(config, "compensate_param_ul_up")	# type: float
        self.compensate_param_ur_up = find_value(config, "compensate_param_ur_up")	# type: float
        self.compensate_param_md_up = find_value(config, "compensate_param_md_up")	# type: float
        self.compensate_param_ul_down = find_value(config, "compensate_param_ul_down")	# type: float
        self.compensate_param_ur_down = find_value(config, "compensate_param_ur_down")	# type: float
        self.compensate_param_md_down = find_value(config, "compensate_param_md_down")	# type: float


# 创建pid参数对象
pid_data = PID_data()

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
diff_filter_ul = SlipAveragingFilter(2)    # 滤波窗口为2个
diff_filter_ur = SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_md = SlipAveragingFilter(2)    # 滤波窗口为2个
diff_filter_gyroz = SlipAveragingFilter(6)  # 滤波窗口为5个

# 创建小车x和y方向上的速度的卡尔曼滤波器
speed_x_fil = KalmanFilter(P = 1.0, Q = 0.05, R = 1.0)
speed_y_fil = KalmanFilter(P = 1.0, Q = 0.05, R = 1.0)
speed_x_fil2 = SlipAveragingFilter(5)  

# 创建姿态数据对象
pose_data = ant_pose.PoseData(diff_filter_gyroz)


# 定义一个抽象类用于顶层设计
# 该类能够存储pid参数并计算得到当前应该输出的pwm值
class ControlPID:
    def compute_pid(self, target: int, actual: int) -> None:
        pass

# 速度环位置式PID
class SpeedPositionPID(ControlPID):
    def __init__(self, diff_filter: SlipAveragingFilter):
        self.kp = 0.0        # type: float
        self.ki = 0.0       # type: float
        self.kd = 0.0       # type: float
        self.target = 0     # type: int
        self.actual = 0     # type: int
        self.nowError = 0   # type: int
        self.preError = 0   # type: int
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.__integral_limitmax = find_value(config, "integral_limitmax")      # type: float
        self.__pwmout_limitmax = find_value(config, "pwmout_limitmax")          # type: float
        self.diff_filter = diff_filter
        self.__A = find_value(config, "A")      # type: float # 变速积分误差阈值上限
        self.__B = find_value(config, "B")      # type: float # 变速积分误差阈值下限
        self.__kp_mid = find_value(config, "kp_mid")  # type: float # 中等误差时的kp系数
        self.__kp_low = find_value(config, "kp_low")  # type: float # 低误差时的kp系数

    def set_pid_params(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute_pid(self, target: int, actual: int):
        self.target = target
        self.actual = actual
        self.preError = self.nowError
        self.nowError = self.target - self.actual
        self.integral += self.nowError

        abs_nowerror = abs(self.nowError)
        coefficient = 1.0   # type: float
        if self.__A == self.__B:
            # 避免除以0
            if (abs_nowerror > self.__A):
                coefficient = 0.0
                kp_coefficient = 1.0
            else:
                coefficient = 1.0
                kp_coefficient = self.__kp_low
        else:
            if abs_nowerror > self.__A:
                coefficient = 0.0
                kp_coefficient = 1.0
            elif abs_nowerror > self.__B:
                coefficient = (self.__A - abs_nowerror) / (self.__A - self.__B)
                # 根据误差动态调节kp系数
                kp_coefficient = self.__kp_mid
            else:
                coefficient = 1.0
                # 根据误差动态调节kp系数
                kp_coefficient = self.__kp_low

        # 根据误差大小调整积分项
        self.integral += coefficient * self.nowError

        # 积分项限幅
        self.integral = max(-self.__integral_limitmax, min(self.integral, self.__integral_limitmax))

        # 对微分项进行滑动平均滤波
        self.derivative = self.diff_filter.filtering(self.nowError - self.preError)

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError * kp_coefficient + self.ki * self.integral + self.kd * self.derivative
        
        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))


# 角度环PID
class AnglePositionPID(ControlPID):
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp        # type: float
        self.ki = ki        # type: float
        self.kd = kd        # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: float
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.__angle_integral_limitmax = find_value(config, "angle_integral_limitmax")      # type: float
        self.__pwmout_limitmax = find_value(config, "angle_pwmout_limitmax")    # type: float
        

    def compute_pid(self, target: float, actual: float):
        self.target = target
        self.actual = actual
        self.preError = self.nowError
        self.nowError = self.target - self.actual
        # 对误差限幅
        if self.nowError > 180:
            self.nowError -= 360
        elif self.nowError < -180:
            self.nowError += 360
            
        self.integral += self.nowError
        self.derivative = self.nowError - self.preError

        # 积分项限幅
        self.integral = max(-self.__angle_integral_limitmax, min(self.integral, self.__angle_integral_limitmax))

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError + self.ki * self.integral + self.kd * self.derivative

        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))

# 创建电机pid对象和角度pid对象
motor_ul_pid = SpeedPositionPID(diff_filter = diff_filter_ul)

motor_ur_pid = SpeedPositionPID(diff_filter = diff_filter_ur)

motor_md_pid = SpeedPositionPID(diff_filter = diff_filter_md)


angle_pid = AnglePositionPID(kp = find_value(config, "angle_normal_kp"), 
                            ki = find_value(config, "angle_normal_ki"), 
                            kd = find_value(config, "angle_normal_kd"))

# 创建MOTOR_CONTROLLER对象
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty = 0, invert = True)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = True)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty = 0, invert = False)

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
        # 小车坐标系下的当前速度
        self.car_speed_x = 0.0  # type: float
        self.car_speed_y = 0.0  # type: float
        self.car_speed_w = 0.0  # type: float
        # 小车在世界坐标系下的速度
        self.real_speed_x = 0.0  # type: float
        self.real_speed_y = 0.0  # type: float
        self.real_speed = 0.0    # type: float
        self.real_speed_w = 0.0  # type: float
        # 小车坐标系下的目标速度
        self.car_speed_x_target = 0.0  # type: float
        self.car_speed_y_target = 0.0  # type: float
        self.car_speed_w_target = 0.0  # type: float
        # 世界坐标系下的目标速度
        self.real_speed_x_target = 0.0  # type: float
        self.real_speed_y_target = 0.0  # type: float
        self.real_speed_w_target = 0.0  # type: float
        # 速度系数
        self.speed_conversion_gamma = find_value(config, "speed_conversion_gamma")   # 适当减小速度数量级
        self.gkd = find_value(config, "gkd")  # type: float  # 角速度补偿系数
        self.speed_fuse_ratio = find_value(config, "speed_fuse_ratio")  # type: float  # 编码器和陀螺仪融合系数
        # 依据角度的位置修正系数（常量）
        self.alpha_x_1 = find_value(config, "alpha_x_1")  # type: float
        self.alpha_y_1 = find_value(config, "alpha_y_1")  # type: float
        self.alpha_x_2 = find_value(config, "alpha_x_2")  # type: float
        self.alpha_y_2 = find_value(config, "alpha_y_2")  # type: float
        self.alpha_x_3 = find_value(config, "alpha_x_3")  # type: float
        self.alpha_y_3 = find_value(config, "alpha_y_3")  # type: float
        self.alpha_x_4 = find_value(config, "alpha_x_4")  # type: float
        self.alpha_y_4 = find_value(config, "alpha_y_4")  # type: float
        self.alpha_x_5 = find_value(config, "alpha_x_5")  # type: float
        self.alpha_y_5 = find_value(config, "alpha_y_5")  # type: float
        self.alpha_x_6 = find_value(config, "alpha_x_6")  # type: float
        self.alpha_y_6 = find_value(config, "alpha_y_6")  # type: float
        self.alpha_x_7 = find_value(config, "alpha_x_7")  # type: float
        self.alpha_y_7 = find_value(config, "alpha_y_7")  # type: float
        self.alpha_x_8 = find_value(config, "alpha_x_8")  # type: float
        self.alpha_y_8 = find_value(config, "alpha_y_8")  # type: float
        # 依据角度的位置修正系数（常量）
        self.alpha_x = 1.0  # type: float
        self.alpha_y = 1.0  # type: float

        self.beta_x = 1.0  # type: float
        self.beta_y = 1.0  # type: float
        self.beta_z = 1.0  # type: float
        # 电机补偿参数
        self.compensate_param_ul = 1.0  # type: float
        self.compensate_param_ur = 1.0  # type: float
        self.compensate_param_md = 1.0  # type: float
        # 位置
        self.x_current = 0.0   # type: float
        self.y_current = 0.0   # type: float
        self.now_yaw = 0.0  # type: float
        # 位置转换系数
        self.position_conversion_gamma = find_value(config, "position_conversion_gamma")   # 适当减小位置数量级
        # 采集周期
        self.collect_dt = find_value(config, "collect_dt")     # type: float  # 单位：秒

    # 小车姿态更新
    def update_pose(self):
        ###################【速度计算】###################
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        self.last_car_speed_w = self.car_speed_w
        # 计算小车当前x,y速度（互补融合）
        # car_speed_x, car_speed_y
        self.car_speed_x = self.speed_fuse_ratio * self.last_car_speed_x + (1 - self.speed_fuse_ratio) * ((MATH.SIN30 * (pose_data.encoder_data_ur + pose_data.encoder_data_ul) - pose_data.encoder_data_md) * self.speed_conversion_gamma)
        self.car_speed_y = self.speed_fuse_ratio * self.last_car_speed_y + (1 - self.speed_fuse_ratio) * ((MATH.COS30 * (pose_data.encoder_data_ul - pose_data.encoder_data_ur)) * self.speed_conversion_gamma)
        # 对小车x,y速度卡尔曼滤波
        self.car_speed_x = speed_x_fil.update(self.car_speed_x)
        self.car_speed_y = speed_y_fil.update(self.car_speed_y)
        # 计算小车当r_correct_y = plan_data.error_correct_y_50_1前角速度
        # car_speed_w单位：度每秒
        self.car_speed_w = pose_data.gyro_z
        # 计算小车在世界坐标系下的偏航角
        # now_yaw单位：弧度
        self.now_yaw += pose_data.gyro_z * self.collect_dt * MATH.PI / 180
        # 限定now_yaw在-180到180度之间
        if self.now_yaw > MATH.PI:  self.now_yaw -= 2 * MATH.PI
        elif self.now_yaw < -MATH.PI:  self.now_yaw += 2 * MATH.PI
        # 转换到世界坐标系下的速度
        self.real_speed_x = self.car_speed_x * math.cos(self.now_yaw) + self.car_speed_y * math.sin(self.now_yaw)
        self.real_speed_y = -self.car_speed_x * math.sin(self.now_yaw) + self.car_speed_y * math.cos(self.now_yaw)
        self.real_speed = math.sqrt(self.real_speed_x ** 2 + self.real_speed_y ** 2)
        self.real_speed_w = self.car_speed_w
    
        
        ###################【位置计算】###################
        # 依据当前航向角调整位置修正系数（解决小车在不同方向上的编码器积分结果不一致问题）
        if ant_plan.my_plan.target_yaw >= -30.0 and ant_plan.my_plan.target_yaw < 30.0:
            self.alpha_x = self.alpha_x_1
            self.alpha_y = self.alpha_y_1
        elif ant_plan.my_plan.target_yaw >= 30.0 and ant_plan.my_plan.target_yaw < 60.0:
            self.alpha_x = self.alpha_x_2
            self.alpha_y = self.alpha_y_2
        elif ant_plan.my_plan.target_yaw >= 60.0 and ant_plan.my_plan.target_yaw < 120.0:
            self.alpha_x = self.alpha_x_3
            self.alpha_y = self.alpha_y_3
        elif ant_plan.my_plan.target_yaw >= 120.0 and ant_plan.my_plan.target_yaw < 150.0:
            self.alpha_x = self.alpha_x_4
            self.alpha_y = self.alpha_y_4
        elif ant_plan.my_plan.target_yaw >= 150.0 and ant_plan.my_plan.target_yaw <= 180.0 or ant_plan.my_plan.target_yaw >= -180.0 and ant_plan.my_plan.target_yaw < -150.0:
            self.alpha_x = self.alpha_x_5
            self.alpha_y = self.alpha_y_5
        elif ant_plan.my_plan.target_yaw >= -150.0 and ant_plan.my_plan.target_yaw < -120.0:
            self.alpha_x = self.alpha_x_6
            self.alpha_y = self.alpha_y_6
        elif ant_plan.my_plan.target_yaw >= -120.0 and ant_plan.my_plan.target_yaw < -60.0:
            self.alpha_x = self.alpha_x_7
            self.alpha_y = self.alpha_y_7
        elif ant_plan.my_plan.target_yaw >= -60.0 and ant_plan.my_plan.target_yaw < -30.0:
            self.alpha_x = self.alpha_x_8
            self.alpha_y = self.alpha_y_8

        # 计算小车当前位置
        self.x_current += self.real_speed_x * self.position_conversion_gamma * self.alpha_x 
        self.y_current += self.real_speed_y * self.position_conversion_gamma * self.alpha_y

    # 全向移动控制函数
    # 参数说明：move_speed_target单位：编码器脉冲， move_angle_target单位：度， turn_angle_target单位：度
    def move_ctrl(self, move_speed_target: int, move_angle_target: float, turn_angle_target: float):
       # 将目标转角和目标航向角限定在-180到180度之间
        if turn_angle_target > 180.0:
            turn_angle_target -= 360.0        
        elif turn_angle_target < -180.0:   
            turn_angle_target += 360.0
        
        if move_angle_target > 180.0:
            move_angle_target -= 360.0
        elif move_angle_target < -180.0:
            move_angle_target += 360.0

        # 计算z轴的目标速度
        angle_pid.compute_pid(turn_angle_target, self.now_yaw * 180 / MATH.PI)
        speed_w = angle_pid.pwm_output

        # 选择合适的电机补偿参数
        if move_angle_target >= 45.0 and move_angle_target < 135.0:
            self.compensate_param_ul = pid_data.compensate_param_ul_right
            self.compensate_param_ur = pid_data.compensate_param_ur_right
            self.compensate_param_md = pid_data.compensate_param_md_right
        elif move_angle_target >= 135.0 or move_angle_target < -135.0:
            self.compensate_param_ul = pid_data.compensate_param_ul_down
            self.compensate_param_ur = pid_data.compensate_param_ur_down
            self.compensate_param_md = pid_data.compensate_param_md_down
        elif move_angle_target >= -135.0 and move_angle_target < -45.0:
            self.compensate_param_ul = pid_data.compensate_param_ul_left
            self.compensate_param_ur = pid_data.compensate_param_ur_left
            self.compensate_param_md = pid_data.compensate_param_md_left
        else:
            self.compensate_param_ul = pid_data.compensate_param_ul_up
            self.compensate_param_ur = pid_data.compensate_param_ur_up
            self.compensate_param_md = pid_data.compensate_param_md_up

        # 将move_angle_target转换为弧度
        move_angle_target = move_angle_target * MATH.PI / 180
        
        # 设置小车在世界坐标系下的目标速度
        self.real_speed_w_target = speed_w
        self.real_speed_x_target = move_speed_target * math.sin(move_angle_target)
        self.real_speed_y_target = move_speed_target * math.cos(move_angle_target)

        # 转换到小车坐标系下的目标速度
        self.car_speed_x_target = move_speed_target * math.sin(move_angle_target - self.now_yaw)
        self.car_speed_y_target = move_speed_target * math.cos(move_angle_target - self.now_yaw)
        self.car_speed_w_target = self.real_speed_w_target

        # 计算各个电机的目标速度
        motor_ul_speed_target = ((self.car_speed_w_target + self.car_speed_x_target) * MATH.OneThird + self.car_speed_y_target / MATH.SQRT3 + pose_data.gyro_z * self.gkd) * self.compensate_param_ul
        motor_ur_speed_target = ((self.car_speed_w_target + self.car_speed_x_target) * MATH.OneThird - self.car_speed_y_target / MATH.SQRT3 + pose_data.gyro_z * self.gkd) * self.compensate_param_ur
        motor_md_speed_target = (self.car_speed_w_target * MATH.OneThird - self.car_speed_x_target * MATH.TwoThirdS + pose_data.gyro_z * self.gkd) * self.compensate_param_md

        # 计算各个电机的pid得到pwm输出
        motor_ul_pid.compute_pid(int(motor_ul_speed_target), pose_data.encoder_data_ul)
        motor_ur_pid.compute_pid(int(motor_ur_speed_target), pose_data.encoder_data_ur)
        motor_md_pid.compute_pid(int(motor_md_speed_target), pose_data.encoder_data_md)

    # 设置电机pwm输出函数
    def set_motor_pwm(self):
        motor_ul.duty(int(motor_ul_pid.pwm_output))
        motor_ur.duty(int(motor_ur_pid.pwm_output))
        motor_md.duty(int(motor_md_pid.pwm_output))


# 创建小车姿态对象
my_car = CarPose()

# 调试电机速度环pid函数
def show_speed_PID_test():
    motor_ul_pid.compute_pid(60, pose_data.encoder_data_ul)
    motor_ur_pid.compute_pid(60, pose_data.encoder_data_ur)
    motor_md_pid.compute_pid(60, pose_data.encoder_data_md)
    
# 测试陀螺仪函数
def test_imu():
    ant_uart.wireless.send_str("{:<f},{:<f},{:<f}\n".format(pose_data.gyro_z, ant_pose.imu_data[5], pose_data.gyro_z_bias))           
    
# 测试角度闭环函数
def complete_angle_circle():
    my_car.update_pose()
    my_car.move_ctrl(0, 0, 0)
    #ant_uart.wireless.send_str("{:<f},{:<f}\n".format(angle_pid.target, angle_pid.actual))
    
# 全向移动转圈测试函数
target_yaw = 0
def all_around_circle():
    global target_yaw
    target_yaw += 1
    if target_yaw >= 180:
        target_yaw = -180
    my_car.move_ctrl(250, target_yaw, 0)
    ant_uart.wireless.send_str("{:<f},{:<f}\n".format(target_yaw, angle_pid.actual))


# 多路复用器（用于测试）
count = 0
def test_simble_displacement():
    global count
    count += 1
    if count <= 600:
        my_car.move_ctrl(400, 180, 0)
    else:
        my_car.move_ctrl(0, 90, 90)
        
# 里程计测试函数
stage = 0	# 当前模式
def test_odometer():
    global stage
    global count
    #ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.car_speed_x, my_car.y_current, my_car.car_speed_y))
    #ant_uart.wireless.send_str("{:<f},{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI, angle_pid.pwm_output))
    if count == 0:
        if my_car.x_current <= 12.4 and stage == 0:
            my_car.move_ctrl(50, 90, 0)
            return
        elif my_car.x_current >= 0.6 and stage == 1:
            my_car.move_ctrl(50, -90, 0)
            return
        elif my_car.x_current >= -99.0 and stage == 2:
            my_car.move_ctrl(0, 0, 0)
            return
        elif my_car.y_current >= 1.0 and stage == 3:
            my_car.move_ctrl(50, 180, 0)
            return
        elif stage == 4:
            my_car.move_ctrl(0, 0, 0)
            return
     
    my_car.move_ctrl(0, 0, 0)
    count += 1
    if count == 200:
        stage += 1
        count = 0
    
        
# 全向定位测试函数
def test_global_localization():
    #ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, ant_plan.my_plan.real_target_x, ant_plan.my_plan.real_target_y, ant_plan.my_plan.rest_distance, ant_plan.my_plan.target_yaw, my_car.now_yaw, ant_plan.my_plan.arrive_flag, ant_plan.my_plan.transition_flag))
    my_car.move_ctrl(ant_plan.my_plan.v_target, ant_plan.my_plan.target_yaw, ant_plan.my_plan.turn_angle_target)


# 定时器1中断回调函数
def time_pit1_handler(time):
    pose_data.update_data()
    # 初始化pid参数
    motor_ul_pid.set_pid_params(pid_data.ul_normal_kp, pid_data.ul_normal_ki, pid_data.ul_normal_kd)
    motor_ur_pid.set_pid_params(pid_data.ur_normal_kp, pid_data.ur_normal_ki, pid_data.ur_normal_kd)
    motor_md_pid.set_pid_params(pid_data.md_normal_kp, pid_data.md_normal_ki, pid_data.md_normal_kd)
    my_car.update_pose()
    
    #ant_uart.wireless.send_str("{:<f},{:<f},{:<f}\n".format(my_car.car_speed_x, speed_x_fil.update(my_car.car_speed_x), speed_x_fil2.filtering(my_car.car_speed_x)))
    
    # 全向移动转圈测试程序
    #all_around_circle()
    #ant_uart.wireless.send_str("{:<f},{:<f}\n".format(my_car.x_current, my_car.car_speed_x))
    
    # 里程计测试程序
    # test_odometer()
    
    # test_simble_displacement()
    
    # 测试角度闭环
    # complete_angle_circle()
    
    # 全向定位测试程序
    test_global_localization()
    
    #if my_car.x_current <= 8.4:
     #   my_car.move_ctrl(60, 90, 0)
    #else:
     #   my_car.move_ctrl(0, 90, 0)
    # 里程计测试
    #ant_uart.wireless.send_str("{:<f}\n".format(my_car.now_yaw))
    
    # 陀螺仪测试
    # test_imu()
    # ant_uart.wireless.send_str("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
    # 速度环测试
    # show_speed_PID_test()
    
    my_car.set_motor_pwm()




