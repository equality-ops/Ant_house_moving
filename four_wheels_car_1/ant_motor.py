from machine import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker, encoder
from seekfree import IMU660RX
import math
import ant_flash
from ant_flash import MATH as MATH
from ant_flash import find_aimed_value as find_value
import ant_else

###################################【文件读取】###################################
# 从config.txt中读取保存所有的参数并保存到config字典中
config = ant_flash.phase_config("/flash/config.txt")

# 读取完再导入
import ant_plan

# 编码器初始化
encoder_lf = encoder("C2" , "C3" , True)
encoder_rf = encoder("C0" , "C1" , True)
encoder_lb = encoder("D15", "D16", True)
encoder_rb = encoder("D13", "D14", True)

# IMU初始化
imu = IMU660RX()
# 定时器1采集已经与imu_data相连
imu_data = []   # type: list

class PID_data:
    def __init__(self):
        self.lf_normal_kp = find_value(config, "lf_normal_kp")  # type: float
        self.lf_normal_ki = find_value(config, "lf_normal_ki")  # type: float
        self.lf_normal_kd = find_value(config, "lf_normal_kd")  # type: float
        self.rf_normal_kp = find_value(config, "rf_normal_kp")  # type: float
        self.rf_normal_ki = find_value(config, "rf_normal_ki")  # type: float
        self.rf_normal_kd = find_value(config, "rf_normal_kd")  # type: float
        self.lb_normal_kp = find_value(config, "lb_normal_kp")  # type: float
        self.lb_normal_ki = find_value(config, "lb_normal_ki")  # type: float
        self.lb_normal_kd = find_value(config, "lb_normal_kd")  # type: float
        self.rf_normal_kd = find_value(config, "rf_normal_kd")  # type: float
        self.rb_normal_kp = find_value(config, "rb_normal_kp")  # type: float
        self.rb_normal_ki = find_value(config, "rb_normal_ki")  # type: float
        self.rb_normal_kd = find_value(config, "rb_normal_kd")  # type: float

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
diff_filter_lf = SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_rf = SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_lb = SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_rb = SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_gyroz = SlipAveragingFilter(6)  # 滤波窗口为6个

# 创建小车x和y方向上的速度的卡尔曼滤波器
speed_x_fil = KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
speed_y_fil = KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)

# 视觉伺服自身转角的卡尔曼滤波器
servo_turn_angle_fil = KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)

class PoseData:
    def __init__(self):
        self.encoder_data_lf = 0    # type: int
        self.encoder_data_rf = 0    # type: int
        self.encoder_data_lb = 0    # type: int
        self.encoder_data_rb = 0    # type: int
        self.gyro_z_bias = 0.0       # type: float
        self.gyro_z_supply = find_value(config, "gyro_z_supply")
        """暂时不需要这些数据
        self.acc_x = 0              # type: int
        self.acc_y = 0              # type: int
        self.acc_z = 0              # type: int
        self.gyro_x = 0             # type: int
        self.gyro_y = 0             # type: int
        self.gyro_z = 0             # type: float
        self.acc_x_bias = 0.0        # type: float
        self.acc_y_bias = 0.0        # type: float
        self.acc_z_bias = 0.0        # type: float
        self.gyro_x_bias = 0.0       # type: float
        self.gyro_y_bias = 0.0       # type: float
        """

    # 初始零偏计算函数
    def init_bias(self):
        """暂时不需要这些数据
        acc_x_sum = 0
        acc_y_sum = 0
        acc_z_sum = 0
        gyro_x_sum = 0
        gyro_y_sum = 0
        """

        gyro_z_sum = 0
        sample_count = 2000

        for i in range(sample_count):
            imu_data = imu.read()
            gyro_z_sum += imu_data[5]
            """暂时不需要处理这些数据
            acc_x_sum += imu_data[0]
            acc_y_sum += imu_data[1]
            acc_z_sum += imu_data[2]
            gyro_x_sum += imu_data[3]
            gyro_y_sum += imu_data[4]
            """
        """时不需要处理这些数据
        self.acc_x_bias = acc_x_sum / sample_count
        self.acc_y_bias = acc_y_sum / sample_count
        self.acc_z_bias = acc_z_sum / sample_count
        self.gyro_x_bias = gyro_x_sum / sample_count
        self.gyro_y_bias = gyro_y_sum / sample_count
        """
        self.gyro_z_bias = gyro_z_sum / sample_count

    # 传感器数据更新函数
    def update_data(self):
        self.encoder_data_lf = encoder_lf.get()
        self.encoder_data_rf = encoder_rf.get()
        self.encoder_data_lb = encoder_lb.get()
        self.encoder_data_rb = encoder_rb.get()
        """暂时不需要处理这些数据
        self.acc_x = imu_data[0] - self.acc_x_bias
        self.acc_y = imu_data[1] - self.acc_y_bias
        self.acc_z = imu_data[2] - self.acc_z_bias
        self.gyro_x = imu_data[3] - self.gyro_x_bias
        self.gyro_y = imu_data[4] - self.gyro_y_bias
        """
        # 去零漂后滑动平均滤波（单位：角度每秒）
        self.gyro_z = -diff_filter_gyroz.filtering(imu_data[5] - self.gyro_z_bias) / 16.4 * self.gyro_z_supply


# 创建姿态数据对象
pose_data = PoseData()

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


# 视觉伺服PD
class ServoPID(ControlPID):
    def __init__(self, kp: float, kd: float):
        self.kp = kp        # type: float
        self.kd = kd        # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: float
        self.nowError = 0   # type: int
        self.preError = 0   # type: int
        self.derivative = 0 # type: int
        self.pwm_output = 0 # type: int
        self.__pwmout_limitmax = find_value(config, "servo_pwmout_limitmax")    # type: int
        

    def compute_pid(self, target: int, actual: int):
        self.target = target
        self.actual = actual
        self.preError = self.nowError
        self.nowError = self.target - self.actual
        # 计算微分项
        self.derivative = self.nowError - self.preError

        # 计算pwm_output
        self.pwm_output = int(self.kp * self.nowError + self.kd * self.derivative)

        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))

        return self.pwm_output

# 创建电机pid对象和角度pid对象
motor_lf_pid = SpeedPositionPID(diff_filter = diff_filter_lf)

motor_rf_pid = SpeedPositionPID(diff_filter = diff_filter_rf)

motor_lb_pid = SpeedPositionPID(diff_filter = diff_filter_lb)

motor_rb_pid = SpeedPositionPID(diff_filter = diff_filter_rb)

angle_pid = AnglePositionPID(kp = find_value(config, "angle_normal_kp"), 
                            ki = find_value(config, "angle_normal_ki"), 
                            kd = find_value(config, "angle_normal_kd"))

servo_pid_x = ServoPID(kp = find_value(config, "servo_kp_x"), kd = find_value(config, "servo_kd_x"))
servo_pid_y = ServoPID(kp = find_value(config, "servo_kp_y"), kd = find_value(config, "servo_kd_y"))

# 创建MOTOR_CONTROLLER对象
motor_lf = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty = 0, invert = True)
motor_rf = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = True)
motor_lb = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty = 0, invert = False)
motor_rb = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = False)

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
        self.speed_conversion_gamma = find_value(config, "speed_conversion_gamma")   # 将速度单位转化为cm每秒
        self.gkd = find_value(config, "gkd")  # type: float  # 角速度补偿系数
        self.speed_fuse_ratio = find_value(config, "speed_fuse_ratio")  # type: float  # 速度融合系数
        # 依据角度的位置修正系数（常量）
        self.alpha_x = 1.0  # type: float
        self.alpha_y = 1.0  # type: float
        self.beta_x = 1.0  # type: float
        self.beta_y = 1.0  # type: float
        self.beta_z = 1.0  # type: float
        # 位置
        self.x_current = 0.0   # type: float
        self.y_current = 0.0   # type: float
        self.now_yaw = 0.0  # type: float
        # 采集周期，单位：秒
        self.collect_dt = find_value(config, "collect_dt")  # type: float  
        # 测试一个电机的里程
        # self.encouder_lf = 0.0
        # self.encouder_rf = 0.0
        # self.encouder_lb = 0.0
        
    # 小车姿态更新
    def update_pose(self):
        ###################【速度计算】###################
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        self.last_car_speed_w = self.car_speed_w
        # 测试一个电机的里程
        # self.encouder_lf += self.speed_conversion_gamma * pose_data.encoder_data_lf / 1000
        # self.encouder_rf += self.speed_conversion_gamma * pose_data.encoder_data_rf / 1000
        # self.encouder_lb += self.speed_conversion_gamma * pose_data.encoder_data_lb / 1000
        # 计算小车当前x,y速度（互补融合）
        # car_speed_x, car_speed_y 单位：厘米每2ms
        self.car_speed_x = self.speed_fuse_ratio * self.last_car_speed_x + (1 - self.speed_fuse_ratio) * (pose_data.encoder_data_lf + pose_data.encoder_data_rb - pose_data.encoder_data_rf - pose_data.encoder_data_lb) * 0.25 * self.speed_conversion_gamma
        self.car_speed_y = self.speed_fuse_ratio * self.last_car_speed_y + (1 - self.speed_fuse_ratio) * (pose_data.encoder_data_lf + pose_data.encoder_data_rb + pose_data.encoder_data_rf + pose_data.encoder_data_lb) * 0.25 * self.speed_conversion_gamma
        # 对小车x,y速度卡尔曼滤波
        self.car_speed_x = speed_x_fil.update(self.car_speed_x)
        self.car_speed_y = speed_y_fil.update(self.car_speed_y)
        #speed_x_fil.update(self.car_speed_x)
        #speed_y_fil.update(self.car_speed_y)
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
        self.real_speed_w = self.car_speed_w
    
        
        ###################【位置计算】###################
        # 依据当前航向角调整位置修正系数（解决小车在不同方向上的编码器积分结果不一致问题）

        # 计算小车当前位置
        self.x_current += self.real_speed_x
        self.y_current += self.real_speed_y


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
        motor_lf_speed_target = self.car_speed_x_target + self.car_speed_y_target + self.car_speed_w_target + pose_data.gyro_z * self.gkd
        motor_rf_speed_target = -self.car_speed_x_target + self.car_speed_y_target - self.car_speed_w_target + pose_data.gyro_z * self.gkd
        motor_lb_speed_target = -self.car_speed_x_target + self.car_speed_y_target + self.car_speed_w_target + pose_data.gyro_z * self.gkd
        motor_rb_speed_target = self.car_speed_x_target + self.car_speed_y_target - self.car_speed_w_target + pose_data.gyro_z * self.gkd

        # 计算各个电机的pid得到pwm输出
        motor_lf_pid.compute_pid(int(motor_lf_speed_target), pose_data.encoder_data_lf)
        motor_rf_pid.compute_pid(int(motor_rf_speed_target), pose_data.encoder_data_rf)
        motor_lb_pid.compute_pid(int(motor_lb_speed_target), pose_data.encoder_data_lb)
        motor_rb_pid.compute_pid(int(motor_rb_speed_target), pose_data.encoder_data_rb)

    # 设置电机pwm输出函数
    def set_motor_pwm(self):
        motor_lf.duty(int(motor_lf_pid.pwm_output))
        motor_rf.duty(int(motor_rf_pid.pwm_output))
        motor_lb.duty(int(motor_lb_pid.pwm_output))
        motor_rb.duty(int(motor_rb_pid.pwm_output))

# 创建小车姿态对象
my_car = CarPose()

# 调试电机速度环pid函数
def show_speed_PID_test():
    motor_lf_pid.compute_pid(60, pose_data.encoder_data_lf)
    motor_rf_pid.compute_pid(60, pose_data.encoder_data_rf)
    motor_lb_pid.compute_pid(60, pose_data.encoder_data_lb)
    
# 测试陀螺仪函数
def test_imu():
    ant_else.wireless.send_str("{:<f},{:<f},{:<f}\n".format(pose_data.gyro_z, imu_data[5], pose_data.gyro_z_bias))           
    
# 测试角度闭环函数
def complete_angle_circle():
    my_car.update_pose()
    my_car.move_ctrl(0, 0, 0)
    
# 全向移动转圈测试函数
target_yaw = 0
def all_around_circle():
    global target_yaw
    target_yaw += 0.1
    if target_yaw >= 180:
        target_yaw = -180
    my_car.move_ctrl(60, target_yaw, 0)


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
    if count == 0:
        if my_car.y_current <= 50.0 and stage == 0:
            my_car.move_ctrl(65, 45, 0)
            return
        elif my_car.x_current >= 0.6 and stage == 1:
            my_car.move_ctrl(0, 0, 0)
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
    #ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_crfrent, my_car.y_crfrent, ant_plan.my_plan.real_target_x, ant_plan.my_plan.real_target_y, ant_plan.my_plan.rest_distance, ant_plan.my_plan.target_yaw, my_car.now_yaw, ant_plan.my_plan.arrive_flag, ant_plan.my_plan.transition_flag))
    my_car.move_ctrl(ant_plan.my_plan.v_target, ant_plan.my_plan.target_yaw, ant_plan.my_plan.trfn_angle_target)

# 测试伺服控制函数
def test_servo_control():
    if ant_plan.my_state.state == ant_plan.my_state.NAVIGATE:
        my_car.move_ctrl(ant_plan.my_plan.v_target, ant_plan.my_plan.target_yaw, ant_plan.my_plan.trfn_angle_target)
    elif ant_plan.my_state.state == ant_plan.my_state.SERVO:
        my_car.move_ctrl(ant_plan.my_vision_manager_2.target_rel_speed, ant_plan.my_vision_manager_2.target_rel_yaw, ant_plan.my_vision_manager_2.target_rel_trfn_angle)
    elif ant_plan.my_state.state == ant_plan.my_state.STOP:
        my_car.move_ctrl(0, 0, 0)

# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    # 初始化pid参数
    motor_lf_pid.set_pid_params(pid_data.lf_normal_kp, pid_data.lf_normal_ki, pid_data.lf_normal_kd)
    motor_rf_pid.set_pid_params(pid_data.rf_normal_kp, pid_data.rf_normal_ki, pid_data.rf_normal_kd)
    motor_lb_pid.set_pid_params(pid_data.lb_normal_kp, pid_data.lb_normal_ki, pid_data.lb_normal_kd)
    motor_rb_pid.set_pid_params(pid_data.rb_normal_kp, pid_data.rb_normal_ki, pid_data.rb_normal_kd)
    
    # 更新小车姿态
    my_car.update_pose()
    
    # 全向移动转圈测试程序
    #all_around_circle()
    #ant_else.wireless.send_str("{:<f},{:<f}\n".format(my_car.x_crfrent, my_car.car_speed_x))
    
    # 里程计测试程序
    #test_odometer()
    
    # test_simble_displacement()
    
    # 测试角度闭环
    # complete_angle_circle()
    
    # 全向定位测试程序
    # test_global_localization()
    
    #if my_car.x_crfrent <= 8.4:
     #   my_car.move_ctrl(60, 90, 0)
    #else:
     #   my_car.move_ctrl(0, 90, 0)
    # 里程计测试
    #ant_else.wireless.send_str("{:<f}\n".format(my_car.now_yaw))
    
    # 陀螺仪测试
    # test_imu()
    # ant_else.wireless.send_str("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
    # 速度环测试
    # show_speed_PID_test()
    
    # 测试伺服控制函数
    test_servo_control()
    
    # 设置电机pwm输出
    my_car.set_motor_pwm()