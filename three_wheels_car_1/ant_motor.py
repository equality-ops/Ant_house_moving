import math

class PID_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys

        self.ul_high_kp = self.flash_sys.find_value("ul_high_kp")  # type: float
        self.ul_high_ki = self.flash_sys.find_value("ul_high_ki")  # type: float
        self.ul_high_kd = self.flash_sys.find_value("ul_high_kd")  # type: float
        self.ur_high_kp = self.flash_sys.find_value("ur_high_kp")  # type: float
        self.ur_high_ki = self.flash_sys.find_value("ur_high_ki")  # type: float
        self.ur_high_kd = self.flash_sys.find_value("ur_high_kd")  # type: float
        self.md_high_kp = self.flash_sys.find_value("md_high_kp")  # type: float
        self.md_high_ki = self.flash_sys.find_value("md_high_ki")  # type: float
        self.md_high_kd = self.flash_sys.find_value("md_high_kd")  # type: float

        self.ul_mid1_kp = self.flash_sys.find_value("ul_mid1_kp")  # type: float
        self.ul_mid1_ki = self.flash_sys.find_value("ul_mid1_ki")  # type: float
        self.ul_mid1_kd = self.flash_sys.find_value("ul_mid1_kd")  # type: float
        self.ur_mid1_kp = self.flash_sys.find_value("ur_mid1_kp")  # type: float
        self.ur_mid1_ki = self.flash_sys.find_value("ur_mid1_ki")  # type: float
        self.ur_mid1_kd = self.flash_sys.find_value("ur_mid1_kd")  # type: float
        self.md_mid1_kp = self.flash_sys.find_value("md_mid1_kp")  # type: float
        self.md_mid1_ki = self.flash_sys.find_value("md_mid1_ki")  # type: float
        self.md_mid1_kd = self.flash_sys.find_value("md_mid1_kd")  # type: float

        self.ul_mid2_kp = self.flash_sys.find_value("ul_mid2_kp")  # type: float
        self.ul_mid2_ki = self.flash_sys.find_value("ul_mid2_ki")  # type: float
        self.ul_mid2_kd = self.flash_sys.find_value("ul_mid2_kd")  # type: float
        self.ur_mid2_kp = self.flash_sys.find_value("ur_mid2_kp")  # type: float
        self.ur_mid2_ki = self.flash_sys.find_value("ur_mid2_ki")  # type: float
        self.ur_mid2_kd = self.flash_sys.find_value("ur_mid2_kd")  # type: float
        self.md_mid2_kp = self.flash_sys.find_value("md_mid2_kp")  # type: float
        self.md_mid2_ki = self.flash_sys.find_value("md_mid2_ki")  # type: float
        self.md_mid2_kd = self.flash_sys.find_value("md_mid2_kd")  # type: float
        
        self.ul_low_kp = self.flash_sys.find_value("ul_low_kp")  # type: float
        self.ul_low_ki = self.flash_sys.find_value("ul_low_ki")  # type: float
        self.ul_low_kd = self.flash_sys.find_value("ul_low_kd")  # type: float
        self.ur_low_kp = self.flash_sys.find_value("ur_low_kp")  # type: float
        self.ur_low_ki = self.flash_sys.find_value("ur_low_ki")  # type: float
        self.ur_low_kd = self.flash_sys.find_value("ur_low_kd")  # type: float
        self.md_low_kp = self.flash_sys.find_value("md_low_kp")  # type: float
        self.md_low_ki = self.flash_sys.find_value("md_low_ki")  # type: float
        self.md_low_kd = self.flash_sys.find_value("md_low_kd")  # type: float

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


class PoseData:
    def __init__(self, flash_sys, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz, encoder_ul_fil, encoder_ur_fil, encoder_md_fil):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入传感器对象
        self.imu = imu
        self.encoder_ul = encoder_ul
        self.encoder_ur = encoder_ur
        self.encoder_md = encoder_md
        # 注入滤波器对象
        self.diff_filter_gyroz = diff_filter_gyroz
        # 注入编码器卡尔曼滤波器对象
        self.encoder_ul_fil = encoder_ul_fil
        self.encoder_ur_fil = encoder_ur_fil
        self.encoder_md_fil = encoder_md_fil
        # IMU数据列表
        self.imu_data = []   # type: list

        self.encoder_data_ul = 0    # type: int
        self.encoder_data_ur = 0    # type: int
        self.encoder_data_md = 0    # type: int
        # 测试
        # self.encoder_data_ul_2 = 0    # type: int
        # self.encoder_data_ur_2 = 0    # type: int
        # self.encoder_data_md_2 = 0    # type: int
        self.gyro_z_bias = 0.0       # type: float
        self.gyro_z_supply = self.flash_sys.find_value("gyro_z_supply")
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
        sample_count = 10000
        # 将imu_data与imu对象链接起来
        self.imu_data = self.imu.get()
        for i in range(sample_count):
            self.imu_data = self.imu.read()
            gyro_z_sum += self.imu_data[5]
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
        self.encoder_data_ul = self.encoder_ul.get()
        self.encoder_data_ur = self.encoder_ur.get()
        self.encoder_data_md = self.encoder_md.get()
        # 对编码器数据进行卡尔曼滤波
        self.encoder_data_ul = int(self.encoder_ul_fil.update(self.encoder_data_ul))
        self.encoder_data_ur = int(self.encoder_ur_fil.update(self.encoder_data_ur))
        self.encoder_data_md = int(self.encoder_md_fil.update(self.encoder_data_md))
        # 测试
        # self.encoder_data_ul_2 = int(self.encoder_ul_fil.update(self.encoder_data_ul))
        # self.encoder_data_ur_2 = int(self.encoder_ur_fil.update(self.encoder_data_ur))
        # self.encoder_data_md_2 = int(self.encoder_md_fil.update(self.encoder_data_md))
        """暂时不需要处理这些数据
        self.acc_x = imu_data[0] - self.acc_x_bias
        self.acc_y = imu_data[1] - self.acc_y_bias
        self.acc_z = imu_data[2] - self.acc_z_bias
        self.gyro_x = imu_data[3] - self.gyro_x_bias
        self.gyro_y = imu_data[4] - self.gyro_y_bias
        """
        # 去零漂后滑动平均滤波（单位：角度每秒）
        self.gyro_z = -self.diff_filter_gyroz.filtering(self.imu_data[5] - self.gyro_z_bias) / 16.4 * self.gyro_z_supply



# 定义一个抽象类用于顶层设计
# 该类能够存储pid参数并计算得到当前应该输出的pwm值
class ControlPID:
    def compute_pid(self, target: int, actual: int) -> None:
        pass

# 速度环位置式PID
class SpeedPositionPID(ControlPID):
    def __init__(self, flash_sys, diff_filter: SlipAveragingFilter):
        # 注入flash系统对象
        self.flash_sys = flash_sys
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
        self.__integral_limitmax = self.flash_sys.find_value("integral_limitmax")      # type: float
        self.__pwmout_limitmax = self.flash_sys.find_value("pwmout_limitmax")          # type: float
        # 注入微分项滤波器对象
        self.diff_filter = diff_filter
        self.__A = self.flash_sys.find_value("A")      # type: float # 变速积分误差阈值上限
        self.__B = self.flash_sys.find_value("B")      # type: float # 变速积分误差阈值下限
        self.__kp_mid = self.flash_sys.find_value("kp_mid")  # type: float # 中等误差时的kp系数
        self.__kp_low = self.flash_sys.find_value("kp_low")  # type: float # 低误差时的kp系数

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
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.kp = self.flash_sys.find_value("angle_normal_kp")        # type: float
        self.ki = self.flash_sys.find_value("angle_normal_ki")        # type: float
        self.kd = self.flash_sys.find_value("angle_normal_kd")        # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: float
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.__angle_integral_limitmax = self.flash_sys.find_value("angle_integral_limitmax")      # type: float
        self.__pwmout_limitmax = self.flash_sys.find_value("angle_pwmout_limitmax")    # type: float
        

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
    def __init__(self, flash_sys, kp: float = 0.0, kd: float = 0.0):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.kp = kp        # type: float
        self.kd = kd        # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: float
        self.nowError = 0   # type: int
        self.preError = 0   # type: int
        self.derivative = 0 # type: int
        self.pwm_output = 0 # type: int
        self.__pwmout_limitmax = self.flash_sys.find_value("servo_pwmout_limitmax")    # type: int
        

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


# 小车姿态控制
class CarPose:
    def __init__(self, flash_sys, pose_data: PoseData, math, speed_x_fil: KalmanFilter, speed_y_fil: KalmanFilter, angle_pid: AnglePositionPID,
                 motor_ul_pid: SpeedPositionPID, motor_ur_pid: SpeedPositionPID, motor_md_pid: SpeedPositionPID, motor_ul, motor_ur, motor_md):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入姿态数据对象
        self.pose_data = pose_data
        # 注入速度卡尔曼滤波器对象
        self.speed_x_fil = speed_x_fil
        self.speed_y_fil = speed_y_fil
        # 注入数学常量对象
        self.MATH = math
        # 注入角度pid对象
        self.angle_pid = angle_pid
        # 注入电机pid对象
        self.motor_ul_pid = motor_ul_pid
        self.motor_ur_pid = motor_ur_pid
        self.motor_md_pid = motor_md_pid
        # 注入电机对象
        self.motor_ul = motor_ul
        self.motor_ur = motor_ur
        self.motor_md = motor_md

        # 机械参数
        self.wheel_radius = self.flash_sys.find_value("wheel_radius")  # type: float  # 轮半径，单位：cm
        self.car_radius = self.flash_sys.find_value("car_radius")          # type: float  # 车体半径，单位：cm
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
        self.speed_conversion_gamma = self.flash_sys.find_value("speed_conversion_gamma")   # 将速度单位转化为cm每秒
        self.gkd = self.flash_sys.find_value("gkd")  # type: float  # 角速度补偿系数
        self.speed_fuse_ratio = self.flash_sys.find_value("speed_fuse_ratio")  # type: float  # 速度融合系数
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
        self.collect_dt = self.flash_sys.find_value("collect_dt")  # type: float  
        # 测试一个电机的里程
        self.encouder_ul = 0.0
        self.encouder_ur = 0.0
        self.encouder_md = 0.0
        
    # 小车姿态更新
    def update_pose(self):
        ###################【速度计算】###################
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        self.last_car_speed_w = self.car_speed_w
        # 测试一个电机的里程
        self.encouder_ul += self.speed_conversion_gamma * self.pose_data.encoder_data_ul / 1000
        self.encouder_ur += self.speed_conversion_gamma * self.pose_data.encoder_data_ur / 1000
        self.encouder_md += self.speed_conversion_gamma * self.pose_data.encoder_data_md / 1000
        # 计算小车当前x,y速度（互补融合）
        # car_speed_x, car_speed_y 单位：厘米每2ms
        self.car_speed_x = self.speed_fuse_ratio * self.last_car_speed_x + (1 - self.speed_fuse_ratio) * (self.MATH.OneThird * (self.pose_data.encoder_data_ur + self.pose_data.encoder_data_ul - self.pose_data.encoder_data_md * 2)  * self.speed_conversion_gamma / 1000)
        self.car_speed_y = self.speed_fuse_ratio * self.last_car_speed_y + (1 - self.speed_fuse_ratio) * ((self.MATH.OneThird * self.MATH.SQRT3 * (self.pose_data.encoder_data_ul - self.pose_data.encoder_data_ur)) * self.speed_conversion_gamma / 1000)
        # 对小车x,y速度卡尔曼滤波
        self.car_speed_x = self.speed_x_fil.update(self.car_speed_x)
        self.car_speed_y = self.speed_y_fil.update(self.car_speed_y)
        #speed_x_fil.update(self.car_speed_x)
        #speed_y_fil.update(self.car_speed_y)
        # car_speed_w单位：度每秒
        self.car_speed_w = self.pose_data.gyro_z
        # 计算小车在世界坐标系下的偏航角
        # now_yaw单位：弧度
        self.now_yaw += self.pose_data.gyro_z * self.collect_dt * self.MATH.PI / 180
        # 限定now_yaw在-180到180度之间
        if self.now_yaw > self.MATH.PI:  self.now_yaw -= 2 * self.MATH.PI
        elif self.now_yaw < -self.MATH.PI:  self.now_yaw += 2 * self.MATH.PI
        # 转换到世界坐标系下的速度
        self.real_speed_x = self.car_speed_x * math.cos(self.now_yaw) + self.car_speed_y * math.sin(self.now_yaw)
        self.real_speed_y = -self.car_speed_x * math.sin(self.now_yaw) + self.car_speed_y * math.cos(self.now_yaw)
        self.real_speed_w = self.car_speed_w
    
        
        ###################【位置计算】###################
        # 依据当前航向角调整位置修正系数（解决小车在不同方向上的编码器积分结果不一致问题）

        # 计算小车当前位置
        # 测试
        # self.x_current += self.real_speed_x * 0.899303  巡航速度为330  沿x轴正方向
        # self.y_current += self.real_speed_y * 0.928489  巡航速度为330  沿y轴正方向
        # self.y_current += self.real_speed_y * 0.932707  巡航速度为330  沿y轴负方向
        self.x_current += self.real_speed_x * 0.899303
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
        self.angle_pid.compute_pid(turn_angle_target, self.now_yaw * 180 / self.MATH.PI)
        speed_w = self.angle_pid.pwm_output

        # 将move_angle_target转换为弧度
        move_angle_target = move_angle_target * self.MATH.PI / 180
        
        # 设置小车在世界坐标系下的目标速度
        self.real_speed_w_target = speed_w
        self.real_speed_x_target = move_speed_target * math.sin(move_angle_target)
        self.real_speed_y_target = move_speed_target * math.cos(move_angle_target)

        # 转换到小车坐标系下的目标速度
        self.car_speed_x_target = move_speed_target * math.sin(move_angle_target - self.now_yaw)
        self.car_speed_y_target = move_speed_target * math.cos(move_angle_target - self.now_yaw)
        self.car_speed_w_target = self.real_speed_w_target

        # 计算各个电机的目标速度
        motor_ul_speed_target = self.car_speed_w_target * self.MATH.OneThird + (self.car_speed_x_target + self.car_speed_y_target * self.MATH.SQRT3) * 0.5 + self.pose_data.gyro_z * self.gkd
        motor_ur_speed_target = self.car_speed_w_target * self.MATH.OneThird + (self.car_speed_x_target - self.car_speed_y_target * self.MATH.SQRT3) * 0.5 + self.pose_data.gyro_z * self.gkd
        motor_md_speed_target = self.car_speed_w_target * self.MATH.OneThird - self.car_speed_x_target + self.pose_data.gyro_z * self.gkd

        # 计算各个电机的pid得到pwm输出
        self.motor_ul_pid.compute_pid(int(motor_ul_speed_target), self.pose_data.encoder_data_ul)
        self.motor_ur_pid.compute_pid(int(motor_ur_speed_target), self.pose_data.encoder_data_ur)
        self.motor_md_pid.compute_pid(int(motor_md_speed_target), self.pose_data.encoder_data_md)

    # 设置电机pwm输出函数
    def set_motor_pwm(self):
        self.motor_ul.duty(int(self.motor_ul_pid.pwm_output))
        self.motor_ur.duty(int(self.motor_ur_pid.pwm_output))
        self.motor_md.duty(int(self.motor_md_pid.pwm_output))
