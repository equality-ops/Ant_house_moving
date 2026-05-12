import math
import time

class PID_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 搬运过程中的pid参数
        self.ul_move_kp = self.flash_sys.find_value("ul_move_kp")  # type: float
        self.ul_move_ki = self.flash_sys.find_value("ul_move_ki")  # type: float
        self.ul_move_kd = self.flash_sys.find_value("ul_move_kd")  # type: float
        self.ur_move_kp = self.flash_sys.find_value("ur_move_kp")  # type: float
        self.ur_move_ki = self.flash_sys.find_value("ur_move_ki")  # type: float
        self.ur_move_kd = self.flash_sys.find_value("ur_move_kd")  # type: float
        self.md_move_kp = self.flash_sys.find_value("md_move_kp")  # type: float
        self.md_move_ki = self.flash_sys.find_value("md_move_ki")  # type: float
        self.md_move_kd = self.flash_sys.find_value("md_move_kd")  # type: float

        self.ul_high_kp = self.flash_sys.find_value("ul_high_kp")  # type: float
        self.ul_high_ki = self.flash_sys.find_value("ul_high_ki")  # type: float
        self.ul_high_kd = self.flash_sys.find_value("ul_high_kd")  # type: float
        self.ur_high_kp = self.flash_sys.find_value("ur_high_kp")  # type: float
        self.ur_high_ki = self.flash_sys.find_value("ur_high_ki")  # type: float
        self.ur_high_kd = self.flash_sys.find_value("ur_high_kd")  # type: float
        self.md_high_kp = self.flash_sys.find_value("md_high_kp")  # type: float
        self.md_high_ki = self.flash_sys.find_value("md_high_ki")  # type: float
        self.md_high_kd = self.flash_sys.find_value("md_high_kd")  # type: float

        self.ul_mid_kp = self.flash_sys.find_value("ul_mid_kp")  # type: float
        self.ul_mid_ki = self.flash_sys.find_value("ul_mid_ki")  # type: float
        self.ul_mid_kd = self.flash_sys.find_value("ul_mid_kd")  # type: float
        self.ur_mid_kp = self.flash_sys.find_value("ur_mid_kp")  # type: float
        self.ur_mid_ki = self.flash_sys.find_value("ur_mid_ki")  # type: float
        self.ur_mid_kd = self.flash_sys.find_value("ur_mid_kd")  # type: float
        self.md_mid_kp = self.flash_sys.find_value("md_mid_kp")  # type: float
        self.md_mid_ki = self.flash_sys.find_value("md_mid_ki")  # type: float
        self.md_mid_kd = self.flash_sys.find_value("md_mid_kd")  # type: float
        
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
        self.last_value = 0.0
        self.buffer = [0] * filter_size

    def buffer_init(self, initial_value):
        self.buffer = [initial_value] * self.filter_size

    # 滤波时传入一个新的数据，返回滤波后的结果(float)
    def filtering(self, data: int) -> float:
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.filter_size
        return sum(self.buffer) / self.filter_size
    
    # 用于处理小车自转角数据的特殊滑动平均滤波，能够处理跨越180度时的跳变问题
    def car_yaw_filtering(self, data) -> float:
        self.last_value = self.buffer[self.index]
        if data - self.last_value > 180:  # 设定一个阈值，单位：角度每秒
            data -= 360
        elif data - self.last_value < -180:
            data += 360
        self.index = (self.index + 1) % self.filter_size
        self.buffer[self.index] = data
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

# TOF数据滤波器
class ToFFilter:
    def __init__(self, window_size=5, alpha=0.3):
        self.buffer = []
        self.window_size = window_size
        self.alpha = alpha
        self.last_val = 0

    def update(self, raw_val):
        # 1. 处理错误值 (假设0是错误代码)
        if raw_val <= 0:
            return self.last_val
            
        # 2. 中值滤波 (取最近5个点排序取中点)
        self.buffer.append(raw_val)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        
        sorted_buf = sorted(self.buffer)
        median = sorted_buf[len(sorted_buf) // 2]
        
        # 3. 一阶IIR滤波 (平滑处理)
        filtered = self.alpha * median + (1 - self.alpha) * self.last_val
        self.last_val = filtered
        
        return filtered
    
    
class PoseData:
    def __init__(self, flash_sys, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入传感器对象
        self.imu = imu
        self.encoder_ul = encoder_ul
        self.encoder_ur = encoder_ur
        self.encoder_md = encoder_md
        # 注入滤波器对象
        self.diff_filter_gyroz = diff_filter_gyroz
        # IMU数据列表
        self.imu_data = []   # type: list

        # 传感器数据
        self.encoder_data_ul = 0    # type: int
        self.encoder_data_ur = 0    # type: int
        self.encoder_data_md = 0    # type: int
        
        # 陀螺仪补偿系数
        self.gyro_z_supply = self.flash_sys.find_value("gyro_y_supply")
        # 加速度
        self.acc_x = 0              # type: float
        self.acc_y = 0              # type: float
        self.acc_z = 0              # type: float
        # 角速度
        self.gyro_x = 0             # type: float
        self.gyro_y = 0             # type: float
        self.gyro_z = 0             # type: float
        # 角速度零漂误差
        self.gyro_x_bias = 0.0       # type: float
        self.gyro_y_bias = 0.0       # type: float
        self.gyro_z_bias = 0.0        # type: float

        # 四元数初始化
        self.q = [1.0, 0.0, 0.0, 0.0]
        # 误差积分项
        self.e_int = [0.0, 0.0, 0.0]
        
        # 算法参数 (根据你的 2ms 采样周期设置)
        self.dt = 0.002 
        self.kp = 10.0  # 加速度计权重
        self.ki = 0.001 # 零偏补偿权重

        # 最终角度输出
        self.now_pitch = 0.0  # 俯仰角
        self.now_roll = 0.0   # 横滚角
        self.now_yaw = 0.0    # 偏航角

    # 更新四元数
    def ahrs_update(self, ax, ay, az, gx, gy, gz):
        """
        核心四元数更新算法
        输入单位：ax-az (g), gx-gz (rad/s)
        """
        q0, q1, q2, q3 = self.q
        
        # 1. 归一化加速度计数据
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0: return # 防止除以0
        ax /= norm; ay /= norm; az /= norm
        
        # 2. 提取四元数矩阵中的理论重力方向 (机体坐标系下)
        vx = 2 * (q1*q3 - q0*q2)
        vy = 2 * (q0*q1 + q2*q3)
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3
        
        # 3. 叉乘计算误差 (测量值与理论值的偏差)
        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)
        
        # --- 改进1：增加积分限幅 (Anti-Windup) ---
        I_LIMIT = 0.2  # 限制积分项最大影响
        self.e_int[0] = max(-I_LIMIT, min(self.e_int[0] + ex * self.ki, I_LIMIT))
        self.e_int[1] = max(-I_LIMIT, min(self.e_int[1] + ey * self.ki, I_LIMIT))
        self.e_int[2] = 0.0 # 6轴系统，不要信任加速度计对 Yaw 的积分修正

        # --- 改进2：补偿角速度 ---
        gx += self.kp * ex + self.e_int[0]
        gy += self.kp * ey + self.e_int[1]
        gz += self.kp * 0  + self.e_int[2] # Yaw只信任陀螺仪
        
        # 6. 一阶龙格库塔法更新四元数
        half_dt = 0.5 * self.dt
        q0 += (-q1*gx - q2*gy - q3*gz) * half_dt
        q1 += (q0*gx + q2*gz - q3*gy) * half_dt
        q2 += (q0*gy - q1*gz + q3*gx) * half_dt
        q3 += (q0*gz + q1*gy - q2*gx) * half_dt
        
        # 7. 再次归一化四元数
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = [q0/norm, q1/norm, q2/norm, q3/norm]
    
    # 将四元数转化为欧拉角
    def update_euler_angles(self):
        """将四元数转换为欧拉角（度）"""
        q0, q1, q2, q3 = self.q
        
        # 1. 俯仰角 Pitch (绕 Y 轴旋转)
        # 数学逻辑：asin(-2*(q1*q3 - q0*q2))
        val = -2.0 * (q1 * q3 - q0 * q2)
        # 边界处理，防止 asin 超域报错
        val = max(-1.0, min(1.0, val))
        self.now_pitch = math.asin(val) * (180.0 / math.pi)
        
        # 2. 横滚角 Roll (绕 X 轴旋转)
        # 数学逻辑：atan2(2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2)
        self.now_roll = math.atan2(2.0 * (q2 * q3 + q0 * q1), 
                                   q0*q0 - q1*q1 - q2*q2 + q3*q3) * (180.0 / math.pi)
        
        # 3. 偏航角 Yaw (绕 Z 轴旋转)
        self.now_yaw = math.atan2(2.0 * (q1 * q2 + q0 * q3), 
                                  q0*q0 + q1*q1 - q2*q2 - q3*q3) * (180.0 / math.pi)

    # 重置四元数
    def reset_yaw(self, ref_yaw_deg):
        """
        通过外部参考信息强制重置当前的偏航角 (Yaw)。
        保留当前的横滚角 (Roll) 和俯仰角 (Pitch)，重新合成四元数。
        
        :param ref_yaw_deg: 外部传感器获取的绝对偏航角，单位：度 (°)
        """
        # 1. 将角度转换为半角弧度
        half_roll = self.now_roll * 0.5 * (math.pi / 180.0)
        half_pitch = self.now_pitch * 0.5 * (math.pi / 180.0)
        half_yaw = -ref_yaw_deg * 0.5 * (math.pi / 180.0)

        # 2. 预计算三角函数以提高运算效率
        sr = math.sin(half_roll)
        cr = math.cos(half_roll)
        sp = math.sin(half_pitch)
        cp = math.cos(half_pitch)
        sy = math.sin(half_yaw)
        cy = math.cos(half_yaw)

        # 3. 欧拉角转四元数 (基于你原始解算的 Z-Y-X 旋转顺序)
        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        # 为了确保精度，再次对生成的四元数进行归一化
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if norm == 0:
            return # 防止除零异常

        # 4. 强制覆盖当前四元数状态
        self.q = [q0/norm, q1/norm, q2/norm, q3/norm]

        # 5. 清空 PI 算法的积分补偿项
        # 这一步极其重要：如果不清空，历史误差的积分积累会在接下来的几个周期内把姿态又“拉回”一点点，导致修正不干脆
        self.e_int = [0.0, 0.0, 0.0]

        # 6. 同步更新底层的欧拉角输出，确保下一个控制周期读取的数据是最新值
        self.update_euler_angles()

    # 初始零偏计算函数，总计需延时3s，初始化陀螺仪的同时进行启动延时，确保平稳启动
    def init_bias(self):
        """暂时不需要这些数据
        acc_x_sum = 0
        acc_y_sum = 0
        acc_z_sum = 0
        """
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0
        sample_count = 1000
        # 将imu_data与imu对象链接起来
        self.imu_data = self.imu.get()
        for i in range(sample_count):
            self.imu_data = self.imu.read()
            gyro_x_sum += self.imu_data[3]
            gyro_y_sum += self.imu_data[4]
            gyro_z_sum += self.imu_data[5]
            time.sleep_ms(4)

        self.gyro_x_bias = gyro_x_sum / sample_count    
        self.gyro_y_bias = gyro_y_sum / sample_count
        self.gyro_z_bias = gyro_z_sum / sample_count

    # 传感器数据更新函数
    def update_data(self):
        self.encoder_data_ul = self.encoder_ul.get() * 4
        self.encoder_data_ur = self.encoder_ur.get() * 4
        self.encoder_data_md = self.encoder_md.get() * 4

        self.gyro_x = (self.imu_data[3] - self.gyro_x_bias) / 16.4 * (math.pi / 180.0)
        self.gyro_y = (self.imu_data[4] - self.gyro_y_bias) / 16.4 * (math.pi / 180.0)
        # self.gyro用于角速度环控制
        self.gyro_z = -(self.imu_data[5] - self.gyro_z_bias) / 16.4 * self.gyro_z_supply
        # gyro_z用于四元数解算
        gyro_z = -self.gyro_z * (math.pi / 180.0)

        # 3. 运行 AHRS 算法（gyro_z顺时针为正，四元数解算需要逆时针为正）
        self.ahrs_update(self.imu_data[0], self.imu_data[1], self.imu_data[2], self.gyro_x, self.gyro_y, gyro_z)
        
        # 4. 更新欧拉角输出
        self.update_euler_angles()

        
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
        # 速度前馈系数
        self.kv = self.flash_sys.find_value("kv")  # type: float
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

    def set_pid_params(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute_pid(self, target: int, actual: int):
        # 如果检测到急刹车指令（目标突变为0），瞬间清空历史包袱
        if target == 0 and self.target != 0:
            self.integral = 0

        self.target = target
        self.actual = actual
        self.preError = self.nowError
        self.nowError = self.target - self.actual

        abs_nowerror = abs(self.nowError)
        coefficient = 1.0   # type: float
        if self.__A == self.__B:
            # 避免除以0
            if (abs_nowerror > self.__A):
                coefficient = 0.0
            else:
                coefficient = 1.0
        else:
            if abs_nowerror > self.__A:
                coefficient = 0.0
            elif abs_nowerror > self.__B:
                coefficient = (self.__A - abs_nowerror) / (self.__A - self.__B)
            else:
                coefficient = 1.0
        
        # 根据误差大小调整积分项
        self.integral += coefficient * self.nowError

        # 积分项限幅
        self.integral = max(-self.__integral_limitmax, min(self.integral, self.__integral_limitmax))

        # 对微分项进行滑动平均滤波
        self.derivative = self.diff_filter.filtering(self.nowError - self.preError)

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError+ self.ki * self.integral + self.kd * self.derivative + self.kv * self.target
        
        
        # 当目标速度为0且此时误差极小时，强制增加一个制动pwm输出来驱动
        if self.target == 0:
            if self.nowError < 5 and self.nowError > 0:
                self.pwm_output += self.pwm_output + 500
            elif self.nowError > -5 and self.nowError < 0:
                self.pwm_output += self.pwm_output - 500
    
        # pwm_output限幅
        self.pwm_output = max(-self.__pwmout_limitmax, min(self.pwm_output, self.__pwmout_limitmax))


# 角度环PID
class AnglePositionPID(ControlPID):
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.kp = self.flash_sys.find_value("angle_normal_kp")        # type: float
        self.kd = self.flash_sys.find_value("angle_normal_kd")        # type: float
        self.target = 0     # type: float
        self.actual = 0     # type: float
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
        self.integral = 0   # type: float
        self.derivative = 0 # type: float
        self.pwm_output = 0 # type: float
        self.high_pwmout_limitmax = self.flash_sys.find_value("high_angle_pwmout_limitmax")    # type: float
        self.low_pwmout_limitmax = self.flash_sys.find_value("low_angle_pwmout_limitmax")    # type: float
        self.pwmout_limitmax = self.high_pwmout_limitmax
        
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

        # 计算pwm_output
        self.pwm_output = self.kp * self.nowError + self.kd * self.derivative

        # pwm_output限幅
        self.pwm_output = max(-self.pwmout_limitmax, min(self.pwm_output, self.pwmout_limitmax))


# 视觉伺服PD
class ServoPID(ControlPID):
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        self.servo_normal_kp_x = self.flash_sys.find_value("servo_normal_kp_x")        # type: float
        self.servo_normal_kd_x = self.flash_sys.find_value("servo_normal_kd_x")        # type: float
        self.servo_normal_kp_y = self.flash_sys.find_value("servo_normal_kp_y")        # type: float
        self.servo_normal_kd_y = self.flash_sys.find_value("servo_normal_kd_y")        # type: float
        self.servo_calibrate_kp_x = self.flash_sys.find_value("servo_calibrate_kp_x")        # type: float
        self.servo_calibrate_kd_x = self.flash_sys.find_value("servo_calibrate_kd_x")        # type: float
        self.servo_calibrate_kp_y = self.flash_sys.find_value("servo_calibrate_kp_y")        # type: float
        self.servo_calibrate_kd_y = self.flash_sys.find_value("servo_calibrate_kd_y")        # type: float
        self.servo_kp_x = 0.0
        self.servo_kd_x = 0.0
        self.servo_kp_y = 0.0
        self.servo_kd_y = 0.0
        self.target_x = 0.0
        self.actual_x = 0     # type: float
        self.target_y_T = self.flash_sys.find_value("servo_target_y_T")     # type: float
        self.target_y_S = self.flash_sys.find_value("servo_target_y_S")     # type: float
        self.target_y_B = self.flash_sys.find_value("servo_target_y_B")     # type: float
        self.target_y_A = self.flash_sys.find_value("servo_target_y_A")     # type: float
        self.target_y = 0.0   # type: float
        self.actual_y = 0     # type: float
        self.nowError_x = 0   # type: float
        self.preError_x = 0   # type: float
        self.nowError_y = 0   # type: float
        self.preError_y = 0   # type: float
        self.derivative_x = 0 # type: float
        self.derivative_y = 0 # type: float
        self.pwm_output_x = 0 # type: int
        self.pwm_output_y = 0 # type: int
        self.current_x = 0  # type: float
        self.current_y = 0  # type: float
        self.__pwmout_limitmax = self.flash_sys.find_value("servo_pwmout_limitmax")    # type: int
    
    # 模型下的pid计算
    def model_compute_pid(self, actual_x: float, actual_y: float):
        # 模型下x和y的目标值都为0
        self.target_x, self.target_y = 0.0, 0.0
        self.actual_x = actual_x
        self.actual_y = actual_y    
        self.preError_x = self.nowError_x
        self.preError_y = self.nowError_y
        self.nowError_x = self.actual_x - self.target_x 
        self.nowError_y = self.actual_y - self.target_y
        # 计算微分项
        self.derivative_x = self.nowError_x - self.preError_x
        self.derivative_y = self.nowError_y - self.preError_y
        # 计算pwm_output
        self.pwm_output_x = int(self.servo_kp_x * self.nowError_x + self.servo_kd_x * self.derivative_x)
        self.pwm_output_y = int(self.servo_kp_y * self.nowError_y + self.servo_kd_y * self.derivative_y)

        # pwm_output限幅
        self.pwm_output_x = max(-self.__pwmout_limitmax, min(self.pwm_output_x, self.__pwmout_limitmax))
        self.pwm_output_y = max(-self.__pwmout_limitmax, min(self.pwm_output_y, self.__pwmout_limitmax))

    # 色块下的pid计算
    def color_compute_pid(self, actual_x: int, actual_y: int):
        # 色块模式下x的目标值为80
        self.target_x = 80
        self.actual_x = actual_x
        self.actual_y = actual_y    
        # 根据拟合公式计算出当前物体中心所在图片宽度与高度的实际距离(cm)
        self.current_x = 1 / (2.99 * 0.0001 * self.actual_y + 7.72 * 0.001)  # type: float
        self.current_y = (-34.0734 * self.actual_y + 4060.2) / (self.actual_y + 52.0064)  # type: float
        self.preError_x = self.nowError_x
        self.preError_y = self.nowError_y
        self.nowError_x = -(self.target_x - self.actual_x) / 160 * self.current_x  # 将像素差转换为实际距离差(cm)
        # 测试拟合后的函数公式是否正确
        # self.nowError_x = -(self.target_x - self.actual_x)  # 直接将距离差作为误差输入
        self.nowError_y = -(self.target_y - self.current_y)
        # 计算微分项
        self.derivative_x = self.nowError_x - self.preError_x
        self.derivative_y = self.nowError_y - self.preError_y
        # 计算pwm_output
        self.pwm_output_x = int(self.servo_kp_x * self.nowError_x + self.servo_kd_x * self.derivative_x)
        self.pwm_output_y = int(self.servo_kp_y * self.nowError_y + self.servo_kd_y * self.derivative_y)

        # pwm_output限幅
        self.pwm_output_x = max(-self.__pwmout_limitmax, min(self.pwm_output_x, self.__pwmout_limitmax))
        self.pwm_output_y = max(-self.__pwmout_limitmax, min(self.pwm_output_y, self.__pwmout_limitmax))


# 小车姿态控制
class CarPose:
    def __init__(self, flash_sys, state_machine, pose_data: PoseData, math, car_yaw_filter: SlipAveragingFilter, angle_pid: AnglePositionPID,
                 motor_ul_pid: SpeedPositionPID, motor_ur_pid: SpeedPositionPID, motor_md_pid: SpeedPositionPID, motor_ul, motor_ur, motor_md):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入速度与路径规划对象
        self.my_state = state_machine
        # 注入姿态数据对象
        self.pose_data = pose_data
        # 注入小车自转角角滑动平均滤波器对象
        self.car_yaw_filter = car_yaw_filter
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
        # 目标转角
        self.turn_angle_target = 0.0  # type: float
        # 速度系数
        self.speed_conversion_gamma = self.flash_sys.find_value("speed_conversion_gamma")   # 将速度单位转化为cm每秒
        self.gkd = self.flash_sys.find_value("gkd")  # type: float  # 角速度补偿系数
        self.speed_fuse_ratio = self.flash_sys.find_value("speed_fuse_ratio")  # type: float  # 速度融合系数
        # 依据角度的位置修正系数（常量）
        self.alpha_x = 1.0  # type: float
        self.alpha_y = 1.0  # type: float
        # 位置
        self.x_current = 0.0   # type: float
        self.y_current = 0.0   # type: float
        self.now_yaw = 0.0  # type: float
        # 采集周期，单位：秒
        self.collect_dt = self.flash_sys.find_value("collect_dt")  # type: float  
        self.last_gyro_z = 0.0  # type: float
        self.last_time = 0      # type: int
        # 测试一个电机的里程
        # self.encouder_ul = 0.0    
        # self.encouder_ur = 0.0
        # self.encouder_md = 0.0
        
    # 小车姿态更新
    def update_pose(self):
        ###################【速度计算】###################
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        self.last_car_speed_w = self.car_speed_w
        # 测试一个电机的里程
        # self.encouder_ul += self.speed_conversion_gamma * self.pose_data.encoder_data_ul / 1000
        # self.encouder_ur += self.speed_conversion_gamma * self.pose_data.encoder_data_ur / 1000
        # self.encouder_md += self.speed_conversion_gamma * self.pose_data.encoder_data_md / 1000

        # 计算小车当前x,y速度（互补融合）
        # car_speed_x, car_speed_y 单位：厘米每5ms
        self.car_speed_x = self.speed_fuse_ratio * self.last_car_speed_x + (1 - self.speed_fuse_ratio) * (self.MATH.OneThird * (self.pose_data.encoder_data_ur + self.pose_data.encoder_data_ul - self.pose_data.encoder_data_md * 2)  * self.speed_conversion_gamma / 1000)
        self.car_speed_y = self.speed_fuse_ratio * self.last_car_speed_y + (1 - self.speed_fuse_ratio) * ((self.MATH.OneThird * self.MATH.SQRT3 * (self.pose_data.encoder_data_ul - self.pose_data.encoder_data_ur)) * self.speed_conversion_gamma / 1000)

        # car_speed_w单位：度每秒
        self.car_speed_w = self.pose_data.gyro_z
        # 计算小车在世界坐标系下的偏航角
        self.now_yaw = -self.pose_data.now_yaw * self.MATH.PI / 180.0
        # 限定now_yaw在-2pi到2pi之间
        if self.now_yaw > self.MATH.PI:  self.now_yaw -= 2 * self.MATH.PI
        elif self.now_yaw < -self.MATH.PI:  self.now_yaw += 2 * self.MATH.PI
        # 转换到世界坐标系下的速度
        self.real_speed_x = self.car_speed_x * math.cos(self.now_yaw) + self.car_speed_y * math.sin(self.now_yaw)
        self.real_speed_y = -self.car_speed_x * math.sin(self.now_yaw) + self.car_speed_y * math.cos(self.now_yaw)
        self.real_speed_w = self.car_speed_w
    
        ###################【位置计算】###################
        # 依据当前航向角调整位置修正系数（解决小车在不同方向上的编码器积分结果不一致问题）
        # 计算小车当前位置，根据运动方向选择补偿系数
        self.x_current += self.real_speed_x * self.alpha_x
        self.y_current += self.real_speed_y * self.alpha_y

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

        # 设置目标转角
        self.turn_angle_target = turn_angle_target
        # self.angle_pid.compute_pid(turn_angle_target, self.now_yaw * 180 / self.MATH.PI)

        # 将move_angle_target转换为弧度
        move_angle_target = move_angle_target * self.MATH.PI / 180
        
        # 设置小车在世界坐标系下的目标速度
        self.real_speed_x_target = move_speed_target * math.sin(move_angle_target)
        self.real_speed_y_target = move_speed_target * math.cos(move_angle_target)
        # 计算角度pid得到转角pwm输出
        # 角度环在10ms中断内
        self.real_speed_w_target = self.angle_pid.pwm_output

        # 转换到小车坐标系下的目标速度
        self.car_speed_x_target = move_speed_target * math.sin(move_angle_target - self.now_yaw)
        self.car_speed_y_target = move_speed_target * math.cos(move_angle_target - self.now_yaw)
        self.car_speed_w_target = self.real_speed_w_target

        # 计算各个电机的目标速度
        motor_ul_speed_target = (self.car_speed_w_target * self.MATH.OneThird + (self.car_speed_x_target + self.car_speed_y_target * self.MATH.SQRT3) * 0.5 + self.pose_data.gyro_z * self.gkd)
        motor_ur_speed_target = (self.car_speed_w_target * self.MATH.OneThird + (self.car_speed_x_target - self.car_speed_y_target * self.MATH.SQRT3) * 0.5 + self.pose_data.gyro_z * self.gkd)
        motor_md_speed_target = (self.car_speed_w_target * self.MATH.OneThird - self.car_speed_x_target + self.pose_data.gyro_z * self.gkd)

        # 计算各个电机的pid得到pwm输出
        self.motor_ul_pid.compute_pid(int(motor_ul_speed_target), self.pose_data.encoder_data_ul)
        self.motor_ur_pid.compute_pid(int(motor_ur_speed_target), self.pose_data.encoder_data_ur)
        self.motor_md_pid.compute_pid(int(motor_md_speed_target), self.pose_data.encoder_data_md)

    # 设置电机pwm输出函数
    def set_motor_pwm(self):
        self.motor_ul.duty(int(self.motor_ul_pid.pwm_output))
        self.motor_ur.duty(int(self.motor_ur_pid.pwm_output))
        self.motor_md.duty(int(self.motor_md_pid.pwm_output))