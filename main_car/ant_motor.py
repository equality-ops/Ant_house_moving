import micropython
from micropython import const
import math
import time
import gc

PI = const(3.1415926)
OneThird = const(0.3333333)
SQRT3 = const(1.7320508)
InField = const(-1)
OnLine = const(0)
OutLine = const(1)

# 光电管控制类
class PhotoControl:
    __slots__ = ("flash_sys","my_beep","my_beep","my_photo","photo_state","current_state","on_line_times")
    def __init__(self, flash_sys, beep, photo) -> None:
        self.flash_sys = flash_sys
        self.my_beep = beep
        self.my_photo = photo
        self.photo_state = self.my_photo.value()
        self.current_state = InField
        # 当光电管位于黄线正上方的次数
        self.on_line_times = 0
        gc.collect()

    def update_photo_state(self):
        current_state = self.my_photo.value()
        if current_state == 1 and self.current_state == InField:
            self.on_line_times += 1
            if self.on_line_times >= 3:  # 连续3次检测到在线，才认为真正进入了线上
                self.on_line_times = 0
                self.current_state = OnLine
        
        if current_state == 0 and self.current_state == OnLine:
            self.current_state = OutLine

    def reset_photo(self):
        self.on_line_times = 0
        self.current_state = InField
# 无刷风扇控制类
class FanControl:
    __slots__ = ("flash_sys","my_fan","my_state","fan_signal_limit","if_fan","fixed_high_level_us")
    def __init__(self, flash_sys, fan , state):
        self.flash_sys = flash_sys
        self.my_fan = fan
        self.my_state = state

        self.fan_signal_limit = 1350  # type: int  # 无刷风扇信号限幅
        self.if_fan = self.flash_sys.find_value("if_fan")  # type: bool  # 是否开启风扇控制
        self.fixed_high_level_us = self.flash_sys.find_value("fixed_high_level_us")  # type: int  # 高电平持续时间，单位微秒
        gc.collect()

    # 设置无刷风扇的高电平时间
    def set_fan_signal(self):
         # 限幅在 1000-self.fan_signal_limit 之间
        if self.if_fan:
            high_level_us = max(1000, min(self.fixed_high_level_us, self.fan_signal_limit)) 
            # 更新高电平时间值
            self.my_fan.highlevel_us(high_level_us)
        else:
            self.fan_off()

    # 测试用的风扇高电平时间设置函数，直接传入一个值进行测试
    def test_fan(self, high_level_us):
        high_level_us = max(1000, min(high_level_us, self.fan_signal_limit)) 
        self.my_fan.highlevel_us(high_level_us)

    # 关闭风扇（设置为最低信号）
    def fan_off(self):
        self.my_fan.highlevel_us(1000)
# 滑动平均滤波器
class SlipAveragingFilter:
    # 构造对象时传入滤波窗口大小
    __slots__ = ('filter_size', 'index', 'last_value', 'buffer')
    def __init__(self, filter_size: int):
        self.filter_size = filter_size
        self.index = 0
        self.last_value = 0.0
        self.buffer = [0.0] * filter_size

        gc.collect()

    def buffer_init(self, initial_value):
        self.buffer = [initial_value] * self.filter_size

    # 滤波时传入一个新的数据，返回滤波后的结果(float)
    def filtering(self, data: float) -> float:
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.filter_size
        return sum(self.buffer) / self.filter_size
# 一维卡尔曼滤波器
class KalmanFilter:
    __slots__ = ('P', 'Q', 'R', 'Output')
    def __init__(self, P=1.0, Q=0.01, R=0.1, initial_output=0.0):
        self.P = P
        self.Q = Q
        self.R = R
        self.Output = initial_output
        gc.collect()

    def update(self, input_value):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.Output += K * (input_value - self.Output)
        self.P = (1 - K) * self.P
        return self.Output    
class PoseData:
    def __init__(self, flash_sys, my_uart3, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入串口对象
        self.my_uart3 = my_uart3
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
        self.gyro_z_gkd = 0         # type: float # 供角速度环控制用的原始角速度值
        # 角速度零漂误差
        self.gyro_x_bias = 0.0       # type: float
        self.gyro_y_bias = 0.0       # type: float
        self.gyro_z_bias = 0.0        # type: float

        # 四元数初始化
        self.q = [1.0, 0.0, 0.0, 0.0]
        # 误差积分项
        self.e_int = [0.0, 0.0, 0.0]
        
        # 上次更新时间戳
        self.last_update_time = time.ticks_us()

        # 算法参数 (根据你的 2ms 采样周期设置)
        self.dt = 0.002 
        self.kp = 1.0  # 加速度计权重
        self.ki = 0.00001 # 零偏补偿权重

        # 最终角度输出
        self.now_pitch = 0.0  # 俯仰角
        self.now_roll = 0.0   # 横滚角
        self.now_yaw = 0.0    # 偏航角

        gc.collect()  # 主动触发垃圾回收，释放内存

    # 更新四元数
    @micropython.native
    def ahrs_update(self, ax, ay, az, gx, gy, gz):
        """
        核心四元数更新算法
        输入单位：ax-az (g), gx-gz (rad/s)
        """
        q0, q1, q2, q3 = self.q
        
        # 1. 当前加速度计的原始数据模长
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0: return # 防止除以0

        G_REFERENCE = 4096.0  # TODO: 请你在串口打印一下静止时 norm 的值，并把它填在这里！
        
        # 计算测量模长与标准重力 1g 的绝对偏差 (单位重新化为 g)
        acc_error = abs(norm - G_REFERENCE) / G_REFERENCE
        
        # 设定信任阈值 (偏差在 0.05g 以内完全信任，偏差大于 0.1g 完全不信任)
        LOWER_THRESHOLD = 0.05
        UPPER_THRESHOLD = 0.1
        
        dynamic_weight = 1.0  # 默认权重为 1
        
        if acc_error < LOWER_THRESHOLD:
            dynamic_weight = 1.0
        elif acc_error > UPPER_THRESHOLD:
            dynamic_weight = 0.0
        else:
            # 线性插值，平滑过渡 
            dynamic_weight = 1.0 - ((acc_error - LOWER_THRESHOLD) / (UPPER_THRESHOLD - LOWER_THRESHOLD))
            
        # 计算当前周期实际使用的 kp
        current_kp = self.kp * dynamic_weight

        # self.my_uart3.write(f"{norm},{current_kp}\n")  # 调试用：输出原始加速度模长

        # 继续执行归一化，将向量化为长度为 1 的单位向量给后续解算用
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
        I_LIMIT = 0.1  # 限制积分项最大影响
        self.e_int[0] = max(-I_LIMIT, min(self.e_int[0] + ex * self.ki, I_LIMIT))
        self.e_int[1] = max(-I_LIMIT, min(self.e_int[1] + ey * self.ki, I_LIMIT))
        self.e_int[2] = 0.0 # 6轴系统，不要信任加速度计对 Yaw 的积分修正，强制清零
        
        # 强制将 ez 置为 0，防止加速度计在 Z 轴上的假误差污染陀螺仪的 gz
        ez = 0.0 

        # --- 改进2：补偿角速度 ---
        gx += current_kp * ex + self.e_int[0]
        gy += current_kp * ey + self.e_int[1]
        gz += current_kp * ez + self.e_int[2]
        
        # 6. 一阶龙格库塔法更新四元数
        half_dt = 0.5 * self.dt
        q0_new = q0 + (-q1*gx - q2*gy - q3*gz) * half_dt
        q1_new = q1 + (q0*gx + q2*gz - q3*gy) * half_dt
        q2_new = q2 + (q0*gy - q1*gz + q3*gx) * half_dt
        q3_new = q3 + (q0*gz + q1*gy - q2*gx) * half_dt
        
        # 7. 再次归一化四元数
        norm = math.sqrt(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new)
        self.q[0] = q0_new/norm
        self.q[1] = q1_new/norm
        self.q[2] = q2_new/norm
        self.q[3] = q3_new/norm
    
    # 将四元数转化为欧拉角
    @micropython.native
    def update_euler_angles(self):
        """将四元数转换为欧拉角（度）"""
        q0, q1, q2, q3 = self.q
        
        # 1. 俯仰角 Pitch (绕 Y 轴旋转)
        # 数学逻辑：asin(-2*(q1*q3 - q0*q2))
        val = -2.0 * (q1 * q3 - q0 * q2)
        # 边界处理，防止 asin 超域报错
        val = max(-1.0, min(1.0, val))
        self.now_pitch = math.asin(val) * (180.0 / PI)
        
        # 保护万向节锁，当 Pitch 接近 +- 90 度时
        if abs(val) > 0.999: # 极高仰角时 Roll 和 Yaw 共线
            self.now_roll = 0.0
            self.now_yaw = math.atan2(2.0 * (q1 * q2 - q0 * q3), 1.0 - 2.0 * (q1 * q1 + q3 * q3)) * (180.0 / PI)
        else:
            # 2. 横滚角 Roll (绕 X 轴旋转)
            # 使用更标准的 1 - 2*X^2 简化运算与误差
            self.now_roll = math.atan2(2.0 * (q2 * q3 + q0 * q1), 
                                    1.0 - 2.0 * (q1 * q1 + q2 * q2)) * (180.0 / PI)
            
            # 3. 偏航角 Yaw (绕 Z 轴旋转)
            self.now_yaw = math.atan2(2.0 * (q1 * q2 + q0 * q3), 
                                    1.0 - 2.0 * (q2 * q2 + q3 * q3)) * (180.0 / PI)

    # 重置四元数
    def reset_yaw(self, ref_yaw_deg):
        """
        通过外部参考信息强制重置当前的偏航角 (Yaw)。
        保留当前的横滚角 (Roll) 和俯仰角 (Pitch)，重新合成四元数。
        
        :param ref_yaw_deg: 外部传感器获取的绝对偏航角，单位：度 (°)
        """
        # 1. 将角度转换为半角弧度
        half_roll = self.now_roll * 0.5 * (PI / 180.0)
        half_pitch = self.now_pitch * 0.5 * (PI / 180.0)
        half_yaw = -ref_yaw_deg * 0.5 * (PI / 180.0)

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
        self.q[0] = q0/norm
        self.q[1] = q1/norm
        self.q[2] = q2/norm
        self.q[3] = q3/norm

        # 5. 清空 PI 算法的积分补偿项
        # 这一步极其重要：如果不清空，历史误差的积分积累会在接下来的几个周期内把姿态又“拉回”一点点，导致修正不干脆
        self.e_int[0] = 0.0
        self.e_int[1] = 0.0
        self.e_int[2] = 0.0

        # 6. 同步更新底层的欧拉角输出，确保下一个控制周期读取的数据是最新值
        self.update_euler_angles()

    # 初始零偏计算函数，总计需延时3s，初始化陀螺仪的同时进行启动延时，确保平稳启动
    def init_bias(self):
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
        # 1. 计算真实的动态 dt
        current_time = time.ticks_us()
        # 计算时间差并转换为秒 (MicroPython 下推荐用 ticks_diff 防溢出)
        self.dt = time.ticks_diff(current_time, self.last_update_time) / 1000000.0
        self.last_update_time = current_time

        # print(f"dt: {self.dt:.6f} s")
        # self.my_uart3.write(f"dt: {self.dt:.6f} s\n")  # 调试用：输出实际 dt
        # 防止 dt 出现离谱的值（比如程序刚启动卡顿）
        if self.dt > 0.1: 
            self.dt = 0.002

        self.encoder_data_ul = self.encoder_ul.get() * 3
        self.encoder_data_ur = self.encoder_ur.get() * 3
        self.encoder_data_md = self.encoder_md.get() * 3

        self.gyro_x = (self.imu_data[3] - self.gyro_x_bias) / 16.4 * (PI / 180.0) * self.gyro_z_supply
        self.gyro_y = (self.imu_data[4] - self.gyro_y_bias) / 16.4 * (PI / 180.0) * self.gyro_z_supply
        # self.gkd用于角速度环控制
        self.gyro_z_gkd = (self.imu_data[5] - self.gyro_z_bias) / 16.4 * self.gyro_z_supply
        # gyro_z用于四元数解算
        self.gyro_z = self.gyro_z_gkd * (PI / 180.0)

        DEADBAND = 0.004 # 弧度每秒
        if abs(self.gyro_x) < DEADBAND: self.gyro_x = 0.0
        if abs(self.gyro_y) < DEADBAND: self.gyro_y = 0.0
        if abs(self.gyro_z) < DEADBAND: self.gyro_z = 0.0

        # 注意：这里千万不要因为陀螺仪为 0 就直接 return 退出！
        # 如果退出，加速度计就无法把移动时产生的错误倾角（Pitch/Roll）慢慢修正回 0
            
        # 3. 运行 AHRS 算法（构建严格的“右前上”或“前往左上”右手坐标系）
        # 基于你的物理方向，我们将其映射为：X向前，Y向左，Z向上 (这也是标准的 FLU 右手标系)
        # 加速度映射：原X向后->取负变向前；原Y向左->保留向左(+1)；原Z向下(静止负)->取负变向上
        # 角速度映射：原gx(绕向后)被翻转；原gy(绕向左)保留；原gz(顺时针)被翻转为逆时针
        self.ahrs_update(-self.imu_data[0], self.imu_data[1], -self.imu_data[2], 
                         -self.gyro_x, self.gyro_y, -self.gyro_z)
        
        # 4. 更新欧拉角输出
        self.update_euler_angles()
# 小车姿态控制
class CarPose:
    __slots__ = (
        'flash_sys','my_state','pose_data','car_yaw_filter','angle_pid','motor_ul_pid',
        'motor_ur_pid','motor_md_pid','motor_ul','motor_ur','motor_md','last_car_speed_x',
        'last_car_speed_y','car_speed_x','car_speed_y','turn_angle_target','speed_conversion_gamma',
        'gkd','speed_fuse_ratio','alpha_x','alpha_y','x_current','y_current','now_yaw','last_gyro_z',
        'last_time',
    )
    def __init__(self, flash_sys, state_machine, pose_data: PoseData, car_yaw_filter: SlipAveragingFilter, angle_pid,
                 motor_ul_pid, motor_ur_pid, motor_md_pid, motor_ul, motor_ur, motor_md):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入速度与路径规划对象
        self.my_state = state_machine
        # 注入姿态数据对象
        self.pose_data = pose_data
        # 注入小车自转角角滑动平均滤波器对象
        self.car_yaw_filter = car_yaw_filter
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
        # 小车坐标系下的当前速度
        self.car_speed_x = 0.0  # type: float
        self.car_speed_y = 0.0  # type: float
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
        self.last_gyro_z = 0.0  # type: float
        self.last_time = 0      # type: int
        gc.collect()  # 主动触发垃圾回收，释放内存
        
    # 小车姿态更新
    def update_pose(self):
        ###################【速度计算】###################
        # 保存上一次速度
        self.last_car_speed_x = self.car_speed_x
        self.last_car_speed_y = self.car_speed_y
        # 计算小车当前x,y速度（互补融合）
        # car_speed_x, car_speed_y 单位：厘米每5ms
        self.car_speed_x = self.speed_fuse_ratio * self.last_car_speed_x + (1 - self.speed_fuse_ratio) * (OneThird * (self.pose_data.encoder_data_ur + self.pose_data.encoder_data_ul - self.pose_data.encoder_data_md * 2)  * self.speed_conversion_gamma / 1000)
        self.car_speed_y = self.speed_fuse_ratio * self.last_car_speed_y + (1 - self.speed_fuse_ratio) * (OneThird * SQRT3 * (self.pose_data.encoder_data_ul - self.pose_data.encoder_data_ur)) * self.speed_conversion_gamma / 1000

        # 计算小车在世界坐标系下的偏航角
        self.now_yaw = -self.pose_data.now_yaw * PI / 180.0
        # 限定now_yaw在-2pi到2pi之间
        if self.now_yaw > PI:  self.now_yaw -= 2 * PI
        elif self.now_yaw < -PI:  self.now_yaw += 2 * PI
        # 转换到世界坐标系下的速度
        real_speed_x = self.car_speed_x * math.cos(self.now_yaw) + self.car_speed_y * math.sin(self.now_yaw)
        real_speed_y = -self.car_speed_x * math.sin(self.now_yaw) + self.car_speed_y * math.cos(self.now_yaw)

        ###################【位置计算】###################
        # 依据当前航向角调整位置修正系数（解决小车在不同方向上的编码器积分结果不一致问题）
        # 计算小车当前位置，根据运动方向选择补偿系数
        self.x_current += real_speed_x * self.alpha_x
        self.y_current += real_speed_y * self.alpha_y

    # 全向移动控制函数
    # 参数说明：move_speed_target单位：编码器脉冲， move_angle_target单位：度， turn_angle_target单位：度
    def move_ctrl(self, move_speed_target: float, move_angle_target: float, turn_angle_target: float):
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

        # 将move_angle_target转换为弧度
        move_angle_target = move_angle_target * PI / 180
        
        # 转换到小车坐标系下的目标速度
        car_speed_x_target = move_speed_target * math.sin(move_angle_target - self.now_yaw)
        car_speed_y_target = move_speed_target * math.cos(move_angle_target - self.now_yaw)
        car_speed_w_target = self.angle_pid.pwm_output

        # 计算各个电机的目标速度
        motor_ul_speed_target = (car_speed_w_target * OneThird + (car_speed_x_target + car_speed_y_target * SQRT3) * 0.5 + self.pose_data.gyro_z_gkd * self.gkd)
        motor_ur_speed_target = (car_speed_w_target * OneThird + (car_speed_x_target - car_speed_y_target * SQRT3) * 0.5 + self.pose_data.gyro_z_gkd * self.gkd)
        motor_md_speed_target = (car_speed_w_target * OneThird - car_speed_x_target + self.pose_data.gyro_z_gkd * self.gkd)

        # 计算各个电机的pid得到pwm输出
        self.motor_ul_pid.compute_pid(motor_ul_speed_target, self.pose_data.encoder_data_ul)
        self.motor_ur_pid.compute_pid(motor_ur_speed_target, self.pose_data.encoder_data_ur)
        self.motor_md_pid.compute_pid(motor_md_speed_target, self.pose_data.encoder_data_md)

    # 设置电机pwm输出函数
    def set_motor_pwm(self):
        self.motor_ul.duty(int(self.motor_ul_pid.pwm_output))
        self.motor_ur.duty(int(self.motor_ur_pid.pwm_output))
        self.motor_md.duty(int(self.motor_md_pid.pwm_output))

    # pwm信号归零
    def pwm_stop(self):
        self.motor_ul.duty(0)
        self.motor_ur.duty(0)
        self.motor_md.duty(0)