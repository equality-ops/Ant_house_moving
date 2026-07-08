import gc
class SlipAveragingFilter:
    # 构造对象时传入滤波窗口大小
    def __init__(self, filter_size: int):
        self.filter_size = filter_size
        self.index = 0
        self.last_value = 0.0
        self.buffer = [0.0] * filter_size

    def buffer_init(self, initial_value):
        self.buffer = [initial_value] * self.filter_size

    # 滤波时传入一个新的数据，返回滤波后的结果(float)
    def filtering(self, data: float) -> float:
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.filter_size
        return sum(self.buffer) / self.filter_size
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
        self.target = 0     # type: float
        self.actual = 0     # type: int
        self.nowError = 0   # type: float
        self.preError = 0   # type: float
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

    def compute_pid(self, target: float, actual: int):
        # 如果检测到急刹车指令（目标突变为0），瞬间清空历史包袱
        if abs(target) <= 5 and abs(self.target) >= 1e-6:
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
        self.servo_kp_x = 0.0
        self.servo_kd_x = 0.0
        self.servo_kp_y = 0.0
        self.servo_kd_y = 0.0
        self.target_x = 0.0
        self.actual_x = 0.0
        self.target_y = 0.0   # type: float
        self.actual_y = 0.0     # type: float
        
        self.target_y_T = self.flash_sys.find_value("servo_target_y_T")     # type: float
        self.target_y_S = self.flash_sys.find_value("servo_target_y_S")     # type: float
        self.target_y_B = self.flash_sys.find_value("servo_target_y_B")     # type: float    

        self.nowError_x = 0   # type: float
        self.preError_x = 0   # type: float
        self.nowError_y = 0   # type: float
        self.preError_y = 0   # type: float
        self.derivative_x = 0 # type: float
        self.derivative_y = 0 # type: float
        self.pwm_output_x = 0 # type: int
        self.pwm_output_y = 0 # type: int
        self.pwmout_normal_limit = self.flash_sys.find_value("pwmout_normal_limit") 
        self.pwmout_limitmax = 100    # type: int
    
        gc.collect()  # 主动触发垃圾回收，释放内存

    # pid计算
    def model_compute_pid(self, actual_x: float, actual_y: float):
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
        self.pwm_output_x = max(-self.pwmout_limitmax, min(self.pwm_output_x, self.pwmout_limitmax))
        self.pwm_output_y = max(-self.pwmout_limitmax, min(self.pwm_output_y, self.pwmout_limitmax))
        # 模型下的pid计算
# 滑动平均滤波器

    