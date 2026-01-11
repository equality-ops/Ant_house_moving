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



# 定义一个抽象类用于顶层设计
# 该类能够存储pid参数并计算得到当前应该输出的pwm值
class ControlPID:
    def compute_pid(self, target: int, actual: int) -> None:
        pass


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





