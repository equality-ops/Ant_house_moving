# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker
from smartcar import encoder
import hqu_motor
import ant_menu

# 包含 gc 与 time 类
import gc
import time

###################################【变量定义及初始化】###################################
enc_data = 0    # type: int
target = 100    # type: int


##################################【实例对象构建及初始化】##################################
# 核心板上 C4 是 LED
# 学习板上 D9  对应二号拨码开关
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 蜂鸣器初始化
beep = Pin('C9' , Pin.OUT, pull = Pin.PULL_UP_47K, value = False)

# 创建MOTOR_CONTROLLER对象
motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)

# 异步串口通信初始化
my_uart6 = UART(5)
my_uart6.init(460800)
my_uart6.write("Motor test begins!\r\n")

# 编码器初始化
encoder_l = encoder("D15", "D16", True)

# 创建电机微分项的滑动平均滤波器对象
diff_filter_1 = hqu_motor.SlipAveragingFilter(5)

# 创建电机pid对象
Posi_speed_PID_l = hqu_motor.SpeedPositionPID(kp = 0.0, ki = 0.2, kd = 0.0, integral_limit = 29000, pwmout_limitmax = 6000, diff_filter = diff_filter_1)

# 新建LCD实例并初始化
cs = Pin('C5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
cs.high()
cs.low()
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(2)
lcd.clear(0x0000)

###################################【函数定义】###################################
# 电机驱动函数
def set_motor(motor, duty) -> None:
    motor.duty(duty)

""" 定时器类 """
# 定时器中断回调函数
def time_pit1_handler(time):
    global enc_data
    enc_data = encoder_l.get()
    Posi_speed_PID_l.compute_pid(target = target, actual = enc_data)
    set_motor(motor_1, Posi_speed_PID_l.pwm_output)
    # 输出波形图用于调试电机pid
    my_uart6.write("%d %d %.3f\r\n" % (target, enc_data, Posi_speed_PID_l.pwm_output))

# 定时器1初始化
def pit1_start():
    pit1 = ticker(1)
    pit1.capture_list(encoder_l)
    pit1.callback(time_pit1_handler)
    pit1.start(10)

# 定时器2初始化
def PIT2_Start():
    pit2 = ticker(1)
    pit2.callback(ant_menu.time_pit2_handler)
    pit2.start(10)

###################################【主程序模块】###################################
# 打开定时器
pit1_start()
PIT2_Start()

while True:

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break

    gc.collect()
