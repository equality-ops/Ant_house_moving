# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker, encoder
import ant_motor
import ant_beep
import ant_flash
from ant_flash import find_aimed_value as find_value
import ant_menu

# 包含 gc 与 time 类
import gc
import time

###################################【文件读取】###################################
# 从config.txt中读取保存所有的参数并保存到config字典中
config = ant_flash.phase_config("/flash/config.txt")

###################################【变量定义及初始化】###################################
enc_data_L = 0    # type: int
target_L = 100    # type: int
beep_state = 0    # type: int

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
diff_filter_1 = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为5个

# 创建电机pid对象
Posi_speed_PID_L = ant_motor.SpeedPositionPID(kp = find_value(config, "L_normal_kp"), 
                                              ki = find_value(config, "L_normal_ki"), 
                                              kd = find_value(config, "L_normal_kd"),  
                                              pwmout_limitmax = 6000, 
                                              diff_filter = diff_filter_1)

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

# 是否成功读取文件和开启定时器检查函数
def detect_if_normal() -> None:
    for i in range(4):
        time.sleep_ms(400)
        led.toggle()

""" 定时器类 """
# 定时器中断回调函数
def time_pit1_handler(time):
    global enc_data
    enc_data = encoder_l.get()
    Posi_speed_PID_L.compute_pid(target = target_L, actual = enc_data)
    set_motor(motor_1, Posi_speed_PID_L.pwm_output)

# 定时器1初始化
def pit1_start():
    pit1 = ticker(1)
    pit1.capture_list(encoder_l)
    pit1.callback(time_pit1_handler)
    pit1.start(10)

# 定时器2初始化
def pit2_start():
    pit2 = ticker(2)
    pit2.callback(ant_menu.time_pit2_handler)
    pit2.start(20)

###################################【主程序模块】###################################
# 打开定时器
pit1_start()
pit2_start()

# 检测是否正常开启定时器并读取文件
detect_if_normal()

while True:
    # 输出波形图用于调试电机pid
    my_uart6.write("%d %d %.3f\r\n" % (target_L, enc_data_L, Posi_speed_PID_L.pwm_output))

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        ticker(1).stop()
        ticker(2).stop()
        break

    gc.collect()
