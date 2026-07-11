# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 包含 gc 与 time 类
import gc
import time

from micropython import const
gc.collect()
# 从 machine 库包含所有内容 
from machine import *
gc.collect()
from seekfree import MOTOR_CONTROLLER, IMU660RX, KEY_HANDLER, BLDC_CONTROLLER, IPS200PRO
gc.collect()
from smartcar import ticker, encoder
gc.collect()
import ant_menu
gc.collect()
import ant_else
gc.collect()

###################################【变量定义及初始化】###################################
PI = const(3.1415926)
READY_NAVIGATE = const(0)   # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)           # 微调状态
RETURN = const(8)		    # 返回状态
STOP = const(9)           # 停止状态
PREDICT = const(10)       # 预测状态

# 多路复用时间计数器
counter = 0      # type: int
# 是否按下启动按键标志位
if_press_start_key = False
# 是否成功启动标志位
start_flag = False

##################################【实例对象构建及初始化】##################################
"""""""""核心板与学习板接口初始化"""""""""
# 核心板上 C4 是 LED
# 学习板上 D9  对应一号拨码开关
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 构造输入电压分压检测电路接口
power_adc = ADC('B27')

pit1 = ticker(1)
pit2 = ticker(2)
pit3 = ticker(3)

"""蜂鸣器初始化"""
beep = Pin('D24', Pin.OUT, value = False)

"""光电管初始化"""
photo = Pin('B4', Pin.IN, value = False)

"""异步串口通信初始化"""
my_uart6 = UART(5)
my_uart6.init(115200)

"""无线串口通信初始化"""
my_uart3 = UART(2)
my_uart3.init(115200)
my_uart8 = UART(7)
my_uart8.init(115200)

"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty=0, invert=False)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert=False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty=0, invert=True)

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("C2" , "C3" , True)
encoder_ur = encoder("D13", "D14", True)
encoder_md = encoder("D16", "D15", True)

"""无刷风扇初始化"""
fan = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_C25, freq=300, highlevel_us = 1000)

# IMU初始化
imu = IMU660RX()

"""菜单与显示屏初始化（IPS200PRO 库自动管理引脚）"""
time.sleep_ms(100)
ips200pro = IPS200PRO(IPS200PRO.TITLE_TOP, 30)
ips200pro.set_backlight(255)

key = KEY_HANDLER(53)
my_beep = ant_else.beep(beep)

#【文件读取】
# 从main_config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/main_config.txt")
my_flash_sys.phase_config()
my_flash_sys.check_list_format()

# 创建菜单对象
my_menu = ant_menu.Menu(my_flash_sys, my_beep, ips200pro)

gc.collect()
###################################【函数定义】###################################
# 电机驱动函数
def set_motor(motor, duty) -> None:
    motor.duty(duty)


# 定时器2中断回调函数
# 用于无线串口调试和发车启动
def time_pit2_handler(time):
    """用于无线串口调试"""

    if start_flag == False:
        # 读取按键（中断中避免阻塞，快速返回）
        key = my_menu.read_key()
        my_menu.handle_key_from_interrupt(key)


# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    global pit2
    pit2.callback(time_pit2_handler)
    pit2.capture_list(key)
    pit2.start(10)  # 10ms

###################################【主程序模块】###################################
# 打开定时器
pit2_start()

while True:

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        gc.collect()
        break

    gc.collect()