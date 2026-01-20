# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容 
from machine import *
from display import *
from smartcar import ticker
import ant_else    # 先导入 ant_else 模块以确保蜂鸣器被初始化
import ant_motor   # 先导入 ant_motor 模块以确保配置文件被加载
from ant_flash import find_aimed_value as find_value
import ant_plan
import ant_menu


# 包含 gc 与 time 类
import gc
import time


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

# 构造输入电压分压检测电路接口
power_adc = ADC('B27')

###################################【函数定义】###################################
# 电机驱动函数
def set_motor(motor, duty) -> None:
    motor.duty(duty)

# 是否成功读取文件和开启定时器检查函数
def detect_if_normal() -> None:
    for i in range(4):
        time.sleep_ms(200)
        led.toggle()

# 检测电源电压函数
def voltage_detect(limit_min: float) -> None:
    power_adc_value = power_adc.read_u16()
    power_voltage = power_adc_value / 65535 * 3.3 * 11
    print(f"The current power supply voltage is {power_voltage}!")
    if power_voltage <= limit_min:
        print(f"The power supply voltage: {power_voltage} is too low!")
        ant_else.beep_warn()


""" 定时器类 """
# 定时器1初始化（中断回调函数在 ant_motor 中）
def pit1_start():
    global imu_data
    pit1 = ticker(1)
    pit1.capture_list(ant_motor.encoder_ul, ant_motor.encoder_ur, ant_motor.encoder_md, ant_motor.imu)
    # 将imu对象与传感器数据缓冲区链接起来
    ant_motor.imu_data = ant_motor.imu.get()
    pit1.callback(ant_motor.time_pit1_handler)
    pit1.start(find_value(ant_motor.config, "motor_control_T"))

# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    pit2 = ticker(2)
    pit2.callback(ant_menu.time_pit2_handler)
    pit2.start(find_value(ant_motor.config, "uart_and_menu_T"))

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    pit3 = ticker(3)
    pit3.callback(ant_plan.time_pit3_handler)
    pit3.start(find_value(ant_motor.config, "plan_calculate_T"))

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.6)

# 进行陀螺仪零漂校准
ant_motor.pose_data.init_bias()

# 屏幕测试程序
# ant_menu.Menu_First()

# 打开定时器
pit1_start()
pit3_start()
pit2_start()


# 检测是否正常初始化所有
detect_if_normal()

while True:
    # 屏幕测试程序
    # ant_menu.lcd.str32(100,80,"<--",0xFFFF)
    # ant_menu.lcd.line(90,40,90,280,color = 0xFFFF, thick = 5)
    # time.sleep_ms(500)
    # ant_menu.lcd.clear(0xF800)
    # time.sleep_ms(500)
    # ant_menu.lcd.clear(0x07E0)
    # time.sleep_ms(500)
    # ant_menu.lcd.clear(0x001F)
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        gc.collect()
        break

    gc.collect()