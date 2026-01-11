# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *
from display import *
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker, encoder
import ant_motor
import ant_flash
import ant_beep
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

# 构造输入电压分压检测电路接口
power_adc = ADC('B27')

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
    if power_voltage <= limit_min:
        print("The power supply voltage is too low!")
        ant_beep.beep_warn()


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
# 检测电源电压是否正常
voltage_detect(11.1)

# 打开定时器
pit1_start()
pit2_start()

# flash测试程序
print(f"L_normal_kp: {Posi_speed_PID_L.kp}")
print(f"L_normal_ki: {Posi_speed_PID_L.ki}")
print(f"L_normal_kd: {Posi_speed_PID_L.kd}")

find_value(config, "hello")

# 屏幕测试程序
ant_menu.Menu_First()

# 检测是否正常初始化所有
detect_if_normal()

while True:
    # 输出波形图用于调试电机pid
    my_uart6.write("%d %d %.3f\r\n" % (target_L, enc_data_L, Posi_speed_PID_L.pwm_output))

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
        break

    gc.collect()


