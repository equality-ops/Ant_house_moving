from machine import *
from display import *
from smartcar import ticker,encoder
from user_main import lcd

# 闭环控制回调
def time_pit1_handler(time):
    ant_key.button_scan() # 函数：按键扫描
    ant_beep.Beep_Operate() # 函数：响应蜂鸣器操作
    ant_motor.encl_data, ant_motor.encr_data = encoder_l.get(), -encoder_r.get()
    ant_element.Encoder_Element() # 编码器辅助元素识别
    # 这部分操作需结合后续其他文件情况！！！！

# 定时器初始化
def PIT1_Start():
    pit1 = ticker(1)
    pit1.callback(time_pit1_handler)
    pit1.start(10)