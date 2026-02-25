# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容 
from machine import *
from display import *
from seekfree import MOTOR_CONTROLLER, IMU660RX, DL1X
from smartcar import ticker, encoder
import ant_else
import ant_menu


# 包含 gc 与 time 类
import gc
import time

##################################【实例对象构建及初始化】##################################
"""""""""核心板与学习板接口初始化"""""""""
# 核心板上 C4 是 LED
# 学习板上 D9  对应一号拨码开关
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()


"""蜂鸣器初始化"""
beep = Pin('D24', Pin.OUT, value = False)

"""异步串口通信初始化"""
my_uart6 = UART(5)
my_uart6.init(460800)

"""无线串口通信初始化"""
my_uart3 = UART(2)
my_uart3.init(115200)

"""菜单与显示屏初始化"""
# 新建LCD实例并初始化
cs = Pin('B29' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
cs.high()
cs.low()
rst = Pin('B31' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
dc  = Pin('B5' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
blk = Pin('C21' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
drv = LCD_Drv(SPI_INDEX = 2, BAUDRATE = 60000000, DC_PIN = dc, RST_PIN = rst, LCD_TYPE = LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(0)
lcd.clear(0x0000)

#采用gpio设置引脚高低电平方式，请自行根据自己单片机采用的IO口修改。
end_switch = Pin('C18', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
key_left = Pin('C8', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_right = Pin('C15', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_up = Pin('C9', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_down = Pin('C14', Pin.IN, pull = Pin.PULL_UP_47K, value = True)

# 创建蜂鸣器对象
my_beep = ant_else.beep(beep)

#【文件读取】
# 从main_config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/main_config.txt")
my_flash_sys.phase_config()

# 创建菜单对象
my_menu = ant_menu.Menu(my_flash_sys, my_beep, key_up, key_down, key_left, key_right, lcd)
###################################【函数定义】###################################

while True:
    key = my_menu.read_key()
    if key == None:
        pass
    if key == my_menu.UP:
        my_menu.arrow_up(key)
    elif key == my_menu.DOWN:
        my_menu.arrow_down(key)
    elif key in (my_menu.LEFT, my_menu.RIGHT):
        if my_menu.Current_line == my_menu.End_line:
            old_page = my_menu.change_page_to
            if my_menu.detect_change_page(key):
                if my_menu.change_page_to != old_page:
                    my_menu.menu_switch()
                    my_menu.show_arrow()
        else:
            my_menu.data_processing(key)
            if my_menu.change_page_to == 1:
                my_menu.Menu_Page1_data_show()
            elif my_menu.change_page_to == 2:
                my_menu.Menu_Page2_data_show()
            elif my_menu.change_page_to == 3:
                my_menu.Menu_Page3_data_show()
            elif my_menu.change_page_to == 4:
                my_menu.Menu_Page4_data_show()
            elif my_menu.change_page_to == 5:
                my_menu.Menu_Page5_data_show()
                
            my_menu.show_arrow()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        gc.collect()
        break

    gc.collect()