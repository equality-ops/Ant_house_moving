from machine import *
from display import *
from smartcar import ticker,encoder
from ant_flash import find_aimed_value as find_value
import time
import ant_motor
import ant_plan
import ant_uart

##########################硬件初始化##########################
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
lcd.mode(2)
lcd.clear(0x0000)

#采用gpio设置引脚高低电平方式，请自行根据自己单片机采用的IO口修改。
end_switch = Pin('C18', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
key_up = Pin('D23', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_down = Pin('D22', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_left = Pin('D20', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_right = Pin('D21', Pin.IN, pull = Pin.PULL_UP_47K, value = True)

###########################读取所需参数############################
ul_normal_kp = find_value(ant_motor.config, "ul_normal_kp")  # type: float
ul_normal_ki = find_value(ant_motor.config, "ul_normal_ki")  # type: float
ul_normal_kd = find_value(ant_motor.config, "ul_normal_kd")  # type: float

# 闭环控制回调
def time_pit2_handler(time):
    # ant_key.button_scan() # 函数：按键扫描（后续要补）
    # ant_beep.Beep_Operate() # 函数：响应蜂鸣器操作(后续要补)
    # ant_motor.encl_data, ant_motor.encr_data = encoder_l.get(), -encoder_r.get()
    # 这部分操作需结合后续其他文件情况！！！！
    
    # 用于无线串口调试
    
    # 速度环输出波形图调参
    # ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.motor_ul_pid.target, ant_motor.motor_ul_pid.actual, ant_motor.motor_ul_pid.pwm_output, ant_motor.motor_ul_pid.derivative * ant_motor.motor_ul_pid.kd))
    # ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.motor_ur_pid.target, ant_motor.motor_ur_pid.actual, ant_motor.motor_ur_pid.pwm_output, ant_motor.motor_ur_pid.derivative * ant_motor.motor_ur_pid.kd))
    # ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.motor_md_pid.target, ant_motor.motor_md_pid.actual, ant_motor.motor_md_pid.pwm_output, ant_motor.motor_md_pid.derivative * ant_motor.motor_md_pid.kd))
    
    # 里程计：
    # ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.x_current, ant_motor.my_car.y_current, ant_plan.my_plan.ideal_target_x, ant_plan.my_plan.ideal_target_y))
    ant_uart.wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.x_current, ant_motor.my_car.y_current, ant_plan.my_plan.real_target_x, ant_plan.my_plan.real_target_y, ant_plan.my_plan.rest_distance, ant_plan.my_plan.target_yaw, ant_motor.my_car.now_yaw, ant_plan.my_plan.arrive_flag, ant_plan.my_plan.transition_flag))
    pass

###############################变量定义###########################
# 当前菜单项
change_page_to = 0  # 将菜单定位到哪一页
Current_line = 0  # 当前行
Start_line, End_line = 0, 0 # 显示的起始行，结束行

LEFT, RIGHT, UP, DOWN = "left", "right", "up", "down"

##############################函数定义###############################
# 检测按键状态
def read_key(debounce_ms = 40):
    # 检测是否按下（低电平有效）
    if key_left.value() == 0:
        time.sleep_ms(debounce_ms)  # 消抖延时
        if key_left.value() == 0:   # 再次确认
            return LEFT
    elif key_right.value() == 0:
        time.sleep_ms(debounce_ms)
        if key_right.value() == 0:
            return RIGHT
    elif key_up.value() == 0:
        time.sleep_ms(debounce_ms)
        if key_up.value() == 0:
            return UP
    elif key_down.value() == 0:
        time.sleep_ms(debounce_ms)
        if key_down.value() == 0:
            return DOWN
    
    return None  # 无按键按下


# 显示箭头
def show_arrow():
    global Start_line,End_line,Current_line
    lcd.str16(120, 64, "<--", 0xFFFF)
    """
    lcd.str16(0,16*19,"line={:<2d}".format(Current_line),0xFFFF)
    for i in range(Start_line, End_line + 1):
        if i == Current_line:
            lcd.str16(200,16*i,"<--",0xFFFF)
        else:
            lcd.str16(200,16*i,"   ",0xFFFF)
    """

# 箭头上移
def arrow_up():
    global Start_line,End_line,Current_line
    if read_key() == UP:
        if Current_line > Start_line:
            Current_line -= 1
        else:
            Current_line = End_line
        show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

# 箭头下移
def arrow_down():
    global Start_line,End_line,Current_line
    if read_key() == DOWN:
        if Current_line < End_line:
            Current_line += 1
        else:
            Current_line = Start_line
        show_arrow
    # 判断按键状态，清除状态并且进行箭头的移动

# 箭头的移动,包含上移和下移
def move_arrow():
    arrow_up()
    arrow_down() 

# 监测指定的跳转页面行是否被按下，并指定目标页面
def detect_change_page():
    global change_page_to, Current_line, End_line
    key = read_key()
    if Current_line == End_line:
        if key == LEFT:
            if change_page_to > 0:
                change_page_to -= 1
            else:
                change_page_to = 2
        elif key == RIGHT:
            if change_page_to == 2:
                change_page_to = 0
            else:
                change_page_to += 1
        return True
    else:
        return False


# 第一页菜单数据显示
def Menu_Page1_data_show():
    lcd.str16(60,64*1,"ul_normal_kp:1",0xFFFF)
    lcd.str16(60,64*1,"ul_normal_ki:1",0xFFFF)

#函数：第 1 页菜单显示
def Menu_Page_1():
    global change_page_to, Start_line,End_line,Current_line
    Start_line,End_line,Current_line=0,0,0
    lcd.clear(0x0000)
    Menu_Page1_data_show()
    show_arrow()
    
def Menu_Page_2():
    global change_page_to, Start_line,End_line,Current_line
    Start_line,End_line,Current_line=0,0,0
    lcd.str16(60,64*1,"ul_normal_kp:2",0xFFFF)
    lcd.str16(60,64*2,"ul_normal_ki:2",0xFFFF)



#函数：菜单选择与切换
def menu_switch():
    if(change_page_to == 1):
        Menu_Page_1()
    elif(change_page_to == 2):
        Menu_Page_2()


# 主函数部分
"""
# === 初始显示 ===
Menu_Page_1()
show_arrow()

# === 主循环 ===
while True:
    last_change_page_to = change_page_to
    show_arrow()
    if detect_change_page():
        if change_page_to != last_change_page_to:
            menu_switch()
            show_arrow()
"""
