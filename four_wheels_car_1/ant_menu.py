from machine import *
from display import *
from smartcar import ticker,encoder
from ant_flash import find_aimed_value as find_value
from ant_flash import MATH as MATH
import time
import ant_motor
import ant_plan
import ant_else
import ant_else

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
lcd.mode(0)
lcd.clear(0x0000)

#采用gpio设置引脚高低电平方式，请自行根据自己单片机采用的IO口修改。
end_switch = Pin('C18', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
key_up = Pin('C14', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_down = Pin('C9', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_left = Pin('C15', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_right = Pin('C8', Pin.IN, pull = Pin.PULL_UP_47K, value = True)

###########################读取所需参数############################
ul_normal_kp = find_value(ant_motor.config, "ul_normal_kp")  # type: float
ul_normal_ki = find_value(ant_motor.config, "ul_normal_ki")  # type: float
ul_normal_kd = find_value(ant_motor.config, "ul_normal_kd")  # type: float
ur_normal_kp = find_value(ant_motor.config, "ur_normal_kp")  # type: float
ur_normal_ki = find_value(ant_motor.config, "ur_normal_ki")  # type: float
ur_normal_kd = find_value(ant_motor.config, "ur_normal_kd")  # type: float

# 闭环控制回调
def time_pit2_handler(time):
    # 用于无线串口调试
    
    # 视觉伺服
    # ant_else.wireless.send_str("x: {:<f}, y: {:<f}, speed: {:<f}, yaw: {:<f}, now_yaw: {:<f}\n".format(ant_motor.servo_pid_x.actual, ant_motor.servo_pid_y.actual, ant_plan.my_vision_manager_2.target_rel_speed, ant_plan.my_vision_manager_2.target_rel_yaw, ant_motor.my_car.now_yaw * 180 / MATH.PI))
    # ant_else.wireless.send_str("{:<f},{:<f}\n".format(ant_plan.my_vision_manager_2.target_rel_yaw, ant_plan.my_vision_manager_2.target_rel_yaw_fil))
    
    # 速度环输出波形图调参
    # ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.motor_ul_pid.target, ant_motor.motor_ul_pid.actual, ant_motor.motor_ul_pid.pwm_output, ant_motor.motor_ul_pid.derivative * ant_motor.motor_ul_pid.kd))
    # ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.motor_ur_pid.target, ant_motor.motor_ur_pid.actual, ant_motor.motor_ur_pid.pwm_output, ant_motor.motor_ur_pid.derivative * ant_motor.motor_ur_pid.kd))
    # ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.motor_md_pid.target, ant_motor.motor_md_pid.actual, ant_motor.motor_md_pid.pwm_output, ant_motor.motor_md_pid.derivative * ant_motor.motor_md_pid.kd))
    
    # imu原始数据
    #ant_else.wireless.send_str("acc = {:>6d}, {:>6d}, {:>6d}\n".format(ant_motor.imu_data[0], ant_motor.imu_data[1], ant_motor.imu_data[2]))
    #ant_else.wireless.send_str("gyro = {:>6d}, {:>6d}, {:>6d}\n".format(ant_motor.imu_data[3], ant_motor.imu_data[4], ant_motor.imu_data[5]))
                                                                          
    # 里程计：
    # ant_else.wireless.send_str("ul: {:<f}, ur: {:<f}, md: {:<f}\n".format(ant_motor.my_car.encouder_ul, ant_motor.my_car.encouder_ur, ant_motor.my_car.encouder_md))
    # ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.x_current, ant_motor.my_car.y_current, ant_motor.my_car.encouder_ul, ant_motor.my_car.encouder_ur, ant_motor.my_car.encouder_md))
    # ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.x_current, ant_motor.my_car.y_current, ant_plan.my_plan.real_target_x, ant_plan.my_plan.real_target_y, ant_plan.my_plan.rest_distance, ant_plan.my_plan.target_yaw, ant_motor.my_car.now_yaw, ant_plan.my_plan.arrive_flag, ant_plan.my_plan.transition_flag))
    
    # 速度规划
    #ant_else.wireless.send_str(("v_target: %d, rest_dis: %.3f, dec_speed_index: %d\r\n") % (ant_plan.my_plan.v_target, ant_plan.my_plan.rest_distance, ant_plan.my_plan.dec_speed_index))
    
    # 检测偏航角是否准确
    # ant_else.wireless.send_str("x:{:<f}   y:{:<f}   target_yaw:{:<f}\n".format(ant_motor.my_car.x_current, ant_motor.my_car.y_current, ant_plan.my_plan.target_yaw))
    
    #卡尔曼滤波（速度）
    #ant_else.wireless.send_str("{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.car_speed_x, ant_motor.speed_x_fil.update(ant_motor.my_car.car_speed_x), ant_motor.speed_x_fil2.filtering(ant_motor.my_car.car_speed_x)))
    
    key = read_key()
    if key == None:
        return
    if key == UP:
        arrow_up(key)
    elif key == DOWN:
        arrow_down(key)
    elif key in (LEFT, RIGHT):
        if Current_line == End_line:
            old_page = change_page_to
            if detect_change_page(key):
                if change_page_to != old_page:
                    menu_switch()
                    show_arrow()
        else:
            data_processing(key)
            if change_page_to == 1:
                Menu_Page1_data_show()
            else:
                Menu_Page2_data_show()
            show_arrow()


###############################变量定义###########################
# 当前菜单项
change_page_to = 1  # 将菜单定位到哪一页
Current_line = 1  # 当前行
Start_line, End_line = 1, 5 # 显示的起始行，结束行

LEFT, RIGHT, UP, DOWN = "left", "right", "up", "down"

##############################函数定义###############################
# 保存数据
def update_config_value(file_path, key, new_value):
    lines = []
    found = False

    with open(file_path, 'r') as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith('#') or not stripped:
                lines.append(line) # 保留原样
                continue

            if '=' in stripped:
                k, v = stripped.split('=', 1)
                k = k.strip()
                if k == key:
                    new_line = f"{k} = {new_value}\n"
                    lines.append(new_line)
                    found = True
                else:
                    lines.append(line)
            else:
                lines.append(line)

    with open(file_path, 'w') as f:
        for line in lines:
            f.write(line)
    # 使用方法
    # update_config_value("config.txt", "ul_normal_kp", ul_normal_kp)

# 统一保存数据
def save_data():
    update_config_value("config.txt", "ul_normal_kp", ul_normal_kp)
    update_config_value("config.txt", "ul_normal_ki", ul_normal_ki)
    update_config_value("config.txt", "ul_normal_kd", ul_normal_kd)
    update_config_value("config.txt", "ur_normal_kp", ur_normal_kp)
    update_config_value("config.txt", "ur_normal_ki", ur_normal_ki)
    update_config_value("config.txt", "ur_normal_kd", ur_normal_kd)

# 数据统一处理
def data_processing(key):
    global ul_normal_kp, ul_normal_ki, ul_normal_kd, ur_normal_kp, ur_normal_ki, ur_normal_kd
    if change_page_to == 1:
        if Current_line == 1:
            if key == LEFT:
                ul_normal_kp -= 0.1
            elif key == RIGHT:
                ul_normal_kp += 0.1
        elif Current_line == 2:
            if key == LEFT:
                ul_normal_ki -= 0.1
            elif key == RIGHT:
                ul_normal_ki += 0.1
        elif Current_line == 3:
            if key == LEFT:
                ul_normal_kd -= 0.1
            elif key == RIGHT:
                ul_normal_kd += 0.1
        elif Current_line == 4:
            if key == RIGHT:
                save_data()
    elif change_page_to == 2:
        if Current_line == 1:
            if key == LEFT:
                ur_normal_kp -= 0.1
            elif key == RIGHT:
                ur_normal_kp += 0.1
        elif Current_line == 2:
            if key == LEFT:
                ur_normal_ki -= 0.1
            elif key == RIGHT:
                ur_normal_ki += 0.1
        elif Current_line == 3:
            if key == LEFT:
                ur_normal_kd -= 0.1
            elif key == RIGHT:
                ur_normal_kd += 0.1
        elif Current_line == 4:
            if key == RIGHT:
                save_data()



# 检测按键状态
def read_key(debounce_ms = 40):
    # 检测是否按下（低电平有效）
    if key_left.value() == 0:
        time.sleep_ms(debounce_ms)  # 消抖延时
        if key_left.value() == 0:   # 再次确认
            ant_else.key_test()
            return LEFT
    elif key_right.value() == 0:
        time.sleep_ms(debounce_ms)
        if key_right.value() == 0:
            ant_else.key_test()
            return RIGHT
    elif key_up.value() == 0:
        time.sleep_ms(debounce_ms)
        if key_up.value() == 0:
            ant_else.key_test()
            return UP
    elif key_down.value() == 0:
        time.sleep_ms(debounce_ms)
        if key_down.value() == 0:
            ant_else.key_test()
            return DOWN
    
    return None  # 无按键按下


# 显示箭头
def show_arrow():
    global Start_line,End_line,Current_line
    for i in range(Start_line, End_line + 1):
        if i == Current_line:
            lcd.str16(150, 64 + 32 * (i - 1), "<--", 0xFFFF)
        else:
            lcd.str16(150, 64 + 32 * (i - 1), "   ", 0xFFFF)

# 箭头上移
def arrow_up(key):
    global Start_line,End_line,Current_line
    if key == UP:
        if Current_line > Start_line:
            Current_line -= 1
        else:
            Current_line = End_line
        show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

# 箭头下移
def arrow_down(key):
    global Start_line,End_line,Current_line
    if key == DOWN:
        if Current_line < End_line:
            Current_line += 1
        else:
            Current_line = Start_line
        show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

# 箭头的移动,包含上移和下移
def move_arrow():
    arrow_up()
    arrow_down() 

# 监测指定的跳转页面行是否被按下，并指定目标页面
def detect_change_page(key):
    global change_page_to, Current_line, End_line
    if Current_line == End_line:
        if key == LEFT:
            if change_page_to == 1:
                change_page_to =2
            else:
                change_page_to -= 1
        elif key == RIGHT:
            if change_page_to == 2:
                change_page_to = 1
            else:
                change_page_to += 1
        return True
    else:
        return False


# 第1页菜单数据显示
def Menu_Page1_data_show():
    lcd.str16(60, 64, f"l_p:{ul_normal_kp:.2f}", 0xFFFF)
    lcd.str16(60, 64 + 32 * 1, f"l_i:{ul_normal_ki:.2f}", 0xFFFF)
    lcd.str16(60, 64 + 32 * 2, f"l_d:{ul_normal_kd:.2f}", 0xFFFF)

# 第1页菜单显示
def Menu_Page_1():
    global change_page_to, Start_line,End_line,Current_line
    Start_line,End_line,Current_line=1,5,1
    lcd.clear(0x0000)
    Menu_Page1_data_show()
    lcd.str16(60, 64 + 32 * 3, "save", 0xFFFF)
    lcd.str16(60, 64 + 32 * 4, "turn", 0xFFFF)

# 第2页菜单数据显示
def Menu_Page2_data_show():
    lcd.str16(60, 64, f"r_p:{ur_normal_kp:.2f}", 0xFFFF)
    lcd.str16(60, 64 + 32 * 1, f"r_i:{ur_normal_ki:.2f}", 0xFFFF)
    lcd.str16(60, 64 + 32 * 2, f"r_d:{ur_normal_kd:.2f}", 0xFFFF)
    
# 第2页菜单显示
def Menu_Page_2():
    global change_page_to, Start_line,End_line,Current_line
    Start_line,End_line,Current_line=1,5,1
    lcd.clear(0x0000)
    Menu_Page2_data_show()
    lcd.str16(60, 64 + 32 * 3, "save", 0xFFFF)
    lcd.str16(60, 64 + 32 * 4, "turn", 0xFFFF)   



#函数：菜单选择与切换
def menu_switch():
    if(change_page_to == 1):
        Menu_Page_1()
    elif(change_page_to == 2):
        Menu_Page_2()


# 主函数部分

# === 初始显示 ===
Menu_Page_1()
show_arrow()
"""
# === 主循环 ===
while True:
    last_change_page_to = change_page_to
    key = read_key()
    if key == UP:
        arrow_up()
    elif key == DOWN:
        arrow_down()
    elif key in (LEFT, RIGHT):
        data_processing()

    if detect_change_page():
        if change_page_to != last_change_page_to:
            menu_switch()
            show_arrow()
    
    time.sleep_ms(50)
"""