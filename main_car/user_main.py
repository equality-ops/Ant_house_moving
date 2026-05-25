# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容 
from micropython import const
from machine import *
from display import *
from seekfree import MOTOR_CONTROLLER, IMU660RX, KEY_HANDLER
from smartcar import ticker, encoder
import ant_vision
import ant_plan
import ant_motor
import ant_else
import ant_menu
import math

# 包含 gc 与 time 类
import gc
import time
###################################【变量定义及初始化】###################################
PI = const(3.1415926)

# 多路复用时间计数器
counter = 0      # type: int
# 是否按下启动按键标志位
if_press_start_key = False
# 是否成功启动标志位
start_flag = False
# 是否操控从车提前到达目标点就位标志位
if_send_preparing_path = False
# 任务阶段变量
DEPART = 0       # 出发（离开发车区）
DOWN = 1         # 位于矩形下边沿
UP = 2           # 位于矩形上边沿
CHECK = 3        # 检验阶段（检查是否搬运完所有物体）
RETURN_WORK = 4  # 返回阶段（搬运完所有物体后返回起点）
# 小车位置（用于apriltag矫正模式）
DOWN_LEFT = 0
DOWN_RIGHT = 1
UP_LEFT = 2
UP_RIGHT = 3

##################################【实例对象构建及初始化】##################################
"""""""""核心板与学习板接口初始化"""""""""
# 核心板上 C4 是 LED
# 学习板上 D9  对应一号拨码开关
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 构造输入电压分压检测电路接口
power_adc = ADC('B27')

"""蜂鸣器初始化"""
beep = Pin('D24', Pin.OUT, value = False)

"""异步串口通信初始化"""
my_uart6 = UART(5)
my_uart6.init(115200)

"""无线串口通信初始化"""
my_uart3 = UART(2)
my_uart3.init(115200)

"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = False)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = False)

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("D13", "D14", True)
encoder_ur = encoder("D16", "D15", True)
encoder_md = encoder("C2" , "C3" ,True)

# IMU初始化
imu = IMU660RX()

# tof深度传感器初始化
# tof = DL1X()

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
lcd.mode(2)
lcd.clear(0x0000)

# 与定时器2周期一致，都为53ms
key = KEY_HANDLER(53)
key_data = key.get()
# 按键对应的数据接口
"""
key_up:     key_data[1]
key_down:   key_data[0]
enc_key:    key_data[2]
key_run:    key_data[3] 
"""

# 菜单编码器初始化
enc_rotation = encoder("C0", "C1", True)

"""""""""创建对象"""""""""
# 创建状态机对象
my_state = ant_plan.StateMachine()

# 创建蜂鸣器对象
my_beep = ant_else.beep(beep)

#【文件读取】
# 从main_config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/main_config.txt")
my_flash_sys.phase_config()

# 创建指令管理对象
my_order_manager = ant_else.order_manager(my_uart6)

# 创建openart串口解析对象
my_art_protocol = ant_else.UARTProtocol(my_uart6)

# 创建主从车无线串口通信对象
my_main_protocol = ant_else.LinkProtocol(my_uart3)

# 创建pid参数对象
pid_data = ant_motor.PID_data(my_flash_sys)

# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_ur = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_md = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_gyroz = ant_motor.SlipAveragingFilter(1)  # 滤波窗口为1个

# 创建小车自转角滤波器对象（已弃用）
car_yaw_fil = ant_motor.SlipAveragingFilter(1)
# 创建视觉伺服正余弦滤波对象
sin_servo_fil = ant_motor.SlipAveragingFilter(5)    
cos_servo_fil = ant_motor.SlipAveragingFilter(5)

# 创建姿态数据对象
pose_data = ant_motor.PoseData(my_flash_sys, my_uart3, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz)

# 创建电机pid对象和角度pid对象
motor_ul_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ul)
motor_ur_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ur)
motor_md_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_md)
angle_pid = ant_motor.AnglePositionPID(my_flash_sys)
servo_pid = ant_motor.ServoPID(my_flash_sys)

# 创建小车姿态对象
my_car = ant_motor.CarPose(my_flash_sys, my_state, pose_data, car_yaw_fil, angle_pid, motor_ul_pid, motor_ur_pid, motor_md_pid,
                        motor_ul, motor_ur, motor_md)

# 创建路径规划数据对象
plan_data = ant_plan.PlanData(my_flash_sys)

# 创建路径规划对象
my_path = ant_plan.PathPlan(plan_data, my_car)

# 创建规划（路径和速度）对象
my_plan = ant_plan.NavigationPlan(my_flash_sys, plan_data, my_car, my_state, my_order_manager, my_uart3, my_beep, my_art_protocol)

# 创建视觉伺服管理对象2
my_vision_manager = ant_vision.VisionManager(my_flash_sys, my_beep, pose_data,  angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, my_car, my_art_protocol, my_order_manager, my_plan, my_state)

# 搬运控制类
my_moving = ant_vision.MoveControl(my_beep, my_car, my_plan, plan_data, my_vision_manager, my_state, my_main_protocol, my_art_protocol, my_order_manager)

# 创建菜单对象
my_menu = ant_menu.Menu(my_flash_sys, my_beep, lcd, enc_rotation, key_data, key)
###################################【函数定义】###################################
# 电机驱动函数
def set_motor(motor, duty) -> None:
    motor.duty(duty)

# 是否成功读取文件和开启定时器检查函数
def detect_if_normal() -> None:
    led.toggle()
    my_beep.test()

# 检测电源电压函数
def voltage_detect(limit_min: float) -> None:
    power_adc_value = power_adc.read_u16()
    power_voltage = power_adc_value / 65535 * 3.3 * 11
    print(f"The current power supply voltage is {power_voltage}!")
    if power_voltage <= limit_min:
        print(f"The power supply voltage: {power_voltage} is too low!")
        my_beep.beep_warn()

# 角度环计算函数
def angle_pid_compute():
    # 计算z轴的目标速度
    angle_pid.compute_pid(my_car.turn_angle_target, my_car.now_yaw * 180 / PI)

# 用于主车启动的函数
def main_start():
    global current_time, last_left_time, start_flag, if_press_start_key
    if start_flag == False:
        if if_press_start_key == False:
            if key_data[3] != 0 and switch2.value() == 0:
                # 清除按键状态
                key.clear(4)
                my_beep.key_test()
                # 测试，记得双车通信时要打开
                my_main_protocol.send_start()
                if_press_start_key = True
        else:   
            # 测试，此时只调试主车，双车正常通信时需要解注释  
            # if my_main_protocol.get_slave_state() == "ready":
                # 初始化小车坐标
                my_car.x_current = plan_data.fixed_point[0][0]
                my_car.y_current = plan_data.fixed_point[0][1]
                my_state.state_work = DOWN
                my_state.state = my_state.READY_NAVIGATE
                start_flag = True
                # 延时一秒避免零漂校准不准确
                time.sleep_ms(1000)
                # 打开定时器1和3
                pit1_start()
                pit3_start()
                # 检测是否正常初始化所有
                detect_if_normal()


# 小车姿态总控制函数
def master_control():
    if my_state.state in [my_state.NAVIGATE, my_state.READY_NAVIGATE, my_state.RETURN, my_state.STOP, my_state.SCAN]:
        my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.MOVE:
        if my_moving.current_state == my_state.ORBIT:
            my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
        elif my_moving.current_state == my_state.SERVO:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        elif my_moving.current_state in [my_state.ADJUST, my_state.MOVE]:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.SERVO:
        # 未丢失物体时正常进行视觉伺服控制，丢失物体时进行矩形轨迹的导航控制
        if my_vision_manager.if_lost_object == False:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        else:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.CALIBRATE:
        if my_vision_manager.if_ready_calibrate == False:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        else:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.ORBIT:
        my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)


# 调试电机速度环pid函数
def show_speed_PID_test():
    global counter
    counter += 1
    # 调试搬运模式下的pid参数
    # my_state.state = my_state.MOVE
    # motor_ul_pid.compute_pid(60, pose_data.encoder_data_ul)
    # motor_ur_pid.compute_pid(300, pose_data.encoder_data_ur)
    # motor_md_pid.compute_pid(300, pose_data.encoder_data_md)

    # 测试不同速度下的pid参数切换情况
    if counter >= 8000:
        counter = 0
    elif counter >= 6000:
        motor_ul_pid.compute_pid(180, pose_data.encoder_data_ul)
    elif counter >= 4000:
        motor_ul_pid.compute_pid(-20, pose_data.encoder_data_ul)
    elif counter >= 2000:
        motor_ul_pid.compute_pid(-120, pose_data.encoder_data_ul)
    else:
        motor_ul_pid.compute_pid(250, pose_data.encoder_data_ul)
    
# 视觉伺服测试函数
def test_vision_servo():
    if my_state.state == my_state.READY_NAVIGATE:
        if my_vision_manager.if_send_order == False:
            my_order_manager.mode_target()
            my_vision_manager.if_send_order = True

        target_point = my_art_protocol.coordinate_receive()
        if target_point:
            my_vision_manager.ready_servo_and_orbit(target_point)
            # my_vision_manager.calculate_dist(target_point[0], target_point[1], 'far')
            my_vision_manager.if_send_order = False
            my_state.state = my_state.SERVO
    elif my_state.state == my_state.SERVO:
        my_vision_manager.visual_servo_control()
        if my_vision_manager.if_finish_servo == True:
            my_order_manager.mode_target()
            my_state.state = my_state.ORBIT
            my_vision_manager.reset_orbit_angle()
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(140.0)
        if my_vision_manager.if_finish_orbit == True:
            my_state.state = my_state.STOP
            my_plan.reset_navigate_angle()
    elif my_state.state == my_state.STOP:
        my_plan.stop()

# 测试小车搬运状态
def test_moving():
    global counter
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.NAVIGATE
    elif my_state.state == my_state.NAVIGATE:
        my_plan.navigate(path = [[160.0, 80.0]])
        if my_plan.if_finish_navigate == True:
            if my_vision_manager.if_send_order == False:
                my_order_manager.mode_target()
                my_vision_manager.if_send_order = True 
            target_point = my_art_protocol.coordinate_receive()
            if target_point and (target_point[2] in ['T', 'S', 'E', 'W', 'B']):
                my_vision_manager.ready_servo_and_orbit(target_point)
                my_plan.reset_navigate()  # 重置导航相关变量
                my_state.state = my_state.SERVO
                my_vision_manager.if_send_order = False 
    elif my_state.state == my_state.SERVO:
        my_vision_manager.visual_servo_control()
        if my_vision_manager.if_finish_servo == True:
            counter += 1
            # 延时500ms
            if counter >= 50:
                my_vision_manager.if_finish_servo = False
                my_moving.ready_move()
                my_uart3.write(f"state: {my_moving.current_state},moving_pt: {my_moving.moving_point},angle_buffer: {my_moving.angle_buffer}\n")
                my_order_manager.mode_target()
                my_state.state = my_state.MOVE
                # 测试
                my_beep.test()
    elif my_state.state == my_state.MOVE:
        my_moving.moving()
        if my_moving.if_finish_move == True:
            my_moving.if_finish_move = False
            my_path.plan_path(plan_data.fixed_point[0][0], plan_data.fixed_point[0][1])
            my_state.state = my_state.RETURN
    elif my_state.state == my_state.RETURN:
        my_plan.navigate(path = my_path.ready_path, target_turn_angle = 0.0)
        if my_plan.if_finish_navigate == True:
            my_plan.reset_navigate()  # 重置导航相关变量
            my_state.state = my_state.STOP
    elif my_state.state == my_state.STOP:
        my_plan.stop()

orbit_angle = 240.0
def test_orbit():
    global orbit_angle, counter, direct_flag
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.ORBIT
        my_vision_manager.object_radius = 16.0
        my_vision_manager.current_servo_object = 'S'
        my_order_manager.mode_target()
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(orbit_angle)
        if my_vision_manager.if_finish_orbit == True:
            counter += 1
            if counter >= 100:
                counter = 0
                my_moving.reset_orbit()
                orbit_angle += 120.0
                orbit_angle = (orbit_angle + 180) % 360 - 180

# 边线和apriltag码校准测试程序
def test_apriltag_calibrate():
    global counter
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.NAVIGATE
    elif my_state.state == my_state.NAVIGATE:
        my_plan.navigate(path = [[160.0, 220.0], [0.0, 120.0], [130.0, -5.0]], target_turn_angle = 40.0)
        if my_plan.if_finish_navigate == True:  
            my_plan.reset_navigate()
            my_state.state = my_state.CALIBRATE
            my_vision_manager.assist_car_pos = [160.0, 0.0]
            my_vision_manager.car_position = 'R'
    elif my_state.state == my_state.CALIBRATE:
        my_vision_manager.apriltag_calibrate_control()
        if my_vision_manager.if_finish_calibrate == True:
            counter += 1
            # 延时1s
            if counter >= 100:
                counter = 0
                my_vision_manager.reset_apriltag_calibrate()
                my_plan.reset_navigate_angle()
                my_state.state = my_state.STOP
    elif my_state.state == my_state.RETURN:
        my_plan.navigate(path = [[0.0, 0.0]], target_turn_angle = 0.0)
        if my_plan.if_finish_navigate == True:  
            my_plan.reset_navigate()
            my_state.state = my_state.STOP

spin_angle = 90.0
def test_spin():
    global spin_angle, counter
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.NAVIGATE
    elif my_state.state == my_state.NAVIGATE:
        my_plan.navigate(target_turn_angle = spin_angle)
        if my_plan.if_finish_navigate == True:
            counter += 1
            if counter >= 100:
                counter = 0
                my_plan.reset_navigate()
                spin_angle += 90.0
                spin_angle = (spin_angle + 180) % 360 - 180

# 测试
# 设置pid参数
def set_pid_params():
    if my_state.state == my_state.MOVE or my_state.state == my_state.ORBIT or my_state.state == my_state.CALIBRATE:
        motor_ul_pid.set_pid_params(pid_data.ul_move_kp, pid_data.ul_move_ki, pid_data.ul_move_kd)
        motor_ur_pid.set_pid_params(pid_data.ur_move_kp, pid_data.ur_move_ki, pid_data.ur_move_kd)
        motor_md_pid.set_pid_params(pid_data.md_move_kp, pid_data.md_move_ki, pid_data.md_move_kd)
    else:
        brake_threshold = 5.0
        target_limit = 1.0
        # 初始化pid参数（线性回归）
        if motor_ul_pid.target <= target_limit and abs(motor_ul_pid.nowError) >= brake_threshold:
            motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
        elif abs(motor_ul_pid.target) >= 290:
            motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
        elif abs(motor_ul_pid.target) >= 150:
            now_ul_kp = pid_data.ul_mid_kp + (pid_data.ul_high_kp - pid_data.ul_mid_kp) * (abs(motor_ul_pid.target) - 150) / 140
            now_ul_ki = pid_data.ul_mid_ki + (pid_data.ul_high_ki - pid_data.ul_mid_ki) * (abs(motor_ul_pid.target) - 150) / 140
            motor_ul_pid.set_pid_params(now_ul_kp, now_ul_ki, pid_data.ul_mid_kd)
        elif abs(motor_ul_pid.target) >= 70:
            now_ul_kp = pid_data.ul_low_kp + (pid_data.ul_mid_kp - pid_data.ul_low_kp) * (abs(motor_ul_pid.target) - 70) / 80
            now_ul_ki = pid_data.ul_low_ki + (pid_data.ul_mid_ki - pid_data.ul_low_ki) * (abs(motor_ul_pid.target) - 70) / 80
            motor_ul_pid.set_pid_params(now_ul_kp, now_ul_ki, pid_data.ul_low_kd)
        else:
            motor_ul_pid.set_pid_params(pid_data.ul_low_kp, pid_data.ul_low_ki, pid_data.ul_low_kd)
            
        if motor_ur_pid.target <= target_limit and abs(motor_ur_pid.nowError) >= brake_threshold:
            motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
        elif abs(motor_ur_pid.target) >= 290:
            motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
        elif abs(motor_ur_pid.target) >= 150:
            now_ur_kp = pid_data.ur_mid_kp + (pid_data.ur_high_kp - pid_data.ur_mid_kp) * (abs(motor_ur_pid.target) - 150) / 140
            now_ur_ki = pid_data.ur_mid_ki + (pid_data.ur_high_ki - pid_data.ur_mid_ki) * (abs(motor_ur_pid.target) - 150) / 140
            motor_ur_pid.set_pid_params(now_ur_kp, now_ur_ki, pid_data.ur_mid_kd)
        elif abs(motor_ur_pid.target) >= 70:
            now_ur_kp = pid_data.ur_low_kp + (pid_data.ur_mid_kp - pid_data.ur_low_kp) * (abs(motor_ur_pid.target) - 70) / 80
            now_ur_ki = pid_data.ur_low_ki + (pid_data.ur_mid_ki - pid_data.ur_low_ki) * (abs(motor_ur_pid.target) - 70) / 80
            motor_ur_pid.set_pid_params(now_ur_kp, now_ur_ki, pid_data.ur_low_kd)
        else:
            motor_ur_pid.set_pid_params(pid_data.ur_low_kp, pid_data.ur_low_ki, pid_data.ur_low_kd)

        if motor_md_pid.target <= target_limit and abs(motor_md_pid.nowError) >= brake_threshold:
            motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
        elif abs(motor_md_pid.target) >= 290:
            motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
        elif abs(motor_md_pid.target) >= 150:
            now_md_kp = pid_data.md_mid_kp + (pid_data.md_high_kp - pid_data.md_mid_kp) * (abs(motor_md_pid.target) - 150) / 140
            now_md_ki = pid_data.md_mid_ki + (pid_data.md_high_ki - pid_data.md_mid_ki) * (abs(motor_md_pid.target) - 150) / 140
            motor_md_pid.set_pid_params(now_md_kp, now_md_ki, pid_data.md_mid_kd)
        elif abs(motor_md_pid.target) >= 70:
            now_md_kp = pid_data.md_low_kp + (pid_data.md_mid_kp - pid_data.md_low_kp) * (abs(motor_md_pid.target) - 70) / 80
            now_md_ki = pid_data.md_low_ki + (pid_data.md_mid_ki - pid_data.md_low_ki) * (abs(motor_md_pid.target) - 70) / 80
            motor_md_pid.set_pid_params(now_md_kp, now_md_ki, pid_data.md_low_kd)
        else:
            motor_md_pid.set_pid_params(pid_data.md_low_kp, pid_data.md_low_ki, pid_data.md_low_kd)

""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    set_pid_params()
    
    # 更新小车姿态
    my_car.update_pose()
    
    # 全向定位测试程序
    # test_global_localization()
    
    # 速度环测试
    # show_speed_PID_test()
    
    # 总控制函数
    master_control()

    # 设置电机pwm输出
    if_stop_pwm = (my_state.state == my_state.CALIBRATE and my_vision_manager.if_finish_calibrate == False and my_vision_manager.if_ready_calibrate == True)
    if if_stop_pwm:
        my_car.pwm_stop()
    else:
        my_car.set_motor_pwm()



# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
    # 角度环计算（10ms）
    angle_pid_compute()

    # 任务执行机
    # task_machine()
    # collaborative_task_machine()

    # 全向定位测试程序
    """
    if my_state.state == my_state.READY_NAVIGATE:
        # my_path.plan_path(245.0, 56.0)
        # my_uart3.write(f"ready_path: {my_path.ready_path}\n")
        my_state.state = my_state.NAVIGATE
    elif my_state.state == my_state.NAVIGATE:
        my_plan.navigate(path = [[160.0, 80.0], [240.0, 80.0], [240.0, 0.0], [160.0, 0.0]])
        # my_main_protocol.send_pose(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        if my_plan.if_finish_navigate == True:
            my_plan.reset_navigate()
            my_state.state = my_state.STOP
            my_beep.test()
    elif my_state.state == my_state.STOP:
        my_plan.stop()
    """
    # 视觉伺服测试程序
    # test_vision_servo()

    # 搬运控制测试程序
    # test_moving()

    # 边线和apriltag码校准测试程序
    test_apriltag_calibrate()

    # 环绕物体测试程序
    # test_orbit()

    # 自转测试程序
    # test_spin()

    pass


# 定时器2中断回调函数
# 用于无线串口调试和发车启动
def time_pit2_handler(time):
    """用于无线串口调试"""
    # 发车启动函数
    main_start()
    
    if start_flag == False:
        # 读取按键（中断中避免阻塞，快速返回）
        key = my_menu.read_key()
        my_menu.handle_key_from_interrupt(key)

    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_vision_manager.orbit_speed, motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd, motor_ur_pid.integral))

# 定时器1初始化（中断回调函数在 ant_motor 中）
def pit1_start():
    global imu_data
    pit1 = ticker(1)
    pit1.capture_list(imu, encoder_ul, encoder_ur, encoder_md)
    # 进行IMU零漂校准并将imu_data与定时器1的底层采集绑定
    pose_data.init_bias()
    pit1.callback(time_pit1_handler)
    pit1.start(my_flash_sys.find_value("motor_control_T"))

# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    pit2 = ticker(2)
    pit2.callback(time_pit2_handler)
    pit2.capture_list(key)
    pit2.start(my_flash_sys.find_value("uart_and_menu_T"))

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    pit3 = ticker(3)
    pit3.callback(time_pit3_handler)
    pit3.start(my_flash_sys.find_value("plan_calculate_T"))

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.2)

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