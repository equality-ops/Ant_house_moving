# 包含 gc 与 time 类
import gc
import time
import os
from micropython import const
gc.collect()
# 从 machine 库包含所有内容 
from machine import *
gc.collect()
from display import *
gc.collect()
from seekfree import MOTOR_CONTROLLER, IMU660RX, KEY_HANDLER, BLDC_CONTROLLER
gc.collect()
from smartcar import ticker, encoder
my_uart3 = UART(2)
my_uart3.init(115200)
gc.collect()
my_uart_debug = UART(7)
my_uart_debug.init(115200)
my_uart_debug.write('1')
gc.collect()
import ant_plan
gc.collect()
import ant_else
gc.collect()
# 与定时器2周期一致，都为53ms
pin_obj = Pin("C15", Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
del(pin_obj)
pin_obj = Pin("C15", Pin.OPEN_DRAIN, pull = Pin.PULL_UP, value = True)
if_menu = False
if not pin_obj.value():#进入调试模式
    if_menu = True
    import ant_menu
    gc.collect()
else:
    import ant_vision
    gc.collect()
    import ant_boundary_plan
    gc.collect()
    import ant_motor
    gc.collect()
    import ant_move
    gc.collect()
    import ant_task
    gc.collect()
    import ant_pid
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
RETREAT = const(10) 

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
# 创建蜂鸣器对象
my_beep = ant_else.beep(beep)

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
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty=0, invert=False)
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

# tof深度传感器初始化
# tof = DL1X()
# 与定时器2周期一致，都为53ms
key = KEY_HANDLER(53)
key_data = key.get()
"""菜单与显示屏初始化"""
# 新建LCD实例并初始化
if if_menu:
    cs = Pin('B29' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
    cs.high()
    cs.low()
    rst = Pin('B31' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
    dc  = Pin('B5' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
    blk = Pin('C21' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 1)
'''
drv = LCD_Drv(SPI_INDEX = 2, BAUDRATE = 60000000, DC_PIN = dc, RST_PIN = rst, LCD_TYPE = LCD_Drv.LCD200_TYPE)
lcd = LCD(drv)
lcd.color(0xFFFF, 0x0000)
lcd.mode(2)
lcd.clear(0x0000)
'''

# 按键对应的数据接口
"""
key_up:     key_data[1]
key_down:   key_data[0]
enc_key:    key_data[2]
key_run:    key_data[3] 
"""

# 菜单编码器初始化
if if_menu:
    enc_rotation = encoder("C0", "C1", True)
else:
    """""""""创建对象"""""""""
    # 创建状态机对象
    my_state = ant_plan.StateMachine()
    #【文件读取】
    # 从main_config.txt中读取保存所有的参数并保存到config字典中
    my_flash_sys = ant_else.flash_system(my_beep, "/flash/main_config.txt")
    my_flash_sys.phase_config()
    my_flash_sys.check_list_format()

    # 创建无刷风扇控制对象
    my_fan = ant_motor.FanControl(my_flash_sys, fan, my_state)

    # 创建光电管控制对象
    my_photo = ant_motor.PhotoControl(my_flash_sys, my_beep, photo)

    # 创建指令管理对象
    my_order_manager = ant_else.order_manager(my_flash_sys, my_uart6)

    # 创建openart串口解析对象
    my_art_protocol = ant_else.UARTProtocol(my_uart6)

    # 创建主从车无线串口通信对象
    my_main_protocol = ant_else.LinkProtocol(my_uart3)

    # 创建主辅车无线串口通信对象
    my_assist_protocol = ant_else.AssistLinkProtocol(my_uart8)

    # 创建pid参数对象
    pid_data = ant_pid.PID_data(my_flash_sys)

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
    motor_ul_pid = ant_pid.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ul)
    motor_ur_pid = ant_pid.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ur)
    motor_md_pid = ant_pid.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_md)
    angle_pid = ant_pid.AnglePositionPID(my_flash_sys)
    servo_pid = ant_pid.ServoPID(my_flash_sys)

    # 创建小车姿态对象
    my_car = ant_motor.CarPose(my_flash_sys, my_state, pose_data, car_yaw_fil, angle_pid, motor_ul_pid, motor_ur_pid, motor_md_pid,
                            motor_ul, motor_ur, motor_md)

    # 创建路径规划数据对象
    plan_data = ant_plan.PlanData(my_flash_sys)

    # 创建路径规划对象
    my_path = ant_plan.PathPlan(plan_data, my_car)

    # 创建规划（路径和速度）对象
    my_plan = ant_plan.NavigationPlan(my_flash_sys, plan_data, my_fan, my_car, my_state, my_order_manager, my_uart3, my_beep, my_art_protocol)


    move_plan = ant_boundary_plan.BoundaryPathPlanner(plan_data, my_car, my_path)
    gc.collect()
    # 创建视觉伺服管理对象2
    my_vision_manager = ant_vision.VisionManager(my_flash_sys, my_beep, pose_data,  angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, my_car, my_art_protocol, my_order_manager, my_plan, my_state)

    # 搬运控制类
    my_moving = ant_move.MoveControl(my_flash_sys,my_beep, my_photo, my_car, my_plan,my_path, plan_data,move_plan, my_vision_manager, my_state, my_main_protocol, my_art_protocol, my_order_manager, my_assist_protocol)
    
    my_obj_plan = ant_boundary_plan.objects_planner(plan_data,my_car,my_plan,move_plan)
    # 任务及类
    my_task = ant_task.TaskController(my_obj_plan,my_beep, my_state, my_uart3, my_car, my_path, my_plan, my_vision_manager,  my_moving, plan_data, my_order_manager, my_art_protocol,  my_main_protocol, my_assist_protocol,my_uart_debug)

# 创建菜单对象
# my_menu = ant_menu.Menu(my_flash_sys, my_beep, lcd, enc_rotation, key_data, key)

gc.collect()
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
    global current_time, last_left_time, start_flag, if_press_start_key, pit2
    if start_flag == False:
        if if_press_start_key == False:
            if key_data[3] != 0:
                # 清除按键状态
                key.clear(4)
                my_beep.key_test()
                # 测试，记得双车通信时要打开
                my_main_protocol.send_start()
                if_press_start_key = True
        else:   
            # 测试，此时只调试主车，双车正常通信时需要解注释  
            if my_main_protocol.get_slave_state() == "ready":
                # 此时开启无刷负压风扇
                my_fan.set_fan_signal()
                # 初始化小车坐标
                my_car.x_current = plan_data.fixed_point[0][0]
                my_car.y_current = plan_data.fixed_point[0][1]
                my_state.state = READY_NAVIGATE
                start_flag = True
                # 延时1.5秒避免零漂校准不准确
                time.sleep_ms(1500)
                # 打开定时器1和3
                pit1_start()
                pit3_start()
                # 检测是否正常初始化所有
                detect_if_normal()

# 调试电机速度环pid函数
def show_speed_PID_test():
    global counter
    # counter += 1
    # motor_ul_pid.compute_pid(170, pose_data.encoder_data_ul)
    motor_ur_pid.compute_pid(220, pose_data.encoder_data_ur)
    # motor_md_pid.compute_pid(170, pose_data.encoder_data_md)
    '''
    # 测试不同速度下的pid参数切换情况
    if counter >= 8000:
        counter = 0
    elif counter >= 6000:
        motor_ur_pid.compute_pid(50, pose_data.encoder_data_ur)
    elif counter >= 4000:
        motor_ur_pid.compute_pid(-200, pose_data.encoder_data_ur)
    elif counter >= 2000:
        motor_ur_pid.compute_pid(-100, pose_data.encoder_data_ur)
    else:
        motor_ur_pid.compute_pid(250, pose_data.encoder_data_ur)
    '''
orbit_angle = 240.0
spin_angle = 90.0

# 小车姿态总控制函数
def master_control():
    if my_state.state in [NAVIGATE, READY_NAVIGATE, RETURN, STOP, SCAN, RETREAT]:
        my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == MOVE:
        if my_moving.current_state == ORBIT:
            my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
        elif my_moving.current_state in [SERVO, ADJUST]:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        elif my_moving.current_state == NAVIGATE:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        elif my_moving.current_state == MOVE:
            if my_plan.fitting_path_:my_car.move_ctrl(my_plan.target_v, my_plan.fit_target_yaw, my_plan.turn_angle_target)
            else:my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state in [SERVO, ADJUST]:
        # 未丢失物体时正常进行视觉伺服控制，丢失物体时进行矩形轨迹的导航控制
        if my_vision_manager.if_lost_object == False:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        else:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == ORBIT:
        my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
    
# 设置pid参数
def set_pid_params():
    if my_state.state == MOVE:
        motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
        motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
        motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
    else:
        brake_threshold = 20.0
        target_limit = 1.0
        # 初始化pid参数（线性回归）
        if abs(motor_ul_pid.target) <= target_limit and abs(motor_ul_pid.nowError) >= brake_threshold:
            motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
        elif abs(motor_ul_pid.target) >= 180:
            motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
        elif abs(motor_ul_pid.target) >= 120:
            now_ul_kp = pid_data.ul_mid_kp + (pid_data.ul_high_kp - pid_data.ul_mid_kp) * (abs(motor_ul_pid.target) - 120) / 60
            now_ul_ki = pid_data.ul_mid_ki + (pid_data.ul_high_ki - pid_data.ul_mid_ki) * (abs(motor_ul_pid.target) - 120) / 60
            motor_ul_pid.set_pid_params(now_ul_kp, now_ul_ki, pid_data.ul_mid_kd)
        elif abs(motor_ul_pid.target) >= 50:
            now_ul_kp = pid_data.ul_low_kp + (pid_data.ul_mid_kp - pid_data.ul_low_kp) * (abs(motor_ul_pid.target) - 50) / 70
            now_ul_ki = pid_data.ul_low_ki + (pid_data.ul_mid_ki - pid_data.ul_low_ki) * (abs(motor_ul_pid.target) - 50) / 70
            motor_ul_pid.set_pid_params(now_ul_kp, now_ul_ki, pid_data.ul_low_kd)
        else:
            motor_ul_pid.set_pid_params(pid_data.ul_low_kp, pid_data.ul_low_ki, pid_data.ul_low_kd)
            
        if abs(motor_ur_pid.target) <= target_limit and abs(motor_ur_pid.nowError) >= brake_threshold:
            motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
        elif abs(motor_ur_pid.target) >= 180:
            motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
        elif abs(motor_ur_pid.target) >= 120:
            now_ur_kp = pid_data.ur_mid_kp + (pid_data.ur_high_kp - pid_data.ur_mid_kp) * (abs(motor_ur_pid.target) - 120) / 60
            now_ur_ki = pid_data.ur_mid_ki + (pid_data.ur_high_ki - pid_data.ur_mid_ki) * (abs(motor_ur_pid.target) - 120) / 60
            motor_ur_pid.set_pid_params(now_ur_kp, now_ur_ki, pid_data.ur_mid_kd)
        elif abs(motor_ur_pid.target) >= 50:
            now_ur_kp = pid_data.ur_low_kp + (pid_data.ur_mid_kp - pid_data.ur_low_kp) * (abs(motor_ur_pid.target) - 50) / 70
            now_ur_ki = pid_data.ur_low_ki + (pid_data.ur_mid_ki - pid_data.ur_low_ki) * (abs(motor_ur_pid.target) - 50) / 70
            motor_ur_pid.set_pid_params(now_ur_kp, now_ur_ki, pid_data.ur_low_kd)
        else:
            motor_ur_pid.set_pid_params(pid_data.ur_low_kp, pid_data.ur_low_ki, pid_data.ur_low_kd)

        if abs(motor_md_pid.target) <= target_limit and abs(motor_md_pid.nowError) >= brake_threshold:
            motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
        elif abs(motor_md_pid.target) >= 180:
            motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
        elif abs(motor_md_pid.target) >= 120:
            now_md_kp = pid_data.md_mid_kp + (pid_data.md_high_kp - pid_data.md_mid_kp) * (abs(motor_md_pid.target) - 120) / 60
            now_md_ki = pid_data.md_mid_ki + (pid_data.md_high_ki - pid_data.md_mid_ki) * (abs(motor_md_pid.target) - 120) / 60
            motor_md_pid.set_pid_params(now_md_kp, now_md_ki, pid_data.md_mid_kd)
        elif abs(motor_md_pid.target) >= 50:
            now_md_kp = pid_data.md_low_kp + (pid_data.md_mid_kp - pid_data.md_low_kp) * (abs(motor_md_pid.target) - 50) / 70
            now_md_ki = pid_data.md_low_ki + (pid_data.md_mid_ki - pid_data.md_low_ki) * (abs(motor_md_pid.target) - 50) / 70
            motor_md_pid.set_pid_params(now_md_kp, now_md_ki, pid_data.md_low_kd)
        else:
            motor_md_pid.set_pid_params(pid_data.md_low_kp, pid_data.md_low_ki, pid_data.md_low_kd)

# 任务机执行函数
def task_machine():
    my_task.run()


""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    # 更新pid参数
    set_pid_params()
    
    # 更新小车姿态
    my_car.update_pose()
    
    # 全向定位测试程序
    # test_global_localization()
    
    # 速度环测试
    # show_speed_PID_test()
    
    # 角度环测试
    # complete_angle_circle()

    # 总控制函数
    master_control()

    # 设置电机pwm输出
    my_car.set_motor_pwm()
    # 更新负压风扇的高电平时间
    """
    if my_fan.if_fan:
        my_fan.test_fan(1200)
        my_fan.if_fan = False
    """
# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
    # 角度环计算（10ms）
    angle_pid_compute()

    # 任务执行机
    # task_machine()

    # 全向定位测试程序
    # 视觉伺服测试程序
    # test_vision_servo()

    # 搬运控制测试程序
    # test_moving()

    # 边线和apriltag码校准测试程序
    # test_apriltag_calibrate()

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

    """
    if start_flag == False:
        # 读取按键（中断中避免阻塞，快速返回）
        key = my_menu.read_key()
        my_menu.handle_key_from_interrupt(key)
    """
    my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.gyro_z}\n")
    # my_uart3.write("{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current))
    # my_uart3.write(f"{my_car.alpha_x},{my_car.alpha_y}\r\n")
    # my_uart3.write(f"{servo_pid.actual_x},{servo_pid.target_x},{servo_pid.pwm_output_x},{servo_pid.actual_y},{servo_pid.target_y},{servo_pid.pwm_output_y},{my_vision_manager.target_rel_yaw}\n")
    # my_uart3.write(f"{my_car.now_yaw * 180 / PI}\n")
    # my_uart3.write(f"{my_vision_manager.target_rel_speed},{my_vision_manager.target_rel_yaw},{my_vision_manager.target_rel_turn_angle},{my_car.now_yaw * 180 / PI}\n")
    # my_uart3.write(f"{my_state.state}\n")
    # my_uart3.write(f"{my_moving.current_state}\r\n")

# 定时器1初始化（中断回调函数在 ant_motor 中）
def pit1_start():
    global imu_data, pit1
    pit1.capture_list(imu, encoder_ul, encoder_ur, encoder_md)
    # 进行IMU零漂校准并将imu_data与定时器1的底层采集绑定
    pose_data.init_bias()
    pit1.callback(time_pit1_handler)
    pit1.start(my_flash_sys.find_value("motor_control_T"))

# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    global pit2
    pit2.callback(time_pit2_handler)
    pit2.capture_list(key)
    pit2.start(my_flash_sys.find_value("uart_and_menu_T"))

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    global pit3
    pit3.callback(time_pit3_handler)
    pit3.start(my_flash_sys.find_value("plan_calculate_T"))

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.2)

# 打开定时器
if if_menu:
    my_beep.test()
    time.sleep_ms(100)
    my_beep.test()
else:
    pit2_start()

while True:
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        gc.collect()
        break

    gc.collect()
