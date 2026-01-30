# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容 
from machine import *
from display import *
from seekfree import MOTOR_CONTROLLER, IMU660RX, WIRELESS_UART
from smartcar import ticker, encoder
import ant_else
import ant_motor
import ant_plan
import ant_menu


# 包含 gc 与 time 类
import gc
import time

###################################【变量定义及初始化】###################################
# 多路复用时间计数器
counter = 0      # type: int

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
my_uart6.init(460800)
# 测试uart通信是否正常
# my_uart6.write("hello\r\n")

"""无线串口通信初始化"""
wireless = WIRELESS_UART(115200)


"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = True)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = True)

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("D13", "D14", False)
encoder_ur = encoder("D15", "D16", True)
encoder_md = encoder("C2" , "C3" , False)

# IMU初始化
imu = IMU660RX()

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
key_up = Pin('C14', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_down = Pin('C9', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_left = Pin('C15', Pin.IN, pull = Pin.PULL_UP_47K, value = True)
key_right = Pin('C8', Pin.IN, pull = Pin.PULL_UP_47K, value = True)

"""""""""创建对象"""""""""
# 创建蜂鸣器对象
my_beep = ant_else.beep(beep)

#【文件读取】
# 从config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/config.txt")
my_flash_sys.phase_config()

# 创建数学常量对象
MATH = ant_else.Math()

# 创建指令管理对象
my_order_manager = ant_else.order_manager(my_uart6)

# 创建pid参数对象
pid_data = ant_motor.PID_data(my_flash_sys)

# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_ur = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_md = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_gyroz = ant_motor.SlipAveragingFilter(6)  # 滤波窗口为6个

# 创建小车x和y方向上的速度的卡尔曼滤波器
speed_x_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
speed_y_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
# 视觉伺服自身转角的卡尔曼滤波器
servo_yaw_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
# 创建编码器卡尔曼滤波器对象
encoder_ul_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
encoder_ur_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
encoder_md_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)

# 创建姿态数据对象
pose_data = ant_motor.PoseData(my_flash_sys, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz, encoder_ul_fil, encoder_ur_fil, encoder_md_fil)

# 创建电机pid对象和角度pid对象
motor_ul_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ul)
motor_ur_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ur)
motor_md_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_md)
angle_pid = ant_motor.AnglePositionPID(my_flash_sys)
servo_pid = ant_motor.ServoPID(my_flash_sys)

# 创建小车姿态对象
my_car = ant_motor.CarPose(my_flash_sys, pose_data, MATH, speed_x_fil, speed_y_fil, angle_pid,
                           motor_ul_pid, motor_ur_pid, motor_md_pid,
                           motor_ul, motor_ur, motor_md)

# 创建状态机对象
my_state = ant_plan.StateMachine()

# 创建路径规划数据对象
plan_data = ant_plan.Plan_data(my_flash_sys)

# 创建规划（路径和速度）对象
my_plan = ant_plan.Plan(my_flash_sys, plan_data, MATH, my_car, my_order_manager, wireless, my_beep)

# 创建视觉伺服管理对象2
my_vision_manager_2 = ant_plan.VisionManager_2(my_flash_sys, my_beep, MATH, servo_pid, servo_yaw_fil, wireless)

# 创建串口解析对象
my_protocol = ant_else.UARTProtocol(my_uart6)

# 创建菜单对象
my_menu = ant_menu.Menu(my_flash_sys, my_beep, key_up, key_down, key_left, key_right, lcd)
###################################【函数定义】###################################
# 电机驱动函数
def set_motor(motor, duty) -> None:
    motor.duty(duty)

# 是否成功读取文件和开启定时器检查函数
def detect_if_normal() -> None:
    led.toggle()

# 检测电源电压函数
def voltage_detect(limit_min: float) -> None:
    power_adc_value = power_adc.read_u16()
    power_voltage = power_adc_value / 65535 * 3.3 * 11
    print(f"The current power supply voltage is {power_voltage}!")
    if power_voltage <= limit_min:
        print(f"The power supply voltage: {power_voltage} is too low!")
        my_beep.beep_warn()


# 调试电机速度环pid函数
def show_speed_PID_test():
    motor_ul_pid.compute_pid(550, pose_data.encoder_data_ul)
    motor_ur_pid.compute_pid(550, pose_data.encoder_data_ur)
    motor_md_pid.compute_pid(550, pose_data.encoder_data_md)

# 测试陀螺仪函数
def test_imu():
    wireless.send_str("{:<f},{:<f},{:<f}\n".format(pose_data.gyro_z, pose_data.imu_data[5], pose_data.gyro_z_bias))           
    
# 测试角度闭环函数
def complete_angle_circle():
    my_car.move_ctrl(0, 0, 0)
    
# 全向移动转圈测试函数
target_yaw = 0
def all_around_circle():
    global target_yaw
    target_yaw += 0.1
    if target_yaw >= 180:
        target_yaw = -180
    my_car.move_ctrl(60, target_yaw, 0)


# 多路复用器（用于测试）
count = 0
def test_simble_displacement():
    global count
    count += 1
    if count <= 600:
        my_car.move_ctrl(400, 180, 0)
    else:
        my_car.move_ctrl(0, 90, 90)
        
# 里程计测试函数
test_stage = 0	# 当前模式
def test_odometer():
    global test_stage
    global count
    if count == 0:
        if my_car.x_current <= 300.0 and test_stage == 0:
            my_car.move_ctrl(300, 90, 90)
            return
        elif my_car.y_current >= -300.0 and test_stage == 1:
            my_car.move_ctrl(300, 180, 90)
            return
        elif my_car.x_current >= 0.0 and test_stage == 2:
            my_car.move_ctrl(300, -90, -90)
            return
        elif my_car.y_current <= 0.0 and test_stage == 3:
            my_car.move_ctrl(300, 0, 0)
            return
        elif test_stage == 4:
            my_car.move_ctrl(0, 0, 0)
            return
     
    my_car.move_ctrl(0, 0, 0)
    count += 1
    if count == 50:
        test_stage += 1
        count = 0
    
        
# 全向定位测试函数
def test_global_localization():
    #ant_else.wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_crfrent, my_car.y_crfrent, ant_plan.my_plan.real_target_x, ant_plan.my_plan.real_target_y, ant_plan.my_plan.rest_distance, ant_plan.my_plan.target_yaw, my_car.now_yaw, ant_plan.my_plan.arrive_flag, ant_plan.my_plan.transition_flag))
    my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)

# 测试伺服控制函数
def test_servo_control():
    if my_state.state == my_state.NAVIGATE or my_state.state == my_state.RETURN:
        my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.SERVO:
        my_car.move_ctrl(my_vision_manager_2.target_rel_speed, my_vision_manager_2.target_rel_yaw, my_vision_manager_2.target_rel_turn_angle)
    elif my_state.state == my_state.STOP:
        my_car.move_ctrl(0, 0, 0)

# 视觉伺服测试函数
def test_vision_servo_2():
    global counter
    if my_state.state == my_state.NAVIGATE:
        # counter += 1
        my_plan.navigate([[150.0, 100.0], [330.0, 150.0], [150.0, 200.0], [-30.0, 100.0], [140.0, 90.0]])
        # 等待十秒后向openart发送指令获取目标点坐标
        if my_plan.finish_navigate == True:
            # counter = 0
            my_plan.finish_navigate = False
            my_state.state = my_state.SERVO
            # 切换为目标识别模式
            my_order_manager.mode_target()
            my_beep.test()
    elif my_state.state == my_state.SERVO:
        # 接收openart发送的目标点坐标
        my_vision_manager_2.target_point = my_protocol.coordinate_receive()
        if my_vision_manager_2.target_point:
            my_vision_manager_2.visual_servo_control(my_vision_manager_2.target_point[0], my_vision_manager_2.target_point[1])
            # 测试
            # wireless.send_str(f"x: {my_vision_manager_2.target_point[0]}, y: {my_vision_manager_2.target_point[1]}, target_yaw: {my_vision_manager_2.target_rel_yaw}\r\n")
        if my_vision_manager_2.finish_servo == True:
            counter += 1
            # 过渡400ms防止惯性过冲
            if counter >= 40:
                counter = 0
                my_car.x_current = 150.0
                my_car.y_current = 122.6
                my_order_manager.finish()
                my_state.state = my_state.RETURN
                my_vision_manager_2.finish_servo = False
                # 测试
                my_beep.test()
    elif my_state.state == my_state.RETURN:
            my_plan.navigate([[0.0, 0.0]])
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.STOP

                
# 边线校准测试函数
def test_boundary_calibration():
    global counter
    if my_state.state == my_state.NAVIGATE:
        counter += 1
        # 等待十秒后向openart发送指令获取边界角度
        if counter >= 500:
            counter = 0
            my_plan.if_gain_calibrate_angle = False
            # 切换为上下边界识别模式
            # my_order_manager.mode_boundary_ud()
            # 切换为左右边界识别模式
            my_order_manager.mode_boundary_lf()
            # 测试是否成功发送指令
            my_beep.test()

        if my_plan.if_gain_calibrate_angle == False:
            my_protocol.angle_receive()
            if len(my_protocol.angle_list) >= 10:
                # 进行边线校准处理
                my_plan.calibrate_angle = sum(my_protocol.angle_list) / len(my_protocol.angle_list)
                my_plan.turn_angle_target = my_plan.calibrate_angle 
                my_order_manager.finish()
                my_state.state = my_state.STOP
                my_plan.if_gain_calibrate_angle = True
                # 测试
                my_beep.test()
                for i in range(0, len(my_protocol.angle_list)):
                    wireless.send_str(f"{my_protocol.angle_list[i]}\n")
                wireless.send_str(f"average_angle: {my_plan.turn_angle_target}\n")
                my_protocol.angle_list.clear()


# 移动中的边线校准测试函数
def test_moving_boundary_calibration():
    if my_plan.if_gain_calibrate_angle == False:
        my_protocol.angle_receive()
        if len(my_protocol.angle_list) >= 1:
            # 进行边线校准处理
            # my_plan.calibrate_angle = sum(my_protocol.angle_list) / len(my_protocol.angle_list)
            # my_plan.turn_angle_target += my_plan.calibrate_angle * 2 / 3
            # 进行里程计矫正处理
            if my_car.x_current < 150.0:
                my_car.x_current = 0.0
            else:
                my_car.x_current = 300.0
            my_order_manager.finish()
            my_plan.if_gain_calibrate_angle = True
            
            # 测试
            my_beep.test()
            for i in range(0, len(my_protocol.angle_list)):
                wireless.send_str(f"{my_protocol.angle_list[i]}\n")
            wireless.send_str(f"average_angle: {my_plan.turn_angle_target}\n")
            my_protocol.angle_list.clear()

# 测试openart不同模式切换函数
def test_change_mode():
    global counter
    if my_state.state == my_state.NAVIGATE:
        counter += 1
        # 等待十秒后向openart发送指令获取边界角度
        if counter >= 1000:
            counter = 0
            my_state.state = my_state.MOVE
            my_order_manager.mode_boundary_ud()
            my_beep.test()
    elif my_state.state == my_state.MOVE:
        counter += 1
        if counter >= 50:
            counter = 0
            my_state.state = my_state.SERVO
            my_order_manager.mode_target()
            my_beep.test()
    elif my_state.state == my_state.SERVO:
        counter += 1
        if counter >= 50:
            counter = 0
            my_state.state = my_state.STOP
            my_order_manager.finish()
            my_beep.test()
    elif my_state.state == my_state.STOP:
        counter += 1
        if counter >= 50:
            counter = 0
            my_state.state = my_state.NAVIGATE
            my_beep.test()

""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    # 初始化pid参数
    if motor_ul_pid.target >= 350:
        motor_ul_pid.set_pid_params(pid_data.ul_extreme_kp, pid_data.ul_extreme_ki, pid_data.ul_extreme_kd)
    elif motor_ul_pid.target >= 240:
        motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
    elif motor_ul_pid.target > 100:
        motor_ul_pid.set_pid_params(pid_data.ul_mid_kp, pid_data.ul_mid_ki, pid_data.ul_mid_kd)
    else:
        motor_ul_pid.set_pid_params(pid_data.ul_low_kp, pid_data.ul_low_ki, pid_data.ul_low_kd)
        
    if motor_ur_pid.target >= 350:
        motor_ur_pid.set_pid_params(pid_data.ur_extreme_kp, pid_data.ur_extreme_ki, pid_data.ur_extreme_kd)
    elif motor_ur_pid.target >= 240:
        motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
    elif motor_ur_pid.target > 100:
        motor_ur_pid.set_pid_params(pid_data.ur_mid_kp, pid_data.ur_mid_ki, pid_data.ur_mid_kd)
    else:
        motor_ur_pid.set_pid_params(pid_data.ur_low_kp, pid_data.ur_low_ki, pid_data.ur_low_kd)

    if motor_md_pid.target >= 350:
        motor_md_pid.set_pid_params(pid_data.md_extreme_kp, pid_data.md_extreme_ki, pid_data.md_extreme_kd)
    elif motor_md_pid.target >= 240:
        motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
    elif motor_md_pid.target > 100:
        motor_md_pid.set_pid_params(pid_data.md_mid_kp, pid_data.md_mid_ki, pid_data.md_mid_kd)
    else:
        motor_md_pid.set_pid_params(pid_data.md_low_kp, pid_data.md_low_ki, pid_data.md_low_kd)
    
    # 更新小车姿态
    my_car.update_pose()
    
    # 全向移动转圈测试程序
    #all_around_circle()
    #ant_else.wireless.send_str("{:<f},{:<f}\n".format(my_car.x_crfrent, my_car.car_speed_x))
    
    # 里程计测试程序
    # test_odometer()
    
    # test_simble_displacement()
    
    # 测试角度闭环
    # complete_angle_circle()
    
    # 全向定位测试程序
    # test_global_localization()
    
    #if my_car.x_crfrent <= 8.4:
     #   my_car.move_ctrl(60, 90, 0)
    #else:
     #   my_car.move_ctrl(0, 90, 0)
    # 里程计测试
    #ant_else.wireless.send_str("{:<f}\n".format(my_car.now_yaw))
    
    # 陀螺仪测试
    # test_imu()
    # ant_else.wireless.send_str("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
    # 速度环测试
    # show_speed_PID_test()
    
    # 测试伺服控制函数
    test_servo_control()
    
    # 测试边线矫正程序
    # my_car.move_ctrl(0, 0.0, my_plan.turn_angle_target)

    # 设置电机pwm输出
    my_car.set_motor_pwm()



# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
    # 测试MCU与openart通信

    
    # 全向定位测试程序
    # my_plan.navigate([[150.0, 100.0], [330.0, 150.0], [150.0, 200.0], [-30.0, 100.0], [0, 0]])
    # my_plan.navigate([plan_data.fixed_point[1], plan_data.fixed_point[3], plan_data.fixed_point[2], plan_data.fixed_point[0]])
    
    # 视觉伺服测试程序
    test_vision_servo_2()
    # 边线校准测试程序
    # test_boundary_calibration()
    test_moving_boundary_calibration()
    # 测试openart不同模式切换程序
    # test_change_mode()
    pass


# 闭环控制回调
def time_pit2_handler(time):
    # 用于无线串口调试
    
    # 视觉伺服
    # wireless.send_str("x: {:<f}, y: {:<f}, speed: {:<f}, yaw: {:<f},  {:<f},{:<f}\n".format(servo_pid.actual_x, servo_pid.actual_y, my_vision_manager_2.target_rel_speed, my_vision_manager_2.target_rel_yaw, servo_pid.pwm_output_x, servo_pid.pwm_output_y))
    # wireless.send_str(f"{my_vision_manager_2.target_rel_yaw}\r\n")
    # wireless.send_str("{:<f},{:<f}\n".format(ant_plan.my_vision_manager_2.target_rel_yaw, ant_plan.my_vision_manager_2.target_rel_yaw_fil))
    
    # 速度环输出波形图调参
    # wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ul_pid.pwm_output, motor_ul_pid.derivative * motor_ul_pid.kd))
    # wireless.send_str("{:<f},{:<f},{:<f},{:<f}\n".format(motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd))
    # wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_md_pid.target, motor_md_pid.actual, motor_md_pid.pwm_output, motor_md_pid.derivative * motor_md_pid.kd, motor_md_pid.integral))
    wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual))
        
    # 角度环输出
    # wireless.send_str(f"{angle_pid.pwm_output}\n")
    # imu原始数据
    # wireless.send_str("acc = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[0], pose_data.imu_data[1], pose_data.imu_data[2]))
    # wireless.send_str("gyro = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[3], pose_data.imu_data[4], pose_data.imu_data[5]))
                                                                          
    # 里程计：
    # wireless.send_str("ul: {:<f}, ur: {:<f}, md: {:<f}\n".format(my_car.encouder_ul, my_car.encouder_ur, my_car.encouder_md))
    # wireless.send_str("now: {:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_car.now_yaw * 180 / MATH.PI, angle_pid.pwm_output))
    # wireless.send_str("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_plan.rest_distance, my_plan.v_target, my_car.now_yaw * 180 / MATH.PI, my_plan.arrive_flag))
    
    # 测试边线校准
    # wireless.send_str(f"{my_plan.calibrate_angle}\n")
    
    # 速度规划
    # wireless.send_str(("v_target: %d, rest_dis: %.3f, dec_speed_index: %d\r\n") % (ant_plan.my_plan.v_target, ant_plan.my_plan.rest_distance, ant_plan.my_plan.dec_speed_index))
    
    # 检测自转角是否准确
    # wireless.send_str("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
    # 观察速度
    # wireless.send_str(f"{motor_ul_pid.target},{motor_ul_pid.actual}\n")
    
    # 检测gkd项数量级
    #wireless.send_str(f"{pose_data.gyro_z * my_car.gkd}, {pose_data.gyro_z}\n")
    
    # 卡尔曼滤波（速度）
    # wireless.send_str("{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.car_speed_x, ant_motor.speed_x_fil.update(ant_motor.my_car.car_speed_x), ant_motor.speed_x_fil2.filtering(ant_motor.my_car.car_speed_x)))
    # wireless.send_str("{:<f}\n".format(pose_data.encoder_data_ul))

    key = my_menu.read_key()
    if key == None:
        return
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
            else:
                my_menu.Menu_Page2_data_show()
            my_menu.show_arrow()


# 定时器1初始化（中断回调函数在 ant_motor 中）
def pit1_start():
    global imu_data
    pit1 = ticker(1)
    pit1.capture_list(encoder_ul, encoder_ur, encoder_md, imu)
    # 进行IMU零漂校准并将imu_data与定时器1的底层采集绑定
    pose_data.init_bias()
    pit1.callback(time_pit1_handler)
    pit1.start(my_flash_sys.find_value("motor_control_T"))

# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    pit2 = ticker(2)
    pit2.callback(time_pit2_handler)
    pit2.start(my_flash_sys.find_value("uart_and_menu_T"))

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    pit3 = ticker(3)
    pit3.callback(time_pit3_handler)
    pit3.start(my_flash_sys.find_value("plan_calculate_T"))

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.6)

# 屏幕测试程序
# ant_menu.Menu_First()

# 打开定时器
pit1_start()
pit3_start()
pit2_start()

# === 初始显示 ===
my_menu.Menu_Page_1()
my_menu.show_arrow()

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