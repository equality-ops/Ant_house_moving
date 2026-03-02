# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态决定是否跳转执行 user_main.py
# 当成功执行 user_main.py 后 C4 LED 会以一秒周期进行闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容 
from machine import *
from display import *
from seekfree import MOTOR_CONTROLLER, IMU660RX, DL1X, KEY_HANDLER
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
# 按键消抖相关变量
current_time = 0
last_left_time = 0
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

"""蜂鸣器初始化"""
beep = Pin('D24', Pin.OUT, value = False)

"""异步串口通信初始化"""
my_uart6 = UART(5)
my_uart6.init(460800)

"""无线串口通信初始化"""
my_uart3 = UART(2)
my_uart3.init(115200)

# 测试uart通信是否正常
# my_uart6.write("hello\r\n")
# my_uart3.write("hello\r\n")

"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = True)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = True)

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("C2" , "C3" , False)
encoder_ur = encoder("D13", "D14", False)
encoder_md = encoder("D15", "D16", True)

# IMU初始化
imu = IMU660RX()

# tof深度传感器初始化
tof = DL1X()

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
# enc_rotation = encoder("D15", "D16", True)

"""""""""创建对象"""""""""
# 创建状态机对象
my_state = ant_plan.StateMachine()

# 创建蜂鸣器对象
my_beep = ant_else.beep(beep)

#【文件读取】
# 从main_config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/slave_config.txt")
my_flash_sys.phase_config()

# 创建数学常量对象
MATH = ant_else.Math()

# 创建指令管理对象
my_order_manager = ant_else.order_manager(my_uart6)

# 创建openart串口解析对象
my_art_protocol = ant_else.UARTProtocol(my_uart6)

# 创建主从车无线串口通信对象
my_slave_protocol = ant_else.LinkProtocol(my_uart3)

# 创建pid参数对象
pid_data = ant_motor.PID_data(my_flash_sys)

# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_ur = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_md = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_gyroz = ant_motor.SlipAveragingFilter(6)  # 滤波窗口为5个

# 创建小车x和y方向上的速度的卡尔曼滤波器
speed_x_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
speed_y_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
# 创建编码器卡尔曼滤波器对象
encoder_ul_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.05, R = 2.0)
encoder_ur_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.05, R = 2.0)
encoder_md_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.05, R = 2.0)
# 创建tof测距滤波器对象
tof_distance_fil = ant_motor.ToFFilter(window_size=5, alpha=0.4)
# 创建小车自转角滤波器对象
car_yaw_fil = ant_motor.SlipAveragingFilter(8)
# 创建主车正余弦滑动平均滤波器对象
sin_diff_fil = ant_motor.SlipAveragingFilter(50)
cos_diff_fil = ant_motor.SlipAveragingFilter(50)
# 创建视觉伺服正余弦滤波对象
sin_servo_fil = ant_motor.SlipAveragingFilter(5)    
cos_servo_fil = ant_motor.SlipAveragingFilter(5)

# 创建姿态数据对象
pose_data = ant_motor.PoseData(my_flash_sys, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz, encoder_ul_fil, encoder_ur_fil, encoder_md_fil)

# 创建电机pid对象和角度pid对象
motor_ul_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ul)
motor_ur_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ur)
motor_md_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_md)
angle_pid = ant_motor.AnglePositionPID(my_flash_sys)
servo_pid = ant_motor.ServoPID(my_flash_sys)

# 创建小车姿态对象
my_car = ant_motor.CarPose(my_flash_sys, my_state, pose_data, MATH, speed_x_fil, speed_y_fil, car_yaw_fil, angle_pid,
                           motor_ul_pid, motor_ur_pid, motor_md_pid,
                           motor_ul, motor_ur, motor_md)

# 创建路径规划数据对象
plan_data = ant_plan.Plan_data(my_flash_sys)

# 创建规划（路径和速度）对象
my_plan = ant_plan.Plan(my_flash_sys, plan_data, MATH, my_car, my_state, my_order_manager, my_uart3, my_beep, my_art_protocol, sin_diff_fil, cos_diff_fil)

# 创建视觉伺服管理对象2
my_vision_manager = ant_plan.VisionManager(my_flash_sys, my_beep, MATH, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, tof, tof_distance_fil, my_car, my_art_protocol, my_order_manager, my_plan)

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

# tof传感器预热初始化函数
def tof_init():
    for i in range(0, 30):
        tof.get()
        time.sleep_ms(5)

# 角度环计算函数
def angle_pid_compute():
    filter_yaw = my_car.car_yaw_filter.car_yaw_filtering(my_car.now_yaw * 180 / MATH.PI)
    # 计算z轴的目标速度
    angle_pid.compute_pid(my_car.turn_angle_target, filter_yaw)

# 用于从车启动的函数
def slave_start():
    global current_time, last_left_time, start_flag, if_press_start_key
    if start_flag == False:
        if if_press_start_key == False:
            current_time = time.ticks_ms()
            if key_data[3] != 0 and switch2.value() == 0:
                # 清除按键状态
                key.clear(4)
                my_beep.key_test()
                if_press_start_key = True
        else:   
            # 测试，此时只调试从车，双车正常通信时需要解注释  
            # if my_slave_protocol.get_start_signal() == True:
            my_state.state_work = 0
            # 初始状态设置为导航状态
            my_state.state = my_state.READY_NAVIGATE
            start_flag = True
            # 延时一秒避免零漂校准不准确
            time.sleep_ms(1000)
            # 打开定时器1和3
            pit1_start()
            pit3_start()
            # 检测是否正常初始化所有
            detect_if_normal()

# 用于准备视觉伺服和环绕
def ready_servo_and_orbit():
    # 根据物品种类选择伺服距离和环绕半径
    if my_vision_manager.current_servo_object == ord('T'):
        servo_pid.target_y = servo_pid.target_y_T
        my_vision_manager.object_radius = my_vision_manager.radius_T
    elif my_vision_manager.current_servo_object == ord('S'):
        servo_pid.target_y = servo_pid.target_y_S
        my_vision_manager.object_radius = my_vision_manager.radius_S
    elif my_vision_manager.current_servo_object == ord('B'):
        servo_pid.target_y = servo_pid.target_y_B
        my_vision_manager.object_radius = my_vision_manager.radius_B

# 调试电机速度环pid函数
def show_speed_PID_test():
    motor_ul_pid.compute_pid(300, pose_data.encoder_data_ul)
    # motor_ur_pid.compute_pid(300, pose_data.encoder_data_ur)
    # motor_md_pid.compute_pid(400, pose_data.encoder_data_md)

# 测试陀螺仪函数
def test_imu():
    my_uart3.write("{:<f},{:<f},{:<f}\n".format(pose_data.gyro_z, pose_data.imu_data[4], pose_data.gyro_z_bias))           
    
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
        if my_car.x_current <= 60.0 and test_stage == 0:
            my_car.move_ctrl(600, 90, 0)
            return
        elif my_car.y_current >= -60.0 and test_stage == 1:
            my_car.move_ctrl(600, 180, 0)
            return
        elif my_car.x_current >= 0.0 and test_stage == 2:
            my_car.move_ctrl(600, -90, 0)
            return
        elif my_car.y_current <= 0.0 and test_stage == 3:
            my_car.move_ctrl(600, 0, 0)
            return
        elif test_stage == 4:
            my_car.move_ctrl(0, 0, 0)
            return
     
    my_car.move_ctrl(0, 0, 0)
    count += 1
    if count == 100:
        test_stage += 1
        count = 0
    
        
# 全向定位测试函数
def test_global_localization():
    #ant_else.my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_crfrent, my_car.y_crfrent, ant_plan.my_plan.real_target_x, ant_plan.my_plan.real_target_y, ant_plan.my_plan.rest_distance, ant_plan.my_plan.target_yaw, my_car.now_yaw, ant_plan.my_plan.arrive_flag, ant_plan.my_plan.transition_flag))
    my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)

# 测试伺服控制函数
def test_servo_control():
    if my_state.state == my_state.NAVIGATE or my_state.state == my_state.RETURN or my_state.state == my_state.STOP or my_state.state == my_state.CALIBRATE or my_state.state == my_state.SCAN or my_state.state == my_state.MOVE :
        my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.SERVO:
        my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
    elif my_state.state == my_state.ORBIT:
        my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
    elif my_state.state == my_state.READY_NAVIGATE:
        my_car.move_ctrl(0, 0.0, my_car.now_yaw * 180.0 / MATH.PI)

# 视觉伺服测试函数
def test_vision_servo():
    global counter
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.NAVIGATE
        my_order_manager.mode_target()
    elif my_state.state == my_state.NAVIGATE:
        my_plan.finish_navigate = False
        target_point = my_art_protocol.coordinate_receive()
        if target_point:
            my_vision_manager.current_servo_object = target_point[2]
            ready_servo_and_orbit()
            my_state.state = my_state.SERVO
            # 测试
            my_beep.test()
    elif my_state.state == my_state.SERVO:
        my_vision_manager.visual_servo_control()
        if my_vision_manager.finish_servo == True:
            counter += 1
            # 过渡400ms防止惯性过冲
            if counter >= 40:
                counter = 0
                my_state.state = my_state.ORBIT
                # 重置标志位
                my_vision_manager.if_send_servo_command = False
                my_vision_manager.finish_servo = False
                # 测试
                my_beep.test()
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(120.0)
        if my_vision_manager.finish_orbit:
            my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
            my_state.state = my_state.STOP
            my_plan.turn_angle_target = my_car.now_yaw * 180.0 / MATH.PI
    elif my_state.state == my_state.STOP:
        my_plan.stop()

                
# 边线校准测试函数
def test_boundary_calibration():
    if my_state.state == my_state.NAVIGATE:
        my_plan.navigate([[160.0, -20.0], [160.0, 240.0]], 180.0)
        if my_plan.finish_navigate == True:
            my_plan.finish_navigate = False
            # 测试
            # my_state.state = my_state.CALIBRATE
            my_state.state = my_state.RETURN
            my_beep.test()
    elif my_state.state == my_state.CALIBRATE:
        my_plan.boundary_calibrate_control()
        if my_plan.if_finish_calibrate == True:
            my_plan.if_finish_calibrate, my_plan.if_gain_calibrate_angle, my_plan.if_ready_calibrate = False, False, False
            my_state.state = my_state.RETURN
            # 测试
            my_beep.test()
    elif my_state.state == my_state.RETURN:
        my_plan.navigate([[0.0, 0.0]], 0.0)
        if my_plan.finish_navigate == True:
            my_plan.finish_navigate = False
            my_state.state = my_state.STOP
            my_beep.test()
    elif my_state.state == my_state.STOP:
        my_plan.stop()

# 测试环绕控制函数
def test_orbit_control():
    if my_state.state == my_state.NAVIGATE:
        my_state.state = my_state.ORBIT
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(120.0)
        if my_vision_manager.finish_orbit == True:
            my_vision_manager.finish_orbit = False
            my_state.state = my_state.STOP
            # 测试
            my_beep.test()
    elif my_state.state == my_state.STOP:
        pass

# 测试从车解析主车状态及坐标通信函数 
def test_main_slave_communication():
    message = my_slave_protocol.get_latest_valid_data('M')
    if message:
        my_uart3.write(f"Received message: {message}\n")
        # 测试
        my_beep.test()

# 主从车协同导航测试函数
def test_main_slave_collaborative_navigation():
    final_path = my_slave_protocol.get_path_list()
    if final_path:
        my_slave_protocol.send_slave_state("get")
        # 测试
        my_beep.test()
        my_uart3.write(f"Received path: {final_path}\n")

"""
# 任务执行机
def task_machine():
    if my_state.state_work == 0:
        if my_state.state == my_state.READY_NAVIGATE:
            if my_plan.if_send_path == False:
                my_main_protocol.send_path([[plan_data.fixed_point[1][0], plan_data.fixed_point[1][1] - 30.0]])
                my_plan.if_send_path = True
            if my_main_protocol.get_slave_state() == "get":
                my_plan.if_send_path = False
                my_state.state = my_state.NAVIGATE
                # 测试
                my_beep.test()
        if my_state.state == my_state.NAVIGATE:
            my_plan.main_tactical_navigate([plan_data.fixed_point[1]], target_turn_angle=0.0)
            # 向从车发送主车姿态
            my_main_protocol.send_pose('M', my_car.x_current, my_car.y_current, my_plan.target_yaw, my_plan.turn_angle_target, my_state.state)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.SCAN
                # 测试
                my_beep.test()
        if my_state.state == my_state.SCAN:
            plan_data.object = [[160.0, 100.0, 1]]  # [x, y, type]
            my_state.state = my_state.NAVIGATE
            my_state.state_work = 1
    elif my_state.state_work == 1:
        if my_state.state == my_state.NAVIGATE:
            my_plan.main_tactical_navigate([plan_data.object[0][0], 55.0], target_turn_angle=0.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.SERVO
                # 测试
                my_beep.test()
        elif my_state.state == my_state.SERVO:
            my_vision_manager.visual_servo_control()
            if my_vision_manager.finish_servo == True:
                my_vision_manager.finish_servo = False
                # 记录实际物体坐标和种类
                plan_data.object_real = [my_car.x_current, my_car.y_current, plan_data.object[0][2]]
                my_state.state = my_state.ORBIT
                # 测试
                my_beep.test()
        elif my_state.state == my_state.ORBIT:
            if my_vision_manager.finish_orbit == False:
                my_vision_manager.orbit_control(120.0)
            else:
                # 完成orbit后停止等待从车也完成环绕
                my_vision_manager.orbit_speed = 0
                if my_main_protocol.get_slave_state() == "finish":
                    my_main_protocol.send_pose('M', plan_data.object_real[0], plan_data.object_real[1], my_plan.target_yaw, my_car.now_yaw * 180 / MATH.PI, my_state.state)
                if my_main_protocol.get_slave_state() == "get":    
                    my_state.state = my_state.STOP
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.STOP:
            my_plan.stop()
            if my_main_protocol.get_slave_state() == "ready":
                my_state.state = my_state.MOVE
        elif my_state.state == my_state.MOVE:
            # 让主车保持目前转角进行定向移动
            my_plan.main_tactical_navigate([my_car.x_current, -20.0], my_car.now_yaw * 180 / MATH.PI)
            # 向从车发送主车姿态
            my_main_protocol.send_pose('M', my_car.x_current, my_car.y_current, my_plan.target_yaw, my_plan.turn_angle_target, my_state.state)
            '''保留从车丢失接口'''
            if my_plan.finish_navigate == True and my_main_protocol.get_slave_state() == "finish":
                my_plan.finish_navigate = False
                my_state.state = my_state.CALIBRATE
                my_state.state_work = 0
                # 测试
                my_beep.test()
        elif my_state.state == my_state.CALIBRATE:
            my_plan.boundary_calibrate_control()
            if my_plan.if_finish_calibrate == True:
                my_plan.if_finish_calibrate, my_plan.if_gain_calibrate_angle = False, False
                my_state.state = my_state.READY_NAVIGATE
                my_state.state_work = 2
                # 测试
                my_beep.test()
    elif my_state.state_work == 2:
            if my_state.state == my_state.READY_NAVIGATE:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path([[plan_data.fixed_point[0][0] - 20.0, plan_data.fixed_point[0][1]]])
                    my_plan.if_send_path = True
                if my_main_protocol.get_slave_state() == "get":
                    my_plan.if_send_path = False
                    my_state.state = my_state.RETURN
                    # 测试
                    my_beep.test()
            elif my_state.state == my_state.RETURN:
                my_plan.main_tactical_navigate([plan_data.fixed_point[0]], 0.0)
                my_main_protocol.send_pose('M', my_car.x_current, my_car.y_current, my_plan.target_yaw, my_car.now_yaw * 180 / MATH.PI, my_state.state)
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_state.state = my_state.STOP
                    # 测试
                    my_beep.test()
            elif my_state.state == my_state.STOP:
                my_plan.stop()
"""

""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    # 初始化pid参数
    if motor_ul_pid.target >= 400:
        motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
    elif motor_ul_pid.target >= 200:
        motor_ul_pid.set_pid_params(pid_data.ul_mid_kp, pid_data.ul_mid_ki, pid_data.ul_mid_kd)
    else:
        motor_ul_pid.set_pid_params(pid_data.ul_low_kp, pid_data.ul_low_ki, pid_data.ul_low_kd)
        
    if motor_ur_pid.target >= 400:
        motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
    elif motor_ur_pid.target >= 200:
        motor_ur_pid.set_pid_params(pid_data.ur_mid_kp, pid_data.ur_mid_ki, pid_data.ur_mid_kd)
    else:
        motor_ur_pid.set_pid_params(pid_data.ur_low_kp, pid_data.ur_low_ki, pid_data.ur_low_kd)

    if motor_md_pid.target >= 400:
        motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
    elif motor_md_pid.target >= 200:
        motor_md_pid.set_pid_params(pid_data.md_mid_kp, pid_data.md_mid_ki, pid_data.md_mid_kd)
    else:
        motor_md_pid.set_pid_params(pid_data.md_low_kp, pid_data.md_low_ki, pid_data.md_low_kd)
    
    # 更新小车姿态
    my_car.update_pose()
    
    # 全向移动转圈测试程序
    #all_around_circle()
    #ant_else.my_uart3.write("{:<f},{:<f}\n".format(my_car.x_crfrent, my_car.car_speed_x))
    
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
    #ant_else.my_uart3.write("{:<f}\n".format(my_car.now_yaw))
    
    # 陀螺仪测试
    # test_imu()
    # ant_else.my_uart3.write("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
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
    # 角度环计算（10ms）
    angle_pid_compute()

    # 任务执行机
    # task_machine()

    # 测试主从车通信
    # test_main_slave_communication()
    # test_main_slave_collaborative_navigation()

    # 全向定位测试程序
    """
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.NAVIGATE
    elif my_state.state == my_state.NAVIGATE:
        my_plan.navigate([[140.0, 190.0], [140.0, 50.0], [0.0, 0.0]], 0.0)
        if my_plan.finish_navigate == True:
            my_plan.finish_navigate = False
            my_state.state = my_state.STOP
            my_beep.test()
    elif my_state.state == my_state.STOP:
        my_plan.stop()
    """
    # my_plan.navigate([plan_data.fixed_point[1], plan_data.fixed_point[3], plan_data.fixed_point[2], plan_data.fixed_point[0]])
    # my_plan.main_tactical_navigate([[320.0, 0.0]], target_turn_angle=0.0)
    # 战术避障
    # my_plan.main_tactical_navigate([[320.0, 240.0], [0, 0]], [[110.0, 70.0, 60.0], [210.0, 70.0, 60.0],  [210.0, 170.0, 60.0],  [110.0, 170.0, 60.0]], target_turn_angle=0.0)
    
    # 视觉伺服测试程序
    test_vision_servo()

    # 边线校准测试程序
    # test_boundary_calibration()
    # test_moving_boundary_calibration()

    # 测试openart不同模式切换程序
    # test_change_mode()

    # 环绕物体测试程序
    # test_orbit_control()
    pass


# 定时器2中断回调函数
# 用于无线串口调试和发车启动
def time_pit2_handler(time):
    """用于无线串口调试"""
    # 发车启动函数
    slave_start()
    
    # 读取按键（中断中避免阻塞，快速返回）
    key = my_menu.read_key()
    my_menu.handle_key_from_interrupt(key)

    # 视觉伺服
    my_uart3.write(f"servo_pid.target_y: {servo_pid.target_y}, object_radius: {my_vision_manager.object_radius}\n")
    # my_uart3.write("x: {:<f}, y: {:<f}, speed: {:<f}, yaw: {:<f},  {:<f},{:<f}\n".format(servo_pid.actual_x, servo_pid.actual_y, my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, servo_pid.pwm_output_x, servo_pid.pwm_output_y))
    # my_uart3.write(f"{my_vision_manager.target_rel_speed_x},{my_vision_manager.target_rel_speed_y},{my_vision_manager.target_rel_yaw}\r\n")
    # my_uart3.write("{:<f},{:<f}\n".format(ant_plan.my_vision_manager.target_rel_yaw, ant_plan.my_vision_manager.target_rel_yaw_fil))
    
    # 速度环输出波形图调参
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ul_pid.pwm_output, motor_ul_pid.derivative * motor_ul_pid.kd, motor_ul_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f}\n".format(motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd, motor_ur_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_md_pid.target, motor_md_pid.actual, motor_md_pid.pwm_output, motor_md_pid.derivative * motor_md_pid.kd, motor_md_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual))
        
    # 角度环输出
    # my_uart3.write(f"{angle_pid.pwm_output},{angle_pid.target},{angle_pid.actual}\n")
    # my_uart3.write(f"{pose_data.gyro_z}, {pose_data.gyro_z_bias}\n")
    # imu原始数据
    # my_uart3.write("acc = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[0], pose_data.imu_data[1], pose_data.imu_data[2]))
    # my_uart3.write("gyro = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[3], pose_data.imu_data[4], pose_data.imu_data[5]))
                                                                          
    # 里程计：
    # my_uart3.write(f"{my_plan.target_yaw}\n")
    # my_uart3.write("ul: {:<f}, ur: {:<f}, md: {:<f}\n".format(my_car.encouder_ul, my_car.encouder_ur, my_car.encouder_md))
    # my_uart3.write("now: {:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_car.now_yaw * 180 / MATH.PI, angle_pid.pwm_output))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_plan.rest_distance, my_plan.v_target, my_car.now_yaw * 180 / MATH.PI, my_plan.arrive_flag))
    
    # my_uart3.write(f"{my_car.angle_pid.target}, {my_car.angle_pid.actual}, {my_car.angle_pid.nowError}, {my_state.state}\n")

    # tof传感器测试
    # my_uart3.write(f"{tof_distance_fil.update(tof.get())},{tof.get()}\r\n")

    # 测试边线校准
    # my_uart3.write(f"{my_plan.calibrate_angle}\n")
    
    # 速度规划
    # my_uart3.write(("v_target: %d, rest_dis: %.3f, dec_speed_index: %d\r\n") % (ant_plan.my_plan.v_target, ant_plan.my_plan.rest_distance, ant_plan.my_plan.dec_speed_index))
    
    # 检测自转角是否准确
    # my_uart3.write("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
    # 观察速度
    # my_uart3.write(f"{motor_ul_pid.target},{motor_ul_pid.actual}\n")
    
    # 检测gkd项数量级
    # my_uart3.write(f"{pose_data.gyro_z * my_car.gkd}, {pose_data.gyro_z}\n")
    
    # 卡尔曼滤波（速度）
    # my_uart3.write("{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.car_speed_x, ant_motor.speed_x_fil.update(ant_motor.my_car.car_speed_x), ant_motor.speed_x_fil2.filtering(ant_motor.my_car.car_speed_x)))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(pose_data.encoder_data_ul, pose_data.encoder_data_ul_2,pose_data.encoder_data_ur, pose_data.encoder_data_ur_2,pose_data.encoder_data_md, pose_data.encoder_data_md_2))
    # my_uart3.write("{:<f},{:<f},{:<f}\n".format(pose_data.encoder_data_ul,pose_data.encoder_data_ur,pose_data.encoder_data_md))

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
    pit2.capture_list(key)
    pit2.callback(time_pit2_handler)
    pit2.start(my_flash_sys.find_value("uart_and_menu_T"))

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    pit3 = ticker(3)
    pit3.capture_list(tof)
    tof_init()
    pit3.callback(time_pit3_handler)
    pit3.start(my_flash_sys.find_value("plan_calculate_T"))

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.4)

# 打开定时器
pit2_start()

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