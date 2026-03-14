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
import ant_vision
import ant_menu


# 包含 gc 与 time 类
import gc
import time

###################################【变量定义及初始化】###################################
# 多路复用时间计数器
ticker1_counter = 0  # type: int
counter = 0      # type: int
# 按键消抖相关变量
current_time = 0
last_left_time = 0
# 是否按下启动按键标志位
if_press_start_key = False
# 是否成功启动标志位
start_flag = False
DOWN = 1         # 位于矩形下边沿
UP = 2           # 位于矩形上边沿
CHECK = 3        # 检验阶段（检查是否搬运完所有物体）
RETURN_WORK = 4  # 返回阶段（搬运完所有物体后返回起点）

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

"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = False)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = False)

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("D16", "D15", True)
encoder_ur = encoder("D13", "D14", True)
encoder_md = encoder("C2" , "C3" ,True)

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
lcd.mode(2)
lcd.clear(0x0000)

# 与定时器2周期一致，都为20ms
key = KEY_HANDLER(20)
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
my_flash_sys = ant_else.flash_system(my_beep, "/flash/main_config.txt")
my_flash_sys.phase_config()

# 创建数学常量对象
MATH = ant_else.Math()

# 创建指令管理对象
my_order_manager = ant_else.order_manager(my_uart6)

# 创建openart串口解析对象
my_art_protocol = ant_else.UARTProtocol(my_uart6)

# 创建主从车无线串口通信对象
my_main_protocol = ant_else.LinkProtocol(my_uart3)

# 创建pid参数对象
pid_data = ant_motor.PID_data(my_flash_sys)

# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_ur = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为3个
diff_filter_md = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_gyroz = ant_motor.SlipAveragingFilter(10)  # 滤波窗口为5个

# 创建小车x和y方向上的速度的卡尔曼滤波器
speed_x_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
speed_y_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 4.0)
# 创建编码器卡尔曼滤波器对象
encoder_ul_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 1.0)
encoder_ur_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 1.0)
encoder_md_fil = ant_motor.KalmanFilter(P = 1.0, Q = 0.01, R = 1.0)
# 创建tof测距滤波器对象
tof_distance_fil = ant_motor.ToFFilter(window_size=5, alpha=0.4)
# 创建小车自转角滤波器对象
car_yaw_fil = ant_motor.SlipAveragingFilter(4)
# 创建主车正余弦滑动平均滤波器对象
sin_diff_fil = ant_motor.SlipAveragingFilter(50)
cos_diff_fil = ant_motor.SlipAveragingFilter(50)
# 创建视觉伺服正余弦滤波对象
sin_servo_fil = ant_motor.SlipAveragingFilter(6)    
cos_servo_fil = ant_motor.SlipAveragingFilter(6)

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
my_vision_manager = ant_vision.VisionManager(my_flash_sys, my_beep, MATH, angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, tof, tof_distance_fil, my_car, my_art_protocol, my_order_manager, my_plan, my_state)

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
                # my_state.state = my_state.READY_NAVIGATE
                my_state.state = my_state.NAVIGATE
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
    # 控制小车面向物体进行视觉伺服控制
    my_vision_manager.target_rel_turn_angle = my_plan.turn_angle_target
    # 根据物品种类选择伺服距离和环绕半径
    if my_vision_manager.current_servo_object == ord('T'):
        servo_pid.target_y = servo_pid.target_y_T
        my_vision_manager.object_radius = my_vision_manager.radius_T
        my_vision_manager.orbit_angle = my_vision_manager.angle_T
    elif my_vision_manager.current_servo_object == ord('S'):
        servo_pid.target_y = servo_pid.target_y_S
        my_vision_manager.object_radius = my_vision_manager.radius_S
        my_vision_manager.orbit_angle = my_vision_manager.angle_S
    elif my_vision_manager.current_servo_object == ord('B'):
        servo_pid.target_y = servo_pid.target_y_B
        my_vision_manager.object_radius = my_vision_manager.radius_B
        my_vision_manager.orbit_angle = my_vision_manager.angle_B

# 重置导航及速度规划相关标志位
def reset_navigate_flags():
    # 导航
    my_plan.finish_navigate = False
    my_plan.arrive_flag = False
    my_plan.dec_speed_index = 0
    plan_data.aimed_point_index = 0
    my_plan.path_points.clear()
    my_plan.if_set_path = False
    my_plan.if_finish_turn = False
    my_plan.transition_flag = False
    # 速度规划
    my_plan.stage = my_plan.STOP
    my_plan.finish_building = False

# 测试陀螺仪函数
def test_imu():
    my_uart3.write("{:<f},{:<f},{:<f}\n".format(pose_data.gyro_y, pose_data.imu_data[4], pose_data.gyro_y_bias))           
    
# 测试角度闭环函数
def complete_angle_circle():
    my_car.move_ctrl(0, 0, 0)

# 小车姿态总控制函数
def master_control():
    if my_state.state == my_state.NAVIGATE or my_state.state == my_state.RETURN or my_state.state == my_state.STOP or my_state.state == my_state.SCAN or my_state.state == my_state.MOVE :
        my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.SERVO:
        # 未丢失物体时正常进行视觉伺服控制，丢失物体时进行矩形轨迹的导航控制
        if my_vision_manager.if_lost_object == False:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        else:
            my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.CALIBRATE:
        if my_vision_manager.if_ready_calibrate == False:
            my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)
        else:
            # 未丢失物体时正常进行视觉伺服控制，丢失物体时进行轨迹的导航控制
            if my_vision_manager.if_lost_object == False:
                my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
            else:
                my_car.move_ctrl(my_plan.v_target, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == my_state.ORBIT or my_state.state == my_state.REVERSE_ORBIT:
        my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
    elif my_state.state == my_state.READY_NAVIGATE:
        my_car.move_ctrl(0, 0.0, my_car.now_yaw * 180.0 / MATH.PI)

# 视觉伺服测试函数
def test_vision_servo():
    global counter
    if my_state.state == my_state.NAVIGATE:
        my_order_manager.mode_target()
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
            # 过渡1s防止惯性过冲
            if counter >= 50:
                counter = 0
                my_state.state = my_state.ORBIT
                my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                # 重置标志位
                my_vision_manager.if_send_servo_command = False
                my_vision_manager.finish_servo = False
                # 测试
                my_beep.test()
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(180.0)
        if my_vision_manager.finish_orbit == True:
            my_vision_manager.finish_orbit = False
            my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
            my_state.state = my_state.STOP
            # 测试
            my_beep.test()
    elif my_state.state == my_state.STOP:
        my_plan.stop()

# 视觉伺服辅助apriltag码矫正
def test_apriltag_calibrate():
    if my_state.state == my_state.NAVIGATE:
        my_state.state = my_state.CALIBRATE
    elif my_state.state == my_state.CALIBRATE:
        my_vision_manager.apriltag_calibrate_control()
        if my_vision_manager.if_finish_calibrate == True:
            my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
            my_state.state = my_state.STOP
    elif my_state.state == my_state.STOP:
        my_plan.stop()

# 双车版的任务执行机
def collaborative_task_machine():
    global counter
    if my_state.state_work == DOWN:
        if my_state.state == my_state.NAVIGATE:
                my_plan.navigate([[plan_data.fixed_point[1][0], plan_data.fixed_point[1][1]]], 0.0)
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    if my_vision_manager.failed_servo_count >= 2:
                        my_vision_manager.failed_servo_count = 0
                        my_state.state = my_state.NAVIGATE
                        my_state.state_work = UP
                    else:
                        my_state.state = my_state.SCAN
                        my_vision_manager.my_order_manager.mode_target()
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.SCAN:
                my_plan.navigate([[plan_data.fixed_point[3][0], plan_data.fixed_point[3][1]]], 0.0)
                if my_plan.finish_navigate == False:
                    target_point = my_art_protocol.coordinate_receive()
                    if target_point and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B')):
                        my_vision_manager.current_servo_object = target_point[2]
                        # 初始化视觉伺服偏航角缓冲区，使其过渡更平滑
                        sin_servo_fil.buffer_init(my_plan.scan_v_max)
                        cos_servo_fil.buffer_init(0)
                        ready_servo_and_orbit()
                        reset_navigate_flags()
                        my_state.state = my_state.SERVO
                else:
                    my_plan.finish_navigate = False
                    # 此时矩形下区域已没有物体，控制小车移动到上区域寻找物体
                    my_state.state_work = UP
                    my_state.state = my_state.NAVIGATE
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                # 若丢失物体则按矩形轨迹行驶寻找物体
                my_plan.navigate([[my_car.x_current+25.0, my_car.y_current], [my_car.x_current+25.0, my_car.y_current-30.0], [my_car.x_current-25.0, my_car.y_current-30.0], [my_car.x_current-25.0, my_car.y_current], [my_car.x_current, my_car.y_current]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point:
                    my_vision_manager.current_servo_object = target_point[2]
                    ready_servo_and_orbit()
                    reset_navigate_flags()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_beep.test()
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 重回扫描点继续寻找物体
                    my_plan.return_to_scan_point = True
                    my_state.state = my_state.NAVIGATE

            if my_vision_manager.finish_servo == True:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(my_vision_manager.current_servo_object, [[my_car.x_current, plan_data.fixed_point[1][1]], [my_car.x_current, my_car.y_current - 10.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    # 重置标志位
                    my_vision_manager.finish_servo = False
                    my_plan.if_send_path = False

                    my_state.state = my_state.ORBIT
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.ORBIT:
            my_vision_manager.orbit_control(my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                order = my_main_protocol.get_slave_state()
                if order == "finish":
                    # 重置从车视觉伺服失败次数
                    my_vision_manager.failed_servo_count = 0
                    my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                    my_state.state = my_state.MOVE
                    # 测试
                    my_beep.test()
                elif order == "lost":
                    my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                    my_vision_manager.failed_servo_count += 1
                    my_state.state = my_state.REVERSE_ORBIT
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.MOVE:
                # 控制小车夹紧物体
                my_plan.navigate([[my_car.x_current + 5.0, -25.0]])
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.car_position = 0
                    my_state.state = my_state.CALIBRATE
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.CALIBRATE:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.apriltag_calibrate_control()
            else:
                # 控制小车前后移动寻找apriltag码
                if my_vision_manager.car_position == 0:
                    my_plan.navigate([[my_car.x_current+15.0, my_car.y_current], [my_car.x_current+15.0, my_car.y_current-20.0], [my_car.x_current-15.0, my_car.y_current-20.0], [my_car.x_current-15.0, my_car.y_current+20.0], [my_car.x_current+15.0, my_car.y_current+20.0]], 90)
                elif my_vision_manager.car_position == 2:
                    my_plan.navigate([[my_car.x_current+15.0, my_car.y_current], [my_car.x_current+15.0, my_car.y_current+20.0], [my_car.x_current-15.0, my_car.y_current+20.0], [my_car.x_current-15.0, my_car.y_current-20.0], [my_car.x_current+15.0, my_car.y_current-20.0]], 90)
                elif my_vision_manager.car_position == 1:
                    my_plan.navigate([[my_car.x_current-15.0, my_car.y_current], [my_car.x_current-15.0, my_car.y_current-20.0], [my_car.x_current+15.0, my_car.y_current-20.0], [my_car.x_current+15.0, my_car.y_current+20.0], [my_car.x_current-15.0, my_car.y_current+20.0]], -90)
                elif my_vision_manager.car_position == 3:
                    my_plan.navigate([[my_car.x_current-15.0, my_car.y_current], [my_car.x_current-15.0, my_car.y_current+20.0], [my_car.x_current+15.0, my_car.y_current+20.0], [my_car.x_current+15.0, my_car.y_current-20.0], [my_car.x_current-15.0, my_car.y_current-20.0]], -90)

                target_point = my_art_protocol.apriltag_receive()
                if target_point:
                    reset_navigate_flags()
                    my_vision_manager.counter = 0
                    my_vision_manager.calibrate_times = 0
                    my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车回到扫描点进行扫描
                if my_plan.finish_navigate == True:
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    # 主车给从车发消息让从车完成矫正
                    my_main_protocol.send_start()
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    my_state.state = my_state.NAVIGATE
                    # 测试
                    my_beep.test()
            if my_vision_manager.if_finish_calibrate == True:
                my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                my_state.state = my_state.NAVIGATE
                # 主车给从车发消息让从车完成矫正
                my_main_protocol.send_start()
                # 测试
                my_beep.test()
        # 让小车通过反向环绕恢复原位
        elif my_state.state == my_state.REVERSE_ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                # 重回扫描点继续寻找物体
                my_plan.return_to_scan_point = True
                my_state.state = my_state.NAVIGATE
                # 测试
                my_beep.test() 
    elif my_state.state_work == UP:
        if my_state.state == my_state.NAVIGATE:
                my_plan.navigate([[plan_data.fixed_point[2][0], plan_data.fixed_point[2][1]]], 180.0)
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    if my_vision_manager.failed_servo_count >= 2:
                        my_vision_manager.failed_servo_count = 0
                        my_state.state_work = CHECK
                        my_state.state = my_state.SCAN
                        my_vision_manager.my_order_manager.mode_target()
                    else:
                        my_state.state = my_state.SCAN
                        my_vision_manager.my_order_manager.mode_target()
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.SCAN:
                my_plan.navigate([[plan_data.fixed_point[4][0], plan_data.fixed_point[4][1]]], 180.0)
                if my_plan.finish_navigate == False:
                    target_point = my_art_protocol.coordinate_receive()
                    if target_point and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B')):
                        my_vision_manager.current_servo_object = target_point[2]
                        # 初始化视觉伺服偏航角缓冲区，使其过渡更平滑
                        sin_servo_fil.buffer_init(my_plan.scan_v_max)
                        cos_servo_fil.buffer_init(0)
                        ready_servo_and_orbit()
                        reset_navigate_flags()
                        my_state.state = my_state.SERVO
                else:
                    # 此时矩形上区域已没有物体，控制小车检查区域内是否还有物体遗漏
                    my_plan.finish_navigate = False
                    my_state.if_move_easy_object = True
                    my_state.state_work = CHECK
                    my_state.state = my_state.SCAN
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                # 若丢失物体则按矩形轨迹行驶寻找物体
                my_plan.navigate([[my_car.x_current+25.0, my_car.y_current], [my_car.x_current+25.0, my_car.y_current+30.0], [my_car.x_current-25.0, my_car.y_current+30.0], [my_car.x_current-25.0, my_car.y_current], [my_car.x_current, my_car.y_current]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point:
                    my_vision_manager.current_servo_object = target_point[2]
                    ready_servo_and_orbit()
                    reset_navigate_flags()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 重回扫描点继续寻找物体
                    my_plan.return_to_scan_point = True
                    my_state.state = my_state.NAVIGATE

            if my_vision_manager.finish_servo == True:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(my_vision_manager.current_servo_object, [[my_car.x_current, plan_data.fixed_point[2][1]], [my_car.x_current, my_car.y_current + 10.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    # 重置标志位
                    my_vision_manager.finish_servo = False
                    my_plan.if_send_path = False

                    my_state.state = my_state.ORBIT
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                order = my_main_protocol.get_slave_state()
                if order == "finish":
                    # 重置从车视觉伺服失败次数
                    my_vision_manager.failed_servo_count = 0
                    my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                    my_state.state = my_state.MOVE
                    # 测试
                    my_beep.test()
                elif order == "lost":
                    my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                    my_vision_manager.failed_servo_count += 1
                    my_state.state = my_state.REVERSE_ORBIT
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.MOVE:
                # 控制小车夹紧物体
                my_plan.navigate([[my_car.x_current + 5.0, 265.0]])
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.car_position = 2
                    my_state.state = my_state.CALIBRATE
                    # 测试
                    my_beep.test()
        elif my_state.state == my_state.CALIBRATE:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.apriltag_calibrate_control()
            else:
                # 控制小车前后移动寻找apriltag码
                if my_vision_manager.car_position == 0:
                    my_plan.navigate([[my_car.x_current+15.0, my_car.y_current], [my_car.x_current+15.0, my_car.y_current-20.0], [my_car.x_current-15.0, my_car.y_current-20.0], [my_car.x_current-15.0, my_car.y_current+20.0], [my_car.x_current+15.0, my_car.y_current+20.0]], 90)
                elif my_vision_manager.car_position == 2:
                    my_plan.navigate([[my_car.x_current+15.0, my_car.y_current], [my_car.x_current+15.0, my_car.y_current+20.0], [my_car.x_current-15.0, my_car.y_current+20.0], [my_car.x_current-15.0, my_car.y_current-20.0], [my_car.x_current+15.0, my_car.y_current-20.0]], 90)
                elif my_vision_manager.car_position == 1:
                    my_plan.navigate([[my_car.x_current-15.0, my_car.y_current], [my_car.x_current-15.0, my_car.y_current-20.0], [my_car.x_current+15.0, my_car.y_current-20.0], [my_car.x_current+15.0, my_car.y_current+20.0], [my_car.x_current-15.0, my_car.y_current+20.0]], -90)
                elif my_vision_manager.car_position == 3:
                    my_plan.navigate([[my_car.x_current-15.0, my_car.y_current], [my_car.x_current-15.0, my_car.y_current+20.0], [my_car.x_current+15.0, my_car.y_current+20.0], [my_car.x_current+15.0, my_car.y_current-20.0], [my_car.x_current-15.0, my_car.y_current-20.0]], -90)

                target_point = my_art_protocol.apriltag_receive()
                if target_point:
                    reset_navigate_flags()
                    my_vision_manager.calibrate_times = 0
                    my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车回到扫描点进行扫描
                if my_plan.finish_navigate == True:
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    # 主车给从车发消息让从车完成矫正
                    my_main_protocol.send_start()
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    my_state.state = my_state.NAVIGATE
                    # 测试
                    my_beep.test()
            if my_vision_manager.if_finish_calibrate == True:
                # 主车完成矫正后给从车发消息让从车完成矫正
                my_main_protocol.send_start()
                my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                if my_state.if_move_easy_object == False:
                    my_state.state = my_state.NAVIGATE
                else:
                    my_vision_manager.my_order_manager.mode_target()
                    my_state.state_work = CHECK
                    my_state.state = my_state.SCAN
                # 测试
                my_beep.test()
        # 让小车通过反向环绕恢复原位
        elif my_state.state == my_state.REVERSE_ORBIT:
            my_vision_manager.orbit_control(my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                # 重回扫描点继续寻找物体
                my_plan.return_to_scan_point = True
                my_state.state = my_state.NAVIGATE
                # 测试
                my_beep.test() 
    elif my_state.state_work == CHECK:
        if my_state.state == my_state.SCAN:
            my_plan.navigate([[210.0, 150.0], [110.0, 150.0]], 180.0)
            if my_plan.finish_navigate == False:
                target_point = my_art_protocol.coordinate_receive()
                if target_point and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B')):
                    my_vision_manager.current_servo_object = target_point[2]
                    ready_servo_and_orbit()
                    reset_navigate_flags()
                    my_state.state_work = UP
                    my_state.state = my_state.SERVO
            else:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(ord('P'), [[30.0, -20.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    my_plan.if_send_path = False
                    my_plan.finish_navigate = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 此时矩形区域内已没有物体，控制小车返回发车区
                    my_state.state_work = RETURN_WORK
                    my_state.state = my_state.RETURN
                    # 测试
                    my_beep.test()
    elif my_state.state_work == RETURN_WORK:
        if my_state.state == my_state.RETURN:
            my_plan.navigate([[plan_data.fixed_point[0][0], plan_data.fixed_point[0][1]]], 0.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.STOP
                my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                # 测试
                my_beep.test()
        elif my_state.state == my_state.STOP:
            my_plan.stop()

# 调试电机速度环pid函数
def show_speed_PID_test():
    global counter
    counter += 1
    motor_ul_pid.compute_pid(100, pose_data.encoder_data_ul)
    motor_ur_pid.compute_pid(100, pose_data.encoder_data_ur)
    motor_md_pid.compute_pid(100, pose_data.encoder_data_md)
    """
    if counter >= 6000:
        counter = 0
    elif counter >= 4500:
        motor_ur_pid.compute_pid(250, pose_data.encoder_data_ur)
    elif counter >= 3000:
        motor_ur_pid.compute_pid(50, pose_data.encoder_data_ur)
    elif counter >= 1500:
        motor_ur_pid.compute_pid(200, pose_data.encoder_data_ur)
    else:
        motor_ur_pid.compute_pid(150, pose_data.encoder_data_ur)
    """
        
""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    global ticker1_counter
    ticker1_counter = (ticker1_counter + 1) % 100
    # 更新传感器数据
    pose_data.update_data()

    # 初始化pid参数
    if motor_ul_pid.target >= 230:
        motor_ul_pid.set_pid_params(pid_data.ul_high_kp, pid_data.ul_high_ki, pid_data.ul_high_kd)
    elif motor_ul_pid.target >= 120:
        motor_ul_pid.set_pid_params(pid_data.ul_mid_kp, pid_data.ul_mid_ki, pid_data.ul_mid_kd)
    else:
        motor_ul_pid.set_pid_params(pid_data.ul_low_kp, pid_data.ul_low_ki, pid_data.ul_low_kd)
        
    if motor_ur_pid.target >= 230:
        motor_ur_pid.set_pid_params(pid_data.ur_high_kp, pid_data.ur_high_ki, pid_data.ur_high_kd)
    elif motor_ur_pid.target >= 120:
        motor_ur_pid.set_pid_params(pid_data.ur_mid_kp, pid_data.ur_mid_ki, pid_data.ur_mid_kd)
    else:
        motor_ur_pid.set_pid_params(pid_data.ur_low_kp, pid_data.ur_low_ki, pid_data.ur_low_kd)

    if motor_md_pid.target >= 230:
        motor_md_pid.set_pid_params(pid_data.md_high_kp, pid_data.md_high_ki, pid_data.md_high_kd)
    elif motor_md_pid.target >= 120:
        motor_md_pid.set_pid_params(pid_data.md_mid_kp, pid_data.md_mid_ki, pid_data.md_mid_kd)
    else:
        motor_md_pid.set_pid_params(pid_data.md_low_kp, pid_data.md_low_ki, pid_data.md_low_kd)
    
    # 更新小车姿态
    my_car.update_pose()
    
    # 角度环计算（8ms）
    if ticker1_counter % 4 == 0:
        angle_pid_compute()
    
    # 测试角度闭环
    # complete_angle_circle()
    
    # 全向定位测试程序
    # test_global_localization()
    
    # 陀螺仪测试
    # test_imu()
    # ant_else.my_uart3.write("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
    
    # 速度环测试
    show_speed_PID_test()
    
    # 总控制函数
    # master_control()

    # 设置电机pwm输出
    my_car.set_motor_pwm()



# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(time) -> None:
    # 角度环计算（12ms）
    # angle_pid_compute()

    # 任务执行机
    # task_machine()
    # collaborative_task_machine()

    # 全向定位测试程序
    
    if my_state.state == my_state.NAVIGATE:
        my_plan.navigate([[0.0, 175.0]], 0.0)
        if my_plan.finish_navigate == True:
            my_plan.finish_navigate = False
            my_state.state = my_state.STOP
            my_beep.test()
    elif my_state.state == my_state.STOP:
        my_plan.stop()
    

    # 拍数据集程序
    """
    if my_state.state_work == UP:
        if my_state.state == my_state.NAVIGATE:
            my_plan.navigate([[-120.0, 0.0]], 0.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state_work = DOWN
    elif my_state.state_work == DOWN:
        if my_state.state == my_state.NAVIGATE:
            my_plan.navigate([[-120.0, 120.0]], 90.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state_work = CHECK
    elif my_state.state_work == CHECK:
        if my_state.state == my_state.NAVIGATE:
            my_plan.navigate([[0.0, 120.0]], 180.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state_work = RETURN_WORK
    elif my_state.state_work == RETURN_WORK:
        if my_state.state == my_state.NAVIGATE:
            my_plan.navigate([[0.0, 0.0]], -90.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state_work = UP
    """
    # my_plan.navigate([plan_data.fixed_point[1], plan_data.fixed_point[3], plan_data.fixed_point[2], plan_data.fixed_point[0]])
    
    # 视觉伺服测试程序
    # test_vision_servo()

    # 边线和apriltag码校准测试程序
    # test_apriltag_calibrate()

    # 环绕物体测试程序
    # test_orbit_control()
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
        
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_plan.rest_distance, my_plan.v_target, my_car.now_yaw * 180 / MATH.PI, my_plan.v_max))
    my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual))

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
    pit2.capture_list(key)
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
    if ticker1_counter % 10 == 0:
        # 视觉伺服
        # my_uart3.write(f"{servo_pid.actual_x},{servo_pid.target_x},{servo_pid.pwm_output_x},{servo_pid.current_y},{servo_pid.target_y},{servo_pid.pwm_output_y},{my_vision_manager.target_rel_yaw}\n")
        # my_uart3.write(f"servo_pid.target_y: {servo_pid.target_y}, object_radius: {my_vision_manager.object_radius}, orbit_angle: {my_vision_manager.orbit_angle}\n")
        # my_uart3.write(f"{servo_pid.actual_x},{servo_pid.target_x},{servo_pid.pwm_output_x}\n")
        # my_uart3.write("x: {:<f}, y: {:<f}, speed: {:<f}, yaw: {:<f}\n".format(servo_pid.actual_x, servo_pid.actual_y, my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw))
        
        # 速度环输出波形图调参
        # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ul_pid.pwm_output, motor_ul_pid.derivative * motor_ul_pid.kd, motor_ul_pid.integral))
        # my_uart3.write("{:<f},{:<f},{:<f},{:<f}\n".format(motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd, motor_ur_pid.integral))
        # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_md_pid.target, motor_md_pid.actual, motor_md_pid.pwm_output, motor_md_pid.derivative * motor_md_pid.kd, motor_md_pid.integral))
        # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual))
            
        # 航向角输出
        # my_uart3.write(f"{my_car.last_move_yaw}\n")
        # 角度环输出
        # my_uart3.write(f"{angle_pid.pwm_output},{angle_pid.target},{angle_pid.actual},{angle_pid.derivative}\n")
        # imu原始数据
        # my_uart3.write("acc = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[0], pose_data.imu_data[1], pose_data.imu_data[2]))
        # my_uart3.write("gyro = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[3], pose_data.imu_data[4], pose_data.imu_data[5]))
                                                                            
        # 里程计：
        # my_uart3.write("ul: {:<f}, ur: {:<f}, md: {:<f}\n".format(my_car.encouder_ul, my_car.encouder_ur, my_car.encouder_md))
        # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_plan.rest_distance, my_plan.v_target, my_car.now_yaw * 180 / MATH.PI, my_plan.v_max))
        # my_uart3.write(f"{pose_data.encoder_data_ul}, {pose_data.encoder_data_ur}, {pose_data.encoder_data_md}, {pose_data.encoder_data_ul_2}, {pose_data.encoder_data_ur_2}, {pose_data.encoder_data_md_2}\n")

        # tof传感器测试
        # my_uart3.write(f"{tof_distance_fil.update(tof.get())},{tof.get()}\r\n")
        
        # 速度规划
        # my_uart3.write("{:<f},{:<f},{:<f},{:<f}\n".format(my_plan.rest_distance, my_plan.v_target, my_plan.v_max, my_state.state))

        # 检测自转角是否准确
        # my_uart3.write("{:<f}\n".format(my_car.now_yaw * 180 / MATH.PI))
        
        # 检测gkd项数量级
        # my_uart3.write(f"{pose_data.gyro_z * my_car.gkd}, {pose_data.gyro_z}\n")
        
        # 环绕测试
        # my_uart3.write(f"{my_vision_manager.orbit_turn_angle}\n")
        
        # 任务机
        # my_uart3.write(f"state_work: {my_state.state_work}, state: {my_state.state}, yaw: {my_car.now_yaw * 180 / MATH.PI}, current_object: {my_vision_manager.current_servo_object}, {my_plan.turn_angle_target}\n")
        pass
        
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        gc.collect()
        break

    gc.collect()