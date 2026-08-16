# 包含 gc 与 time 类
import gc
import time
import os
from micropython import const

def max_block():
    gc.collect()
    low = 0
    high = gc.mem_free()
    while low + 16 < high:
        mid = (low + high) // 2
        try:
            b = bytearray(mid)
            del b
            low = mid
        except MemoryError:
            high = mid
        gc.collect()
    return low

def mem(tag):
    print(tag,gc.mem_free(),gc.mem_alloc(),max_block())
    #my_uart3.write(f"{tag},{gc.mem_free()}, {gc.mem_alloc()}, {max_block()}\n")
    gc.collect()

# ==================== 内存分配追踪（带标签 + 增量 diff） ====================
MEM_TRACE = True    # 总开关：True 打印追踪，False 全部静默（不改动业务逻辑）

_mem_inited = False
_mem_last_alloc = 0
_mem_last_free = 0

def mem_trace(tag):
    """打印带标签的内存分配进度及相对上一步的增量。
    内部先 gc.collect()，测到的是该步的净持久占用，便于定位内存分配异常。"""
    global _mem_inited, _mem_last_alloc, _mem_last_free
    if not MEM_TRACE:
        return
    gc.collect()
    alloc = gc.mem_alloc()
    free = gc.mem_free()
    if not _mem_inited:
        print("[MEM] %-26s alloc=%d free=%d" % (tag, alloc, free))
        _mem_inited = True
    else:
        print("[MEM] %-26s alloc=%d(%+d) free=%d(%+d)" % (tag, alloc, alloc - _mem_last_alloc, free, free - _mem_last_free))
    _mem_last_alloc = alloc
    _mem_last_free = free

mem_trace("baseline")
# 从 machine 库包含所有内容
from machine import *
mem_trace("import machine")
from seekfree import MOTOR_CONTROLLER, IMU660RX, KEY_HANDLER, BLDC_CONTROLLER
mem_trace("import seekfree")
from smartcar import ticker, encoder
mem_trace("import smartcar")
my_uart_debug = UART(7)
my_uart_debug.init(115200)
mem_trace("my_uart_debug")
import ant_task
mem_trace("import ant_task")
import ant_move
mem_trace("import ant_move")
import ant_plan
mem_trace("import ant_plan")
import ant_vision
mem_trace("import ant_vision")
import ant_boundary_plan
mem_trace("import ant_boundary_plan")
import ant_motor
mem_trace("import ant_motor")
import ant_else
mem_trace("import ant_else")
#import ant_pid
#gc.collect()

###################################【变量定义及初始化】###################################
_PI = const(3.1415926)
_READY_NAVIGATE = const(0)   # 准备导航状态
_NAVIGATE = const(1)       # 导航状态
_SCAN = const(2)           # 扫描状态
_SERVO = const(3)          # 视觉伺服状态
_ORBIT = const(4)          # 环绕状态
_MOVE = const(5)           # 搬运状态
_CALIBRATE = const(6)      # 校准状态
_ADJUST = const(7)           # 微调状态
_RETURN = const(8)		    # 返回状态
_STOP = const(9)           # 停止状态
_RETREAT = const(10) 

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
mem_trace("my_beep")

"""光电管初始化"""
photo = Pin('B4', Pin.IN, value = False)

"""异步串口通信初始化"""
my_uart6 = UART(5)
my_uart6.init(115200)

"""无线串口通信初始化"""
my_uart3 = UART(2)
my_uart3.init(115200)
my_uart2 = UART(1)
my_uart2.init(115200)
os.dupterm(my_uart2)
"""电机初始化"""
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = True)
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = True)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = False)
mem_trace("motors")

"""传感器初始化"""
# 编码器初始化
encoder_ul = encoder("C2" , "C3" , True)
encoder_ur = encoder("D13", "D14", True)
encoder_md = encoder("D16", "D15", True)
mem_trace("encoders")

"""无刷风扇初始化"""
fan = BLDC_CONTROLLER(BLDC_CONTROLLER.PWM_C25, freq=300, highlevel_us = 1000)

# IMU初始化
imu = IMU660RX()

# tof深度传感器初始化
# tof = DL1X()
# 与定时器2周期一致，都为53ms
key = KEY_HANDLER(53)
key_data = key.get()
mem_trace("sensors")
"""""""""创建对象"""""""""
# 创建状态机对象
my_state = ant_plan.StateMachine()
mem_trace("my_state")
#【文件读取】
# 从main_config.txt中读取保存所有的参数并保存到config字典中
my_flash_sys = ant_else.flash_system(my_beep, "/flash/main_config.txt")
my_flash_sys.phase_config()
my_flash_sys.check_list_format()

# 这三个参数在对象创建完成后才使用，释放配置字典前先缓存。
_timer_periods = (
    my_flash_sys.find_value("motor_control_T"),
    my_flash_sys.find_value("uart_and_menu_T"),
    my_flash_sys.find_value("plan_calculate_T"),
)

mem_trace("my_flash_sys")
my_write_system = ant_else.write_system(my_flash_sys)
mem_trace("my_write_system")

# 创建无刷风扇控制对象
my_fan = ant_motor.FanControl(my_flash_sys, fan, my_state)
mem_trace("my_fan")

# 创建光电管控制对象
my_photo = ant_motor.PhotoControl(my_flash_sys, my_beep, photo)
mem_trace("my_photo")

# 创建指令管理对象
my_order_manager = ant_else.order_manager(my_flash_sys,my_uart6)
mem_trace("my_order_manager")

# 创建openart串口解析对象
my_art_protocol = ant_else.UARTProtocol(my_uart6)
mem_trace("my_art_protocol")

# 创建主从车无线串口通信对象
my_main_protocol = ant_else.LinkProtocol(my_uart3)
mem_trace("my_main_protocol")

# PID参数按 UL/UR/MD、high/mid/low、kp/ki/kd 存入一个扁平元组。
pid_gains = ant_motor.load_pid_gains(my_flash_sys)
mem_trace("pid_gains")

# 创建电机微分项的滑动平均滤波器对象
diff_filter_ul = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为2个
diff_filter_ur = ant_motor.SlipAveragingFilter(3)    # 滤波窗口为3个
diff_filter_md = ant_motor.SlipAveragingFilter(5)    # 滤波窗口为2个
diff_filter_gyroz = ant_motor.SlipAveragingFilter(1)  # 滤波窗口为1个
mem_trace("diff_filters")

# 创建加速度计滤波对象
acc_x_fil = ant_motor.SlipAveragingFilter(5)
acc_y_fil = ant_motor.SlipAveragingFilter(5)
acc_z_fil = ant_motor.SlipAveragingFilter(5)
acc_z_fil.buffer_init(4096)  # 初始化z轴加速度计滤波器的初始值为4096
mem_trace("acc_filters")

# 创建小车自转角滤波器对象（已弃用）
car_yaw_fil = ant_motor.SlipAveragingFilter(1)
# 创建视觉伺服正余弦滤波对象
sin_servo_fil = ant_motor.SlipAveragingFilter(5)
cos_servo_fil = ant_motor.SlipAveragingFilter(5)
mem_trace("servo_filters")

# 创建姿态数据对象
pose_data = ant_motor.PoseData(my_flash_sys, my_uart2, imu, encoder_ul, encoder_ur, encoder_md, diff_filter_gyroz, acc_x_fil, acc_y_fil, acc_z_fil)
mem_trace("pose_data")

# 创建电机pid对象和角度pid对象
motor_ul_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ul)
motor_ur_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_ur)
motor_md_pid = ant_motor.SpeedPositionPID(my_flash_sys, diff_filter = diff_filter_md)
angle_pid = ant_motor.AnglePositionPID(my_flash_sys)
servo_pid = ant_motor.ServoPID(my_flash_sys)
mem_trace("pids")

# 创建小车姿态对象
my_car = ant_motor.CarPose(my_flash_sys, my_state, pose_data, car_yaw_fil, angle_pid, motor_ul_pid, motor_ur_pid, motor_md_pid,
                        motor_ul, motor_ur, motor_md)
mem_trace("my_car")

# 创建路径规划数据对象
plan_data = ant_plan.PlanData(my_flash_sys)
mem_trace("plan_data")

# 创建路径规划对象
my_path = ant_plan.PathPlan(plan_data, my_car)
mem_trace("my_path")

# 创建规划（路径和速度）对象
my_plan = ant_plan.NavigationPlan(my_flash_sys, plan_data, my_fan, my_car, my_state, my_order_manager, my_uart3, my_beep, my_art_protocol,angle_pid)
mem_trace("my_plan")

move_plan = ant_boundary_plan.BoundaryPathPlanner(plan_data, my_car, my_path,my_flash_sys)
mem_trace("move_plan")

my_obj_plan = ant_boundary_plan.objects_planner(my_flash_sys,my_write_system,plan_data,my_car,my_plan,move_plan)
mem_trace("my_obj_plan")

# 创建视觉伺服管理对象2
my_vision_manager = ant_vision.VisionManager(my_flash_sys, my_beep, pose_data,  angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, my_car, my_art_protocol, my_order_manager, my_plan, my_state)
mem_trace("my_vision_manager")

# 搬运控制类
my_moving = ant_move.MoveControl(my_write_system,my_flash_sys,my_beep, my_photo, my_car, my_plan,my_path, plan_data,move_plan, my_vision_manager, my_state, my_main_protocol, my_art_protocol, my_order_manager,my_uart3, angle_pid,my_obj_plan)
mem_trace("my_moving")

# 任务及类
my_task = ant_task.TaskController(my_write_system,my_flash_sys,my_obj_plan,my_beep, my_state, my_uart3, my_car, my_path, my_plan, my_vision_manager,  my_moving, plan_data, my_order_manager, my_art_protocol,  my_main_protocol, my_uart_debug)
mem_trace("my_task")

# 所有构造器已经读完参数，断开对大配置字典的引用。
# 如果重新启用 ant_menu，必须在此处之前创建菜单对象。
my_flash_sys.release_config()
mem_trace("release config")

# 测试打印变量解析是否成功
"""
print("fixed+point:", plan_data.fixed_point)
print("center_rect:", plan_data.center_rect)
print("rectangle_obstacles:", plan_data.rectangle_obstacles)
"""

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
        my_beep.low_power_warn()

# 角度环计算函数
def angle_pid_compute():
    # 计算z轴的目标速度
    angle_pid.compute_pid(my_car.turn_angle_target, my_car.now_yaw * 180 / _PI)

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
                # 盲盒任务测试，一定要修改！！！
                my_state.state = _READY_NAVIGATE
                # my_state.state = _RETURN 
                start_flag = True
                # 延时1秒避免零漂校准不准确
                time.sleep_ms(1000)
                # 打开定时器1和3
                pit1_start()
                pit3_start()
                # 检测是否正常初始化所有
                detect_if_normal()
                # 初始化小车坐标及姿态角
                my_car.x_current = plan_data.fixed_point[0][0]
                my_car.y_current = plan_data.fixed_point[0][1]
                my_car.now_yaw = 0.0


# 小车姿态总控制函数
def master_control():
    if my_state.state in [_NAVIGATE, _READY_NAVIGATE, _RETURN, _STOP, _SCAN, _RETREAT]:
        my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == _MOVE:
        if my_moving.current_state == _ORBIT:
            my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)
        elif my_moving.current_state in [_SERVO, _ADJUST]:
            if not my_vision_manager.if_lost_object:
                my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
            else:
                my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        elif my_moving.current_state in [_NAVIGATE, _SCAN]:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        elif my_moving.current_state == _MOVE:
            if my_plan.fitting_path_:my_car.move_ctrl(my_plan.target_v, my_plan.fit_target_yaw, my_plan.turn_angle_target)
            else:my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state in [_SERVO, _ADJUST]:
        # 未丢失物体时正常进行视觉伺服控制，丢失物体时进行矩形轨迹的导航控制
        if my_vision_manager.if_lost_object == False:
            my_car.move_ctrl(my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, my_vision_manager.target_rel_turn_angle)
        else:
            my_car.move_ctrl(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
    elif my_state.state == _ORBIT:
        my_car.move_ctrl(my_vision_manager.orbit_speed, my_vision_manager.orbit_yaw, my_vision_manager.orbit_turn_angle)

# 根据目标速度选择对应挡位的PID参数（gain scheduling）
# 阈值常量
_HIGH_TARGET = 180         # >= 此值使用High挡
_MID_TARGET = 120          # >= 此值使用Mid→High线性插值
_LOW_TARGET = 50           # >= 此值使用Low→Mid线性插值，< 此值使用Low挡
_PID_UL = const(0)
_PID_UR = const(9)
_PID_MD = const(18)
_PID_HIGH = const(0)
_PID_MID = const(3)
_PID_LOW = const(6)

def _apply_pid_params(motor_pid, base):
    motor_pid.set_pid_params(
        pid_gains[base], pid_gains[base + 1], pid_gains[base + 2])

def _select_pid_params(motor_pid, motor_base):
    """为单个电机按目标速度选择并设置PID参数"""
    target_abs = abs(motor_pid.target)
    high = motor_base + _PID_HIGH
    mid = motor_base + _PID_MID
    low = motor_base + _PID_LOW

    # 刹车条件：目标接近0但误差很大 → 高挡参数强力纠正
    if target_abs >= _HIGH_TARGET:
        _apply_pid_params(motor_pid, high)
    elif target_abs >= _MID_TARGET:
        ratio = (target_abs - _MID_TARGET) / (_HIGH_TARGET - _MID_TARGET)
        motor_pid.set_pid_params(
            pid_gains[mid] + (pid_gains[high] - pid_gains[mid]) * ratio,
            pid_gains[mid + 1] + (pid_gains[high + 1] - pid_gains[mid + 1]) * ratio,
            pid_gains[mid + 2] + (pid_gains[high + 2] - pid_gains[mid + 2]) * ratio)
    elif target_abs >= _LOW_TARGET:
        ratio = (target_abs - _LOW_TARGET) / (_MID_TARGET - _LOW_TARGET)
        motor_pid.set_pid_params(
            pid_gains[low] + (pid_gains[mid] - pid_gains[low]) * ratio,
            pid_gains[low + 1] + (pid_gains[mid + 1] - pid_gains[low + 1]) * ratio,
            pid_gains[low + 2] + (pid_gains[mid + 2] - pid_gains[low + 2]) * ratio)
    else:
        _apply_pid_params(motor_pid, low)


def set_pid_params():
    if my_state.state == _MOVE:
        _apply_pid_params(motor_ul_pid, _PID_UL + _PID_HIGH)
        _apply_pid_params(motor_ur_pid, _PID_UR + _PID_HIGH)
        _apply_pid_params(motor_md_pid, _PID_MD + _PID_HIGH)
    else:
        _select_pid_params(motor_ul_pid, _PID_UL)
        _select_pid_params(motor_ur_pid, _PID_UR)
        _select_pid_params(motor_md_pid, _PID_MD)

# 任务机执行函数
def task_machine():
    my_task.run()

""" 定时器类 """
# 定时器1中断回调函数
def time_pit1_handler(time):
    # 更新传感器数据
    pose_data.update_data()

    # 更新小车姿态
    my_car.update_pose()
    
    # 总控制函数
    master_control()

    # 更新pid参数
    set_pid_params()

    # 设置电机pwm输出
    my_car.set_motor_pwm()


time__ = 0
# 定时器3中断处理函数：路径规划与速度规划计算
def time_pit3_handler(timer) -> None:
    global time__
    # 角度环计算（10ms）
    angle_pid_compute()

    # 任务执行机
    task_machine()

    """
    # 全向定位测试程序
    if my_state.state == _READY_NAVIGATE:
        my_path.plan_path(220.0, 230.0)
        print(f"ready_path: {my_path.ready_path}\n")
        my_state.state = _NAVIGATE
        time__ = time.ticks_ms()
        my_car.x_current = 0.0
        my_car.y_current = 0.0
    elif my_state.state == _NAVIGATE:
        my_plan.navigate(path = my_path.ready_path, target_turn_angle = 0.0)
        # my_plan.navigate(path = [[50.0, 30.0], [50.0, 100.0]], target_turn_angle = 30.0)
        # my_plan.navigate(path = [[160,0],[160,240],[0,240],[-160,240],[-160,0],[0,0]])
        # my_plan.navigate(path = [[-100,20.0],[50, 100.0],[0,240],[130,70],[100,-30],[-10,60],[20,10],[0,0]])
        # my_main_protocol.send_pose(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        # my_plan.navigate(path = [[0.0, 120.0]])
        # my_plan.navigate(path = [[0.0, 80.0], [80.0, 80.0], [80.0, 0.0], [0.0, 0.0], [80.0, 0.0], [80.0, 80.0], [0.0, 80.0], [0.0, 0.0]])
        if my_plan.if_finish_navigate == True:
            my_plan.reset_navigate()
            my_plan.reset_navigate_angle()
            my_state.state = _STOP
            my_beep.test()
            print(f"Navigation finished in {time.ticks_diff(time.ticks_ms(), time__)} ms")
    elif my_state.state == _STOP:
        my_plan.stop()
        my_uart3.write(f"x: {my_car.x_current},y: {my_car.y_current}\n")
    """
    """
    if my_state.state == _READY_NAVIGATE:
        my_path.plan_path(220.0, 220.0)
        my_uart3.write(f"ready_path: {my_path.ready_path}\n")
        # my_state.state = _MOVE
        my_plan.keep_x_or_y_v = False
        # my_moving.current_state = _MOVE
        my_plan.move_state = _MOVE
        my_plan.move_v_max = 160
        my_car.x_current = 0.0
        my_car.y_current = 0.0
    elif my_state.state == _MOVE:
        my_plan.navigate(path = [[50.0, 50.0], [50.0, 150.0]], target_turn_angle = 45.0)
        # my_plan.navigate(path = [[160,0],[160,240],[0,240],[-160,240],[-160,0],[0,0]])
        # my_plan.navigate(path = [[-100,20.0],[50, 100.0],[0,240],[130,70],[100,-30],[-10,60],[20,10],[0,0]])
        # my_main_protocol.send_pose(my_plan.target_v, my_plan.target_yaw, my_plan.turn_angle_target)
        # my_plan.navigate(path = [[0.0, 120.0]])
        # my_plan.navigate(path = [[0.0, 80.0], [80.0, 80.0], [80.0, 0.0], [0.0, 0.0], [80.0, 0.0], [80.0, 80.0], [0.0, 80.0], [0.0, 0.0]])
        if my_plan.if_finish_navigate == True:
            my_plan.reset_navigate()
            my_plan.reset_navigate_angle()
            my_state.state = _STOP
            my_beep.test()
    elif my_state.state == _STOP:
        my_uart3.write(f"main_car: {my_car.x_current},{my_car.y_current}\n")
        my_plan.stop()
        # my_uart3.write(f"x: {my_car.x_current},y: {my_car.y_current}\n")
    """
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

    # my_uart2.write(f"{my_state.state}\r\n")
    # my_uart2.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ul_pid.pwm_output, motor_ul_pid.derivative * motor_ul_pid.kd, motor_ul_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd, motor_ur_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_md_pid.target, motor_md_pid.actual, motor_md_pid.pwm_output, motor_md_pid.derivative * motor_md_pid.kd, motor_md_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual, my_plan.target_v))
    # my_uart3.write(f"{angle_pid.kp},{angle_pid.target},{angle_pid.actual},{angle_pid.pwm_output}\n")
    # my_uart3.write(f"{my_vision_manager.if_lost_object}\r\n")
    # my_uart3.write(f"{my_moving.current_state},{my_vision_manager.if_lost_object}\r\n")
    # my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.acc_x},{pose_data.acc_y},{pose_data.acc_z},{pose_data.gyro_x},{pose_data.gyro_y},{pose_data.gyro_z}\n")
    # my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw},{pose_data.gyro_z}\n")
    # my_uart3.write("{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current))
    # my_uart3.write(f"{my_car.alpha_x},{my_car.alpha_y}\r\n")
    # my_uart3.write(f"{servo_pid.actual_x},{servo_pid.target_x},{servo_pid.pwm_output_x},{servo_pid.actual_y},{servo_pid.target_y},{servo_pid.pwm_output_y},{my_vision_manager.target_rel_yaw}\n")
    # my_uart3.write(f"{my_car.now_yaw * 180 / _PI}\n")
    # my_uart3.write(f"{my_vision_manager.target_rel_speed},{my_vision_manager.target_rel_yaw},{my_vision_manager.target_rel_turn_angle},{my_car.now_yaw * 180 / _PI}\n")
    # my_uart3.write(f"{my_state.state}\n")
    # my_uart3.write(f"{my_moving.current_state}\r\n")

# 定时器1初始化（中断回调函数在 ant_motor 中）
def pit1_start():
    global imu_data, pit1
    pit1.capture_list(imu, encoder_ul, encoder_ur, encoder_md)
    # 进行IMU零漂校准并将imu_data与定时器1的底层采集绑定
    pose_data.init_bias()
    pit1.callback(time_pit1_handler)
    # 底层为4ms定时器
    pit1.start(_timer_periods[0])

# 定时器2初始化（中断回调函数在 ant_menu 中）
def pit2_start():
    global pit2
    pit2.callback(time_pit2_handler)
    pit2.capture_list(key)
    pit2.start(_timer_periods[1])

# 定时器3初始化（中断回调函数在 ant_plan 中）
def pit3_start():
    global pit3
    pit3.callback(time_pit3_handler)
    # 规划为10ms定时器
    pit3.start(_timer_periods[2])

###################################【主程序模块】###################################
# 检测电源电压是否正常
voltage_detect(11.2)

pit2_start()

while True:
    if my_state.state == _READY_NAVIGATE and my_task.if_start_task:
        if my_task.if_transitioning:
            my_task.enter()
        if not my_task.if_end_first_scan:
            my_task.exit()
            continue

        if not my_task.if_choose_object:
            if my_task.now_objects:
                if my_obj_plan.judge_object_character(my_task.now_objects, my_task.last_side):
                    mem("JUDGE_COMPLETE")
                    target = my_obj_plan.plan_target
                    my_task.if_end_first_scan = True
                    #my_write_system.write_str(f"final_objects:{my_task.now_objects}\n")
                    #my_write_system.write_str(f"target{my_task.object_plan.target_objects}\n")
                    #my_write_system.write_str(f"path{my_task.object_plan.path}\n")
                    #my_write_system.write_str(f"score{my_task.object_plan.target_score}\n")
                    my_obj_plan.target_objects = []
                    my_obj_plan.target_score = []
                    my_obj_plan.path = []
                    my_obj_plan.now_objects = []
                    gc.collect()
                    if not target:
                        #self.my_uart.write("False\n")
                        my_task.exit()
                    else:
                        my_obj_plan.barrier.pop(target[0])
                        my_task.now_objects.pop(target[0])
                        my_moving.now_barriar=my_obj_plan.barrier[:]
                        #self.my_uart.write(f"barriar{my_task.my_moving.now_barriar}\n")
                        my_task.current_object=target[1]
                        my_vision_manager.current_servo_object = my_task.current_object
                        # print(f"READY_START{time.ticks_ms()}\n")
                        rm = my_moving.ready_move([target[2],target[3]],now_side = my_task.last_side,target_side = target[4],RECT = target[5],Num = target[6])
                        mem("READYEND")
                        # self.my_uart.write(f"car_position:{my_task.my_moving.push_postion}\n")
                        if rm:
                            my_moving.saved_best_path =my_task.object_plan.best_path
                            #num_compensation = my_task.data.current_index * my_task.num_clamp_factor
                            #my_moving.clamp_distance = my_task.clamp_distance[my_task.current_object]+num_compensation
                            my_task.if_choose_object = True
                            my_plan.reset_navigate()
                        else:my_task.exit()
            else:my_task.exit()
        else:
            if plan_data.current_index >=plan_data.total_objects_num:
                my_task.my_state.state = _RETURN
                my_task.if_transitioning = True
                continue
            # 进入准备导航状态，做好路径规划准备和导航信息准?
            planned_path = my_moving.navigate_buffer['MAIN_P']
            insert_point = []
            retreat_lenth = 25
            if my_task.last_side == "L":
                if my_task.if_first_round:my_task.if_first_round = False
                else:insert_point = [my_task.my_car.x_current+retreat_lenth,my_task.my_car.y_current]
            elif my_task.last_side == "R":
                if my_task.if_first_round:my_task.if_first_round = False
                else:insert_point = [my_task.my_car.x_current-retreat_lenth,my_task.my_car.y_current]
            elif my_task.last_side == "U":
                if my_task.if_first_round:my_task.if_first_round = False
                else:insert_point = [my_task.my_car.x_current,my_task.my_car.y_current-retreat_lenth]
            else:
                if my_task.if_first_round:my_task.if_first_round = False
                else:insert_point = [my_task.my_car.x_current,my_task.my_car.y_current+retreat_lenth]
            if insert_point:planned_path = [insert_point] + planned_path
            my_task.my_moving.navigate_buffer['MAIN_P'] = planned_path
            my_task.exit()  # 退出当前状态，进入导航状态
    gc.collect()
