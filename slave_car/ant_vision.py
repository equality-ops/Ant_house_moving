from micropython import const
import math
import gc

PI = const(3.1415926)
READY_NAVIGATE = const(0) # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)         # 微调状态
RETURN = const(8)		  # 返回状态
STOP = const(9)           # 停止状态
RETREAT = const(10)       # 后退状态

# 多路复用器计数器
counter = 0

# 视觉伺服控制类(PD控制器)
# 视觉伺服控制类(PD控制器)
class VisionManager:
    def __init__(self, flash_sys, beep, pose_data, angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, car, protocol, order_manager, plan, state):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入传感器数据对象
        self.pose_data = pose_data
        # 注入角度环pid对象
        self.angle_pid = angle_pid
        # 注入伺服PD控制器对象
        self.servo_pid = servo_pid
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入正弦滑动平均滤波器对象
        self.sin_servo_fil = sin_servo_fil
        # 注入余弦滑动平均滤波器对象
        self.cos_servo_fil = cos_servo_fil
        # 注入无线串口对象，用于调试
        self.my_uart3 = my_uart3
        # 注入小车姿态控制对象
        self.my_car = car
        # 注入通信协议对象
        self.my_art_protocol = protocol
        # 注入指令管理对象
        self.my_order_manager = order_manager
        # 注入路径规划对象
        self.my_plan = plan
        # 注入状态机对象
        self.my_state = state

        # 当前伺服的物品种类
        self.current_servo_object = ''
        # 当前伺服连续丢失物体的帧数
        self.servo_lost_count = 0
        # 视觉伺服失败的次数
        self.failed_servo_count = 0 
        # 最终小车停在物体前的距离（随着物体种类改变）
        self.final_dist_x = 0.0
        self.final_dist_y = 0.0
        # 视觉伺服的两个阶段：第一阶段为快速接近阶段，第二阶段为精确调整阶段
        self.servo_stage = 1
        # PD控制相关变量
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")  # type: float  # 视觉伺服控制距离阈值
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")  # type: float  # 视觉伺服控制距离阈值
        self.target_rel_speed_x = 0.0          # type: float   # 伺服控制目标x速度
        self.target_rel_speed_y = 0.0          # type: float   # 伺服控制目标y速度
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")  # type: float   # 视觉伺服控制最大速度
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")  # type: float   # 视觉伺服控制最小速度 
        self.target_point = []                      # type: list   # 目标点像素坐标
        self.target_rel_speed = 0.0                 # type: float     # 目标速度
        self.target_rel_yaw = 0.0                   # type: float   # 目标航向角
        self.target_rel_turn_angle = 0.0            # type: float   # 目标转角
        # 解算后的物体与小车的相对位置偏差
        self.relative_raw_x = 0.0
        self.relative_raw_y = 0.0
        self.last_relative_raw_y = 0.0
        self.relative_actual_x = 0.0
        self.relative_actual_y = 0.0
        self.actual_dist = 0.0
        # 解算后的物体与小车的绝对位置偏差（相对于世界坐标系下）
        self.absolute_actual_x = 0.0
        self.absolute_actual_y = 0.0
        # 视觉伺服完成的预测点位
        self.real_servo_point = [0, 0]

        # 小车上一帧记录的坐标
        self.last_car_x = 0.0
        self.last_car_y = 0.0
        # ==================================================================

        # 环绕控制相关变量
        self.orbit_center_x = 0.0
        self.orbit_center_y = 0.0
        self.orbit_radius = 0.0            # type: float   # 环绕半径
        self.orbit_speed = 0.0             # type: float     # 环绕速度
        self.orbit_yaw = 0.0               # type: float   # 环绕航向角
        self.orbit_turn_angle = 0.0        # type: float   # 环绕转角
        self.current_dis = 0.0             # type: float   # 当前距离
        self.target_angle = 0.0            # type: float   # 目标角度
        self.orbit_v_max = self.flash_sys.find_value("orbit_v_max")   # type: int   # 环绕最大速度
        self.orbit_v_min = self.flash_sys.find_value("orbit_v_min")   # type: int   # 环绕最小速度
        self.object_radius = 0.0           # type: float   # 物体半径
        self.orbit_angle = 0.0             # type: float   # 环绕角度
        self.record_angle = 0.0            # type: float   # 记录的角度(记录小车的最初的角度)
        self.radius_T_U = self.flash_sys.find_value("radius_T_U")   # type: float   # 网球半径
        self.radius_T_R = self.flash_sys.find_value("radius_T_R")   # type: float   # 网球半径
        self.radius_T_L = self.flash_sys.find_value("radius_T_L")   # type: float   # 网球半径
        self.radius_T_D = self.flash_sys.find_value("radius_T_D")   # type: float   # 网球半径
        self.radius_S_U = self.flash_sys.find_value("radius_S_U")   # type: float   # 网球半径
        self.radius_S_R = self.flash_sys.find_value("radius_S_R")   # type: float   # 网球半径
        self.radius_S_L = self.flash_sys.find_value("radius_S_L")   # type: float   # 网球半径
        self.radius_S_D = self.flash_sys.find_value("radius_S_D")   # type: float   # 网球半径
        self.radius_B_U = self.flash_sys.find_value("radius_B_U")   # type: float   # 网球半径
        self.radius_B_R = self.flash_sys.find_value("radius_B_R")   # type: float   # 网球半径
        self.radius_B_L = self.flash_sys.find_value("radius_B_L")   # type: float   # 网球半径
        self.radius_B_D = self.flash_sys.find_value("radius_B_D")   # type: float   # 网球半径
        self.angle_T = self.flash_sys.find_value("angle_T")     # type: float   # 网球环绕角度
        self.angle_S = self.flash_sys.find_value("angle_S")     # type: float   # 沙袋环绕角度
        self.angle_B = self.flash_sys.find_value("angle_B")     # type: float   # 玩具熊环绕角度
        self.direct = 'CW'  # 'CW'为顺时针(Clockwise)，'CCW'为逆时针(Counter-Clockwise)
        self.correct_dist = 10.51    # 经验修正值（物体在推杆正前方的值）
        # apriltag码矫正相关变量
        # 延时计数器
        self.counter = 0       # type: int     # 延时计数器
        self.assist_car_pos = []    # type: list    # 辅助车的位置位置
        # 目标横或纵坐标缓冲区
        self.point_buffer = []     # type: list    # 目标横或纵坐标缓冲区
        # 目标角度缓冲区
        self.angle_buffer = []     # type: list    # 目标角度缓冲区      
        # 边线矫正时小车位置
        self.car_position = 'L'  # 'L', 'R', 'U', 'D'分别代表小车在左边线、右边线、上边线、下边线
        self.rel_pos_to_apriltag = 'left'  # 'left'为左侧，'right'为右侧
        # 临时用于测试的角度变量
        self.angle_temp = 0.0
        # 停在物体的左侧或者右侧
        self.direction = ''  
        self.last_real_servo_point = None
        # 标志位
        self.if_send_order = False        # type: bool   # 是否向openart发送指令标志位
        self.if_lost_object = False       # type: bool   # 是否丢失目标物体标志位
        self.if_finish_servo = False      # 是否完成视觉伺服控制标志位
        self.if_orbit_ready = False       # type: bool   # 是否获取目标距离标志位
        self.if_finish_orbit = False      # type: bool   # 是否完成环绕控制标志位
        # ================= 视觉伺服矫正相关变量 =================
        # 单应性矩阵（由cv2.findHomography求得，作用是将像素坐标转换为实际物理坐标，考虑了摄像头的内参和外参）
        self.correct_dist = 10.51    # 经验修正值（物体在推杆正前方的值）
        self.H_matrix = [[ 1.93284562e+00, -1.87582067e-04, -1.42042018e+02],
                        [-4.39372726e-16, -1.31007314e+00,  1.96628024e+02],
                        [-1.32799359e-17,  6.57475145e-02,  1.00000000e+00]]
                        
        # apriltag码矫正相关变量--------------------------------------------------------
        apr_pos_L = self.flash_sys.find_value("apr_pos_L")
        apr_pos_R = self.flash_sys.find_value("apr_pos_R")
        apr_pos_D = self.flash_sys.find_value("apr_pos_D")
        apr_pos_U = self.flash_sys.find_value("apr_pos_U")
        self.apriltage_postion = {'U':apr_pos_U,'D':apr_pos_D,'R':apr_pos_R,'L':apr_pos_L}  # type: dict   # apriltag码在世界坐标系下的坐标
        self.apriltag_threshold_x = self.flash_sys.find_value("apriltag_threshold_x")
        self.apriltag_threshold_y = self.flash_sys.find_value("apriltag_threshold_y")
        self.calibrate_times = 0
        self.calibrate_waiting_time = self.flash_sys.find_value("calibrate_waiting_time")
        self.if_ready_calibrate = False
        self.if_finish_calibrate = False
        self.if_gain_calibrate_angle = False
        # 边线矫正时小车位置
        self.car_position = 'L'  # 'L', 'R', 'U', 'D'分别代表小车在左边线、右边线、上边线、下边线
        # 延时计数器
        self.if_waiting = True
        self.counter = 0       # type: int     # 延时计数器
        self.calibrate_buffer = []     # type: list    # 目标横或纵坐标缓冲区
        self.lost_path = []    # type: list    # 矫正时丢失需要行进的路径
        gc.collect()
        
    # 重置视觉伺服角度
    def reset_servo_angle(self):
        self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / PI

    # 重置环绕角度
    def reset_orbit_angle(self):
        self.orbit_turn_angle = self.my_car.now_yaw * 180.0 / PI

    # 重置环绕控制标志位
    def reset_orbit(self):
        self.if_orbit_ready = False
        self.if_finish_orbit = False

    # 重置小车上一帧记录的坐标
    def reset_last_car_pos(self):
        self.last_car_x = self.my_car.x_current
        self.last_car_y = self.my_car.y_current

    # 用单应性矩阵将像素坐标转换为实际物理坐标（单位：cm）
    def pixel_to_real_world(self, u, v, object_kind = None):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :param object_kind: 物体种类
        :return: 真实的物理坐标 (X_w, Y_w)
        """
        object_H = 0.0  # 默认值，防止 current_servo_object 为空或匹配不到时出现未赋值报错
        if self.my_state.state == CALIBRATE:
            object_H = 0.0
        else:
            if not object_kind:
                object_kind = self.current_servo_object
            if object_kind == 'T': 
                object_H = 2.5
            elif object_kind in ['S', 'E']:
                object_H = 5.0
            elif object_kind in ['W', 'B']:
                object_H = 3.5

        H_matrix = self.H_matrix

        K = (23.5 - object_H) / 23.5
        # 计算缩放因子
        w_prime = H_matrix[2][0] * u + H_matrix[2][1] * v + H_matrix[2][2]
        # 计算真实的物理坐标
        X_w = (H_matrix[0][0] * u + H_matrix[0][1] * v + H_matrix[0][2]) / w_prime * K
        Y_w = (H_matrix[1][0] * u + H_matrix[1][1] * v + H_matrix[1][2]) / w_prime * K

        return X_w, Y_w

    # 推测目标点位并进行视觉伺服控制
    def predict_point(self, x, y,limit_y = None):
        car_radius = 9.0
        raw_x, raw_y = self.pixel_to_real_world(x, y)
        if limit_y:
            if raw_y>limit_y: return []
        raw_y += car_radius
        relative_angle = -math.atan2(-raw_x, raw_y)
        actual_angle = self.my_car.now_yaw + relative_angle
        if actual_angle > PI:
            actual_angle -= 2 * PI
        elif actual_angle < -PI:
            actual_angle += 2 * PI
        actual_dist = math.sqrt(raw_x ** 2 + raw_y ** 2)
        absolute_x = actual_dist * math.sin(actual_angle) + self.my_car.x_current
        absolute_y = actual_dist * math.cos(actual_angle) + self.my_car.y_current
        return [absolute_x, absolute_y]
    
    # 动态调整视觉伺服pid参数
    def adjust_pid_by_dist(self, dist):
        # 距离越近，Kp 越小，防止超调；
        scale = max(0.8, min(1.0, dist / 10.0)) # 10cm外全速，近处最少降到90%
        self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_normal_x * scale
        self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_normal_y * scale

    def if_in_rect(self,x,y):
        rect_x_min = self.my_plan.plan_data.center_rect[0][0] - 5
        rect_x_max = self.my_plan.plan_data.center_rect[3][0] + 5
        rect_y_min = self.my_plan.plan_data.center_rect[0][1] - 5
        rect_y_max = self.my_plan.plan_data.center_rect[3][1] + 5
        if x < rect_x_min or x > rect_x_max or\
            y < rect_y_min or y > rect_y_max:
            return False
        return True
    
    def calc_object_global_pos(self, pixel_x, pixel_y, object_kind=None):
        # 像素点 -> 车体坐标系下真实坐标
        if object_kind:
            self.current_servo_object = object_kind
        rel_x, rel_y = self.pixel_to_real_world(pixel_x, pixel_y, object_kind)
        rel_y += 5
        # 车体坐标系下，x 为车右侧，y 为车前方
        dist = math.sqrt(rel_x ** 2 + rel_y ** 2)
        now_yaw = self.my_car.now_yaw * 180.0 / PI
        rel_yaw = math.atan2(rel_x, rel_y) * 180.0 / PI
        actual_yaw = now_yaw + rel_yaw
        actual_yaw = (actual_yaw + 180.0) % 360.0 - 180.0
        abs_x = dist * math.sin(actual_yaw * PI / 180.0)
        abs_y = dist * math.cos(actual_yaw * PI / 180.0)
        return [
            self.my_car.x_current + abs_x,
            self.my_car.y_current + abs_y
        ]
    # 物体像素点坐标解算函数
    def calculate_dist(self, x: int, y: int):
        # 将像素点坐标换算为相对坐标系下x和y方向上的实际偏移量
        self.relative_raw_x, self.relative_raw_y = self.pixel_to_real_world(x, y)
        self.relative_raw_y = self.relative_raw_y - self.final_dist_y - self.correct_dist
        # 根据小车记录的上一次坐标点进行矫正，避免因为小车移动导致的解算误差
        car_dist = math.sqrt((self.my_car.x_current - self.last_car_x) ** 2 + (self.my_car.y_current - self.last_car_y) ** 2)
        car_yaw = -math.atan2(-(self.my_car.x_current - self.last_car_x), (self.my_car.y_current - self.last_car_y)) * 180.0 / PI
        relative_yaw = car_yaw * PI / 180.0 - self.my_car.now_yaw
        # 限幅
        if relative_yaw > PI:
            relative_yaw -= 2 * PI
        elif relative_yaw < -PI:
            relative_yaw += 2 * PI
        self.relative_actual_x = self.relative_raw_x - (car_dist * math.sin(relative_yaw))
        self.relative_actual_y = self.relative_raw_y - (car_dist * math.cos(relative_yaw))
        self.actual_dist = math.sqrt(self.relative_actual_x ** 2 + self.relative_actual_y ** 2)
        # 计算物体相对于小车的绝对偏差
        now_yaw = self.my_car.now_yaw * 180 / PI
        rel_yaw = -math.atan2(-self.relative_actual_x, self.relative_actual_y) * 180.0 / PI
        actual_yaw = now_yaw + rel_yaw
        if actual_yaw > 180.0:
            actual_yaw -= 360.0
        elif actual_yaw < -180.0:
            actual_yaw += 360.0
        self.absolute_actual_x = self.actual_dist * math.sin(actual_yaw * PI / 180.0)
        self.absolute_actual_y = self.actual_dist * math.cos(actual_yaw * PI / 180.0)
        self.real_servo_point = [self.my_car.x_current + self.absolute_actual_x, self.my_car.y_current + self.absolute_actual_y]
        # 测试打印
        # self.my_uart3.write(f"{self.relative_raw_x},{self.relative_raw_y}\r\n")

    # 视觉伺服控制函数
    def visual_servo_control(self):
        if self.if_finish_servo == True:
            return # 已经完成视觉伺服控制，直接返回
        
        # 1. 尝试接收新一帧数据
        self.target_point = self.my_art_protocol.coordinate_receive()

        # 判断红色沙包是否在矩形框内
        if self.target_point:
            if self.current_servo_object == 'S':
                actual_point = self.predict_point(self.target_point[0], self.target_point[1])
                if_red_valid = self.if_in_rect(actual_point[0], actual_point[1])
            else:
                if_red_valid = True

        # 2. 判断是否收到有效的新视觉帧
        if self.target_point and chr(self.target_point[2]) == self.current_servo_object and if_red_valid:
            self.calculate_dist(self.target_point[0], self.target_point[1])

            # 突变检测：与上一帧伺服点位比较，防止噪点/干扰导致的振荡
            MAX_POINT_CHANGE = 12.0  # 最大坐标变化阈值（单位：cm）
            if self.last_real_servo_point is not None:
                dx = abs(self.real_servo_point[0] - self.last_real_servo_point[0])
                dy = abs(self.real_servo_point[1] - self.last_real_servo_point[1])

                self.reset_last_car_pos()

                if (dx > MAX_POINT_CHANGE or dy > MAX_POINT_CHANGE):
                    if self.relative_raw_y < self.last_relative_raw_y:
                        # 帧有效，更新记录
                        self.last_relative_raw_y = self.relative_raw_y
                        self.last_real_servo_point = self.real_servo_point.copy()
                        self.servo_lost_count = 0
                    else:
                        # 变化过大，丢弃本帧，还原为上一帧有效坐标
                        self.real_servo_point = self.last_real_servo_point.copy()
                        self.servo_lost_count += 1
                else:
                    # 帧有效，更新记录
                    self.last_relative_raw_y = self.relative_raw_y
                    self.last_real_servo_point = self.real_servo_point.copy()
                    self.servo_lost_count = 0
            else:
                # 首帧，直接接受
                self.last_relative_raw_y = self.relative_raw_y
                self.last_real_servo_point = self.real_servo_point.copy()
                self.last_car_x = self.my_car.x_current
                self.last_car_y = self.my_car.y_current
                self.servo_lost_count = 0
        else:
            self.servo_lost_count += 1

        if self.servo_lost_count >= 150:
            self.target_rel_speed = 0.0
            self.target_rel_yaw = 0.0
            self.if_lost_object = True
            self.servo_lost_count = 0
            return # 彻底丢失，跳出伺服逻辑
            
        # 用预测的点位，依赖惯导，进行pid控制
        now_error_x = self.real_servo_point[0] - self.my_car.x_current
        now_error_y = self.real_servo_point[1] - self.my_car.y_current
        dist = math.sqrt(now_error_x ** 2 + now_error_y ** 2)
        self.adjust_pid_by_dist(dist)
        # ================= 高频控制解耦 =================
        # if self.servo_lost_count <= 80:
        if self.servo_lost_count <= 80:
            self.servo_pid.model_compute_pid(now_error_x, now_error_y)
            self.target_rel_speed_x = self.servo_pid.pwm_output_x
            self.target_rel_speed_y = self.servo_pid.pwm_output_y
        else:
            self.last_real_servo_point = None
            # 连续丢失超过一定帧数后，降低小车速度
            self.target_rel_speed = 40.0
            return 

        # 4. 判断是否完成视觉伺服控制（离物体的距离和自身转角都需要达到目标）
        if abs(self.absolute_actual_x) <= self.finish_threshold_x and abs(self.absolute_actual_y) <= self.finish_threshold_y:
            self.target_rel_speed = 0.0
            self.target_rel_yaw = 0.0
            self.last_real_servo_point = None  # 重置上一帧伺服点位
            # 切换回正常的视觉伺服pid参数
            self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_normal_x
            self.servo_pid.servo_kd_x = self.servo_pid.servo_kd_normal_x
            self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_normal_y
            self.servo_pid.servo_kd_y = self.servo_pid.servo_kd_normal_y
            self.my_order_manager.finish()
            self.if_finish_servo = True
        else:
            # 原有的滤波和速度限制逻辑保持不变
            self.target_rel_speed_x = self.sin_servo_fil.filtering(self.target_rel_speed_x)
            self.target_rel_speed_y = self.cos_servo_fil.filtering(self.target_rel_speed_y)                                            
            self.target_rel_speed = math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2)
            # 计算目标角度，单位：度（注意避免除以0）
            self.target_rel_yaw = -math.atan2(-self.target_rel_speed_x, self.target_rel_speed_y) * 180.0 / PI
            if self.target_rel_yaw > 180.0:
                self.target_rel_yaw -= 360.0
            elif self.target_rel_yaw < -180.0:
                self.target_rel_yaw += 360.0  
            if self.target_rel_yaw > 70.0 or self.target_rel_yaw < -70.0:
                self.target_rel_speed = self.target_rel_speed * 0.8
            self.target_rel_speed = max(self.min_rel_speed, min(self.target_rel_speed, self.max_rel_speed))

    # 环绕控制函数，传入环绕物体旋转的目标世界坐标系角度（单位：度）（范围：-180到180）
    def orbit_control(self, target_angle: float, direct = None):
        if self.if_orbit_ready == False:
            # 保持静止
            self.orbit_speed = 0.0
            self.orbit_radius = self.object_radius
            self.record_angle = self.my_car.now_yaw * 180 / PI
            self.target_angle = target_angle
            # 限制目标角度在-180到180度之间
            if self.target_angle > 180.0:
                self.target_angle -= 360.0
            elif self.target_angle < -180.0:
                self.target_angle += 360.0
                
            # 计算需要旋转的相对角度来确定方向
            diff_angle = self.target_angle - self.record_angle
            if diff_angle > 180.0:
                diff_angle -= 360.0
            elif diff_angle < -180.0:
                diff_angle += 360.0
            
            self.reset_orbit_angle()
            # 确定旋转方向（顺时针还是逆时针）
            if direct is not None:
                self.direct = direct
            elif diff_angle >= 0.0:
                self.direct = 'CW'
            else:
                self.direct = 'CCW'
            self.current_dis = 0.0

             # 计算总的环绕角度（考虑选择的环绕方向，CW为顺时针，CCW为逆时针）
            natural_cw = (diff_angle >= 0.0)
            actual_cw = (self.direct == 'CW')
            self.total_orbit_angle = abs(diff_angle) if natural_cw == actual_cw else 360.0 - abs(diff_angle)

            # ====== 新增：记录下当前的理想旋转圆心坐标 ======
            # 刚开始环绕时，record_angle 为车头直面圆心的角度，由此推导世界坐标系下的圆心坐标
            self.orbit_center_x = self.my_car.x_current + self.orbit_radius * math.sin(self.record_angle * PI / 180.0)
            self.orbit_center_y = self.my_car.y_current + self.orbit_radius * math.cos(self.record_angle * PI / 180.0)
            self.if_orbit_ready = True
        else:
            if self.if_finish_orbit == True:
                return
            
            # ====== 修改：基于当前X/Y坐标的闭环位置控制 ======
            # 计算当前小车与圆心的实际向量
            dx = self.orbit_center_x - self.my_car.x_current
            dy = self.orbit_center_y - self.my_car.y_current
            actual_r = math.sqrt(dx**2 + dy**2)
            
            # 计算当前处于圆上的相位角 (从小车指向圆心)
            theta = -math.atan2(-dx, dy) * 180.0 / PI
            
            # 半径误差（大于0代表实际比指定半径近，需要向外扩）
            err_r = self.orbit_radius - actual_r
            
            # 向心/离心纠正比例 (将厘米级的偏离对应成航向角偏置)
            kr = 2.0
            
            if self.direct == 'CW':
                # 顺时针切线为 theta - 90。若太近(err_r>0)，需向外偏，减小转角
                self.orbit_yaw = theta - 90.0 - kr * err_r
            elif self.direct == 'CCW':
                # 逆时针切线为 theta + 90。若太近(err_r>0)，需向外偏，增加转角
                self.orbit_yaw = theta + 90.0 + kr * err_r
                
            self.orbit_yaw = (self.orbit_yaw + 180.0) % 360.0 - 180.0
            
            # ====== 新增：实时闭环车体姿态角 ======
            # theta 是从圆心指向小车的角度，小车要面向圆心，所以车头朝向应为 theta + 180 度
            self.orbit_turn_angle = theta
            self.orbit_turn_angle = (self.orbit_turn_angle + 180.0) % 360.0 - 180.0
            
            # 更新当前小车的速度（保留原有逻辑判断）
            diff = abs(self.target_angle - self.my_car.now_yaw * 180 / PI)
            if diff > 180.0:
                diff = 360.0 - diff

            # 环绕速度规划：对称梯形速度曲线 —— 启动时线性加速，结束时线性减速
            accel_range = min(15.0, self.total_orbit_angle / 2.0)   # 加速区间（度）
            decel_range = min(40.0, self.total_orbit_angle / 2.0)   # 减速区间（度）
            traveled = max(0.0, self.total_orbit_angle - diff)       # 已走过的角度
            if traveled < accel_range:
                # 启动阶段：三次 ease-out，起步快、过渡平滑
                t = traveled / accel_range              # 0.0 → 1.0
                ease = 1.0 - (1.0 - t) ** 3     # 三次 ease-out
                self.orbit_speed = self.orbit_v_min + (self.orbit_v_max - self.orbit_v_min) * ease
            elif diff < decel_range:
                t = diff / decel_range                  # 1.0 → 0.0
                ease = t * t * (3.0 - 2.0 * t)          # smoothstep：两端导数=0，无顿挫
                self.orbit_speed = self.orbit_v_min + (self.orbit_v_max - self.orbit_v_min) * ease
            else:
                # 匀速阶段：保持最大速度
                self.orbit_speed = self.orbit_v_max

            # 速度限幅
            self.orbit_speed = max(self.orbit_v_min, min(self.orbit_speed, self.orbit_v_max))

            # 判断是否完成环绕
            if diff <= 1.5:	
                self.orbit_speed = 0.0
                self.orbit_turn_angle = self.my_car.now_yaw * 180 / PI
                self.if_finish_orbit = True

    # 用于准备视觉伺服和环绕
    def ready_servo_and_orbit(self, object = None, state = 'servo', point = []):
        # 用于准备视觉伺服和环绕
        self.if_finish_servo = False
        self.if_lost_object = False
        self.servo_lost_count = 0
        self.target_rel_speed = 0.0
        self.target_rel_speed_x = 0.0
        self.target_rel_speed_y = 0.0
        self.target_rel_yaw = 0.0
        self.reset_last_car_pos()
        # 选择正常的视觉伺服pid参数
        self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_normal_x
        self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_normal_y
        self.servo_pid.servo_kd_x = self.servo_pid.servo_kd_normal_x
        self.servo_pid.servo_kd_y = self.servo_pid.servo_kd_normal_y
        self.record_angle = self.my_car.now_yaw  # 保持弧度制供 judge_next_turn 默认使用
        current_yaw_deg = self.record_angle * 180.0 / PI
        if current_yaw_deg > -45.0 and current_yaw_deg <= 45.0: current_turn_deg = 0.0
        elif current_yaw_deg > 45.0 and current_yaw_deg <= 135.0:current_turn_deg = 90.0
        elif current_yaw_deg > 135.0 or current_yaw_deg <= -135.0:current_turn_deg = 180.0
        elif current_yaw_deg > -135.0 and current_yaw_deg <= -45.0:current_turn_deg = -90.0

        # 此时将视觉伺服速度将为0
        self.target_rel_speed = 0.0
        
        # 控制小车面向物体进行视觉伺服控制
        if object:
            self.current_servo_object = object

        # 根据物品种类选择伺服距离、环绕半径和搬运速度
        if self.current_servo_object == 'T':
            self.my_plan.error_x = self.my_plan.error_x_T
            self.final_dist_y = self.servo_pid.target_y_T
            if  current_turn_deg == -90:self.object_radius = self.radius_T_R
            elif current_turn_deg == 90:self.object_radius = self.radius_T_L
            elif current_turn_deg == 180:self.object_radius = self.radius_T_U
            else:self.object_radius = self.radius_T_D
            self.orbit_angle = self.angle_T
            self.my_plan.move_v_max = self.my_plan.move_v_max_T
        elif self.current_servo_object in ['S', 'E']:
            self.my_plan.error_x = self.my_plan.error_x_S
            self.final_dist_y = self.servo_pid.target_y_S
            if  current_turn_deg == -90:self.object_radius = self.radius_S_R
            elif current_turn_deg == 90:self.object_radius = self.radius_S_L
            elif current_turn_deg == 180:self.object_radius = self.radius_S_U
            else:self.object_radius = self.radius_S_D
            self.orbit_angle = self.angle_S
            self.my_plan.move_v_max = self.my_plan.move_v_max_S
        elif self.current_servo_object in ['B', 'W']:
            self.my_plan.error_x = self.my_plan.error_x_B
            self.final_dist_y = self.servo_pid.target_y_B
            if  current_turn_deg == -90:self.object_radius = self.radius_B_R
            elif current_turn_deg == 90:self.object_radius = self.radius_B_L
            elif current_turn_deg == 180:self.object_radius = self.radius_B_U
            else:self.object_radius = self.radius_B_D
            self.orbit_angle = self.angle_B
            self.my_plan.move_v_max = self.my_plan.move_v_max_B
        
        if state == 'servo':
            pass
        # 微调模式下伺服距离减少
        else:
            self.final_dist_y *= 0.8
            self.final_dist_x *= 0.8

        if point:
            self.calculate_dist(point[0], point[1])
            self.last_relative_raw_y = self.relative_raw_y
            self.last_real_servo_point = None
    
    def reset_calibrate(self):
        self.if_ready_calibrate =False
        self.if_finish_calibrate =False
        self.if_gain_calibrate_angle = False    
        self.if_waiting = True
        self.if_lost_object = False
        self.reset_last_car_pos()
        self.calibrate_buffer = []
        self.my_plan.reset_navigate()
        self.counter = 0

    # apriltag辅助校准校准控制函数
    def apriltag_calibrate_control(self):
        if self.if_finish_calibrate == True:
            return # 已经完成视觉伺服控制，直接返回  
        if self.if_ready_calibrate == False:
            if self.if_waiting:
                if self.counter >= self.calibrate_waiting_time:
                    self.counter = 0
                    self.if_waiting = False
                else:
                    self.counter += 1
            else:
                self.my_plan.navigate(self.calibrate_buffer[0],self.calibrate_buffer[1])
                if self.my_plan.if_finish_navigate == True:
                    self.counter += 1
                    # 延时200ms等待小车稳定
                    if self.counter >= 20:
                        # 重置计数器
                        self.counter = 0
                        self.my_plan.reset_navigate()
                        self.if_ready_calibrate = True
                        # 伺服apriltag时固定目标点坐标（单位：像素），并且固定目标转角为0（即小车面向apriltag）
                        self.final_dist_y = self.servo_pid.target_y_A
                        # 将视觉伺服pid参数切换为矫正模式的参数
                        self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_calibrate_x
                        self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_calibrate_y
                        self.servo_pid.servo_kd_x = self.servo_pid.servo_kd_calibrate_x
                        self.servo_pid.servo_kd_y = self.servo_pid.servo_kd_calibrate_y
                        self.calibrate_times = 0
                        # 清空目标角度缓冲区
                        self.angle_buffer.clear()
                        self.target_rel_turn_angle = self.my_plan.turn_angle_target
                        # 更新小车相对于apriltag的位置
                        if self.car_position == 'L':
                            now_yaw = self.my_car.now_yaw * 180.0 / PI
                            if now_yaw >= -90.0 and now_yaw < 90.0:
                                self.rel_pos_to_apriltag = 'left'
                            else:
                                self.rel_pos_to_apriltag = 'right'
                        elif self.car_position == 'R':
                            now_yaw = self.my_car.now_yaw * 180.0 / PI
                            if now_yaw >= -90.0 and now_yaw < 90.0:
                                self.rel_pos_to_apriltag = 'right'
                            else:
                                self.rel_pos_to_apriltag = 'left'
                        elif self.car_position == 'U':
                            now_yaw = self.my_car.now_yaw * 180.0 / PI
                            if now_yaw >= 0.0 and now_yaw < 180.0:
                                self.rel_pos_to_apriltag = 'left'
                            else:
                                self.rel_pos_to_apriltag = 'right'
                        self.reset_last_car_pos()
                        self.my_order_manager.mode_apriltag()
        else:
            target_point = self.my_art_protocol.apriltag_receive()
            if target_point:
                self.servo_lost_count = 0
                # 留下外部测试接口
                self.angle_temp = target_point[2]
                
                # 判断是否完成视觉伺服控制
                diff = abs(self.target_rel_turn_angle - self.my_car.now_yaw * 180.0 / PI)
                if diff > 180.0:
                    diff = 360.0 - diff

                # print(f"{abs(self.absolute_actual_x)}, {abs(self.absolute_actual_y)}, {diff}, {self.calibrate_times}\r\n")
                
                if self.if_gain_calibrate_angle and (((abs(self.absolute_actual_x) <= self.apriltag_threshold_x and abs(self.absolute_actual_y) <= self.apriltag_threshold_y) and diff <= 1.0 and self.calibrate_times != 1) or len(self.angle_buffer) >= 15):
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.calibrate_times += 1
                    # 完成两次矫正才算结束
                    if self.calibrate_times >= 2:
                        self.calibrate_times = 0
                        self.counter = 0
                        # 里程计和姿态角硬复位
                        # print(f"final_angle: {sum(self.angle_buffer[2:]) / len(self.angle_buffer[2:])}")
                        final_angle = sum(self.angle_buffer[10:]) / len(self.angle_buffer[10:])
                            
                        diff = abs(final_angle - self.my_car.now_yaw * 180.0 / PI)
                        if diff > 180.0:
                            diff = 360.0 - diff

                        ANGLE_CALIBRATE_THRESHOLD = 2.0
                        # 当角度与实际角度差别过大时才矫正
                        if diff > ANGLE_CALIBRATE_THRESHOLD:
                            self.pose_data.reset_yaw(final_angle)
                            self.my_car.now_yaw = -self.pose_data.now_yaw * PI / 180.0

                        self.angle_buffer.clear()
                        # 更新小车里程计坐标
                        RELATIVE_DIST = 17.5
                        if self.car_position == 'L':
                            # self.my_car.x_current = self.apriltage_postion['L'][0]
                            apriltag_y = self.apriltage_postion['L'][1]
                            if self.rel_pos_to_apriltag == 'left':
                                self.my_car.y_current = apriltag_y - RELATIVE_DIST
                            elif self.rel_pos_to_apriltag == 'right':
                                self.my_car.y_current = apriltag_y + RELATIVE_DIST
                        elif self.car_position == 'R':
                            # self.my_car.x_current = self.apriltage_postion['R'][0]
                            apriltag_y = self.apriltage_postion['R'][1]
                            if self.rel_pos_to_apriltag == 'left':
                                self.my_car.y_current = apriltag_y + RELATIVE_DIST
                            elif self.rel_pos_to_apriltag == 'right':
                                self.my_car.y_current = apriltag_y - RELATIVE_DIST
                        elif self.car_position == 'U':
                            # self.my_car.y_current = self.apriltage_postion['U'][1]
                            apriltag_x = self.apriltage_postion['U'][0]
                            if self.rel_pos_to_apriltag == 'left':
                                self.my_car.x_current = apriltag_x - RELATIVE_DIST
                            elif self.rel_pos_to_apriltag == 'right':
                                self.my_car.x_current = apriltag_x + RELATIVE_DIST
                        # 在切换模式前保持当前转角
                        self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / PI
                        self.my_order_manager.finish()
                        self.if_finish_calibrate = True
                else:
                    self.servo_lost_count = 0

                    self.calculate_dist(target_point[0], target_point[1])

                    # print(f"{target_point}, {self.real_servo_point}\r\n")

                    self.reset_last_car_pos()
                        
                    # 用预测的点位，依赖惯导，进行pid控制
                    now_error_x = self.real_servo_point[0] - self.my_car.x_current
                    now_error_y = self.real_servo_point[1] - self.my_car.y_current
                    dist = math.sqrt(now_error_x ** 2 + now_error_y ** 2)
                    
                    # ================= 高频控制解耦 =================
                    # if self.servo_lost_count <= 80:
                    if self.servo_lost_count <= 80:
                        self.servo_pid.model_compute_pid(now_error_x, now_error_y)
                        self.target_rel_speed_x = self.servo_pid.pwm_output_x
                        self.target_rel_speed_y = self.servo_pid.pwm_output_y
                    else:
                        # 连续丢失超过一定帧数后，降低小车速度
                        self.target_rel_speed = 0.0
                        return 
                    
                    # 计算综合目标速度和航向角
                    # 滤波
                    self.target_rel_speed_x = self.sin_servo_fil.filtering(self.target_rel_speed_x)
                    self.target_rel_speed_y = self.cos_servo_fil.filtering(self.target_rel_speed_y)                                            
                    # 计算目标角度，单位：度（注意避免除以0）
                    self.target_rel_yaw = -math.atan2(-self.target_rel_speed_x, self.target_rel_speed_y) * 180.0 / PI
                    if self.target_rel_yaw > 180.0:
                        self.target_rel_yaw -= 360.0
                    elif self.target_rel_yaw < -180.0:
                        self.target_rel_yaw += 360.0  

                    if self.calibrate_times == 1:
                        self.target_rel_speed = 0
                    else:
                        # 计算伺服速度
                        self.target_rel_speed = int(math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2))
                        # 当横移角度过大时，速度折半
                        if self.target_rel_yaw > 45.0 or self.target_rel_yaw < -45.0:
                            self.target_rel_speed = int(self.target_rel_speed * 0.8)
                        self.target_rel_speed = max(self.min_rel_speed, min(self.target_rel_speed, self.max_rel_speed))
            
                if self.if_gain_calibrate_angle == False or self.calibrate_times == 1:
                    if self.calibrate_times == 1:
                        # 计算目标转角(多次测量取平均值)
                        if self.car_position == 'L':
                            if self.rel_pos_to_apriltag == 'left':
                                self.angle_buffer.append(0.0 + target_point[2])
                            elif self.rel_pos_to_apriltag == 'right':
                                angle = 180.0 - target_point[2]
                                if angle > 180.0:
                                    angle -= 360.0
                                self.angle_buffer.append(angle)
                        elif self.car_position == 'R':
                            if self.rel_pos_to_apriltag == 'left':
                                angle = 180.0 + target_point[2]
                                if angle > 180.0:
                                    angle -= 360.0
                                self.angle_buffer.append(angle)
                            elif self.rel_pos_to_apriltag == 'right':
                                self.angle_buffer.append(0.0 - target_point[2])
                        elif self.car_position == 'U':
                            if self.rel_pos_to_apriltag == 'left':
                                angle = 90.0 + target_point[2]
                                if angle > 180.0:
                                    angle -= 360.0
                                self.angle_buffer.append(angle)
                            elif self.rel_pos_to_apriltag == 'right':
                                angle = -90.0 - target_point[2]
                                if angle < -180.0:  
                                    angle += 360.0
                                self.angle_buffer.append(angle)
                    else:
                        """
                        now_yaw = self.my_car.now_yaw * 180.0 / PI
                        # 计算目标转角
                        if self.rel_pos_to_apriltag == 'left':
                            self.target_rel_turn_angle = now_yaw - target_point[2]
                        elif self.rel_pos_to_apriltag == 'right':
                            self.target_rel_turn_angle = now_yaw + target_point[2]
                        """
                        self.if_gain_calibrate_angle = True

            else:
                self.servo_lost_count += 1
                # 连续丢失150帧apriltag坐标后（在1.5s内不再收到物体坐标信息），认为apriltag丢失，停止小车运动
                if self.servo_lost_count >= 150:
                    if self.car_position == 'L':
                        if self.rel_pos_to_apriltag == 'left':
                            self.lost_path = [[self.my_car.x_current, self.my_car.y_current - 15.0],[self.my_car.x_current + 15.0, self.my_car.y_current - 15.0],
                                              [self.my_car.x_current + 15.0, self.my_car.y_current + 10.0]]
                        elif self.rel_pos_to_apriltag == 'right':
                            self.lost_path = [[self.my_car.x_current, self.my_car.y_current + 15.0],[self.my_car.x_current + 15.0, self.my_car.y_current + 15.0],
                                              [self.my_car.x_current + 15.0, self.my_car.y_current - 10.0]]
                    elif self.car_position == 'R':
                        if self.rel_pos_to_apriltag == 'left':
                            self.lost_path = [[self.my_car.x_current, self.my_car.y_current + 15.0],[self.my_car.x_current - 15.0, self.my_car.y_current + 15.0],
                                              [self.my_car.x_current - 15.0, self.my_car.y_current - 10.0]]
                        elif self.rel_pos_to_apriltag == 'right':
                            self.lost_path = [[self.my_car.x_current, self.my_car.y_current - 15.0],[self.my_car.x_current - 15.0, self.my_car.y_current - 15.0],
                                              [self.my_car.x_current + 15.0, self.my_car.y_current + 10.0]]
                    elif self.car_position == 'U':
                        if self.rel_pos_to_apriltag == 'left':
                            self.lost_path = [[self.my_car.x_current - 15.0, self.my_car.y_current],[self.my_car.x_current - 15.0, self.my_car.y_current - 15.0],
                                              [self.my_car.x_current + 10.0, self.my_car.y_current - 15.0]]
                        elif self.rel_pos_to_apriltag == 'right':
                            self.lost_path = [[self.my_car.x_current + 15.0, self.my_car.y_current],[self.my_car.x_current + 15.0, self.my_car.y_current - 15.0],
                                              [self.my_car.x_current - 10.0, self.my_car.y_current - 15.0]]
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.servo_lost_count = 0
                    self.if_lost_object = True