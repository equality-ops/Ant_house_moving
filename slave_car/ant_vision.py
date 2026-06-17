from micropython import const
import math
import gc

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
FOLLOW = const(10)          # 跟随状态

InField = const(-1)
OnLine = const(0)
OutLine = const(1)
# 多路复用器计数器
counter = 0

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
        self.final_dist = 0.0
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

        # ================= 视觉伺服矫正相关变量 =================
        # 单应性矩阵（由cv2.findHomography求得，作用是将像素坐标转换为实际物理坐标，考虑了摄像头的内参和外参）
        self.close_H_matrix =[[ 2.74130432e+00,4.32802220e-02,-2.29544169e+02],
                            [-1.38227049e-01,-2.38356178e+00,3.21314055e+02],
                            [-1.93211578e-03,9.33698324e-02,1.00000000e+00]]
                        
        self.far_H_matrix = [[ 2.74130432e+00,4.32802220e-02,-2.29544169e+02],
                            [-1.38227049e-01,-2.38356178e+00,3.21314055e+02],
                            [-1.93211578e-03,9.33698324e-02,1.00000000e+00]]
        # 解算后的物体与小车的相对位置偏差
        self.relative_raw_x = 0.0
        self.relative_raw_y = 0.0
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
        self.radius_T = self.flash_sys.find_value("radius_T")   # type: float   # 网球半径
        self.radius_S = self.flash_sys.find_value("radius_S")   # type: float   # 沙袋半径
        self.radius_B = self.flash_sys.find_value("radius_B")   # type: float   # 玩具熊半径
        self.angle_T = self.flash_sys.find_value("angle_T")     # type: float   # 网球环绕角度
        self.angle_S = self.flash_sys.find_value("angle_S")     # type: float   # 沙袋环绕角度
        self.angle_B = self.flash_sys.find_value("angle_B")     # type: float   # 玩具熊环绕角度
        self.direct = 'CW'  # 'CW'为顺时针(Clockwise)，'CCW'为逆时针(Counter-Clockwise)
        self.car_radius = 13.0   # 小车推杆到中心的距离
        self.correct_dist = 5.90    # 经验修正值（物体在推杆正前方的值）
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
        # 临时用于测试的角度变量
        self.angle_temp = 0.0

        # 标志位
        self.if_send_order = False        # type: bool   # 是否向openart发送指令标志位
        self.if_lost_object = False       # type: bool   # 是否丢失目标物体标志位
        self.if_finish_servo = False      # 是否完成视觉伺服控制标志位
        self.if_orbit_ready = False       # type: bool   # 是否获取目标距离标志位
        self.if_finish_orbit = False      # type: bool   # 是否完成环绕控制标志位
        self.if_ready_calibrate = False       # type: bool  # 判断是否准备好进行校准标志位
        self.if_finish_calibrate = False       # type: bool  # 判断是否完成校准标志位

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

    # 用单应性矩阵将像素坐标转换为实际物理坐标（单位：cm）
    def pixel_to_real_world(self, u, v, sign: str):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :param sign: 远近标志
        :return: 真实的物理坐标 (X_w, Y_w)
        """
        object_H = 0.0  # 默认值，防止 current_servo_object 为空或匹配不到时出现未赋值报错
        if self.my_state == CALIBRATE:
            object_H = 0.0
        else:
            if self.current_servo_object in ['T']:
                object_H = 2.5
            elif self.current_servo_object in ['S', 'E']:
                object_H = 7.0
            elif self.current_servo_object in ['W', 'B']:
                object_H = 2.0

        # 根据物体远近选择单应性矩阵H
        if sign == 'close':
            H_matrix = self.close_H_matrix
        elif sign == 'far':
            H_matrix = self.far_H_matrix

        K = (22.2 - object_H) / 22.2
        # 计算缩放因子
        w_prime = H_matrix[2][0] * u + H_matrix[2][1] * v + H_matrix[2][2]
        # 计算真实的物理坐标
        X_w = (H_matrix[0][0] * u + H_matrix[0][1] * v + H_matrix[0][2]) / w_prime * K
        Y_w = (H_matrix[1][0] * u + H_matrix[1][1] * v + H_matrix[1][2]) / w_prime * K

        return X_w, Y_w

    # 动态调整视觉伺服pid参数
    def adjust_pid_by_dist(self, dist):
        # 距离越近，Kp 越小，防止超调；
        scale = max(0.6, min(1.0, dist / 8.0)) # 8cm外全速，近处最少降60%
        self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_normal_x * scale
        self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_normal_y * scale

    # 物体像素点坐标解算函数
    def calculate_dist(self, x: int, y: int, sign: str = 'far'):
        # 将像素点坐标换算为相对坐标系下x和y方向上的实际偏移量
        self.relative_raw_x, self.relative_raw_y = self.pixel_to_real_world(x, y, sign)
        self.relative_raw_y = self.relative_raw_y - self.correct_dist - self.final_dist
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

    def visual_servo_control(self):
        if self.if_finish_servo == True:
            return # 已经完成视觉伺服控制，直接返回
        # 1. 尝试接收新一帧数据
        self.target_point = self.my_art_protocol.coordinate_receive()
        
        # 2. 判断是否收到有效的新视觉帧
        if self.target_point and chr(self.target_point[2]) == self.current_servo_object:
            # old_servo_point = list(self.real_servo_point)  # 暂存上一次算出的绝对目标点
            self.calculate_dist(self.target_point[0], self.target_point[1], 'close')
            '''
            # 判断两帧世界坐标偏差，如果大跳变则认为是另外一个同类干扰物体
            jump_dist = math.sqrt((self.real_servo_point[0] - old_servo_point[0])**2 + (self.real_servo_point[1] - old_servo_point[1])**2)
            
            if old_servo_point != [0, 0] and jump_dist > 10.0:  # 10cm以上的跳变认为是其他物体
                self.real_servo_point = old_servo_point   # 判断为其他物体，还原回被锁定的旧目标坐标
                self.servo_lost_count += 1                # 视作掉帧或丢失
                
                # 彻底丢失保护
                if self.servo_lost_count >= 150:
                    self.target_rel_speed = 0.0
                    self.target_rel_yaw = 0.0
                    self.if_lost_object = True
                    self.servo_lost_count = 0
                    return
            else:
            '''
            # 当前物体验证通过，或是第一帧
            # 记录下小车当前的坐标点
            self.last_car_x = self.my_car.x_current
            self.last_car_y = self.my_car.y_current

            # 重置掉帧计数
            self.servo_lost_count = 0
        else:
            self.servo_lost_count += 1
            # 彻底丢失保护
            if self.servo_lost_count >= 150:
                self.target_rel_speed = 0.0
                self.target_rel_yaw = 0.0
                # 测试，先不重置丢失物体标志位
                # self.if_lost_object = True
                self.servo_lost_count = 0
                return # 彻底丢失，跳出伺服逻辑
            
        # 用预测的点位，依赖惯导，进行pid控制
        now_error_x = self.real_servo_point[0] - self.my_car.x_current
        now_error_y = self.real_servo_point[1] - self.my_car.y_current
        dist = math.sqrt(now_error_x ** 2 + now_error_y ** 2)
        self.adjust_pid_by_dist(dist)
        # ================= 高频控制解耦 =================
        if self.servo_lost_count <= 80:
            self.servo_pid.model_compute_pid(now_error_x, now_error_y)
            self.target_rel_speed_x = self.servo_pid.pwm_output_x
            self.target_rel_speed_y = self.servo_pid.pwm_output_y
        else:
            # 连续丢失超过一定帧数后，降低小车速度
            self.target_rel_speed = 50.0
            return 

        # 4. 判断是否完成视觉伺服控制
        if abs(self.absolute_actual_x) <= self.finish_threshold_x and abs(self.absolute_actual_y) <= self.finish_threshold_y:
            self.target_rel_speed = 0.0
            self.target_rel_yaw = 0.0
            # 选择正常伺服状态下的pid参数
            self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_normal_x
            self.servo_pid.servo_kd_x = self.servo_pid.servo_kd_normal_x
            self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_normal_y
            self.servo_pid.servo_kd_y = self.servo_pid.servo_kd_normal_y
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
            if self.target_rel_yaw > 60.0 or self.target_rel_yaw < -60.0:
                self.target_rel_speed = self.target_rel_speed * 0.7
            self.target_rel_speed = max(self.min_rel_speed, min(self.target_rel_speed, self.max_rel_speed))

    # 计算环绕中心坐标函数（传入物体中心像素点坐标）
    def calculate_orbit_center(self, x, y):
        raw_x, raw_y = self.pixel_to_real_world(x, y, 'close')
        # 3.0为物体平均半径
        raw_y = raw_y + self.car_radius - self.correct_dist + 3.0  # 将物体距离修正为从小车中心到物体的距离
        raw_yaw = -math.atan2(-raw_x, raw_y)
        real_yaw = (raw_yaw + self.my_car.now_yaw + PI) % (2 * PI) - PI
        actual_dist = math.sqrt(raw_x**2 + raw_y**2)
        self.orbit_center_x = self.my_car.x_current + actual_dist * math.sin(real_yaw)
        self.orbit_center_y = self.my_car.y_current + actual_dist * math.cos(real_yaw)

    
    # 环绕控制函数，传入环绕物体旋转的目标世界坐标系角度（单位：度）（范围：-180到180）
    def orbit_control(self, target_angle: float, direct = None):
        if self.if_orbit_ready == False:
            # 选择合适的里程计系数（无负压）
            self.my_car.alpha_x = 0.9093
            self.my_car.alpha_y = 0.936709
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
            kr = 2.5
            
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
            accel_range = min(10.0, self.total_orbit_angle / 2.0)   # 加速区间（度）
            decel_range = min(20.0, self.total_orbit_angle / 2.0)   # 减速区间（度）
            traveled = max(0.0, self.total_orbit_angle - diff)       # 已走过的角度

            if traveled < accel_range:
                # 启动阶段：线性从v_min加速到v_max
                self.orbit_speed = self.orbit_v_min + (self.orbit_v_max - self.orbit_v_min) * traveled / accel_range
            elif diff < decel_range:
                # 减速阶段：线性从v_max减速到v_min
                self.orbit_speed = self.orbit_v_max - (self.orbit_v_max - self.orbit_v_min) * (decel_range - diff) / decel_range
            else:
                # 匀速阶段：保持最大速度
                self.orbit_speed = self.orbit_v_max

            # 速度限幅
            self.orbit_speed = max(self.orbit_v_min, min(self.orbit_speed, self.orbit_v_max))

            # 判断是否完成环绕
            if diff <= 1.0:	
                self.orbit_speed = 0.0
                self.orbit_turn_angle = self.my_car.now_yaw * 180 / PI
                self.if_finish_orbit = True

    # apriltag辅助校准校准控制函数
    def apriltag_calibrate_control(self):
        """'L'代表左边线, 'R'代表右边线, 'U'代表上边线, 'D'代表下边线"""
        if self.if_ready_calibrate == False:
            # 准备阶段：调整小车位置和角度，面向apriltag
            if self.car_position == 'L':
                self.record_angle = -90.0
            elif self.car_position == 'U':
                self.record_angle = 0.0
            elif self.car_position == 'R':
                self.record_angle = 90.0
            elif self.car_position == 'D':
                self.record_angle = 180.0
            
            self.my_plan.navigate(target_turn_angle = self.record_angle)
            
            if self.my_plan.if_finish_navigate == True:
                self.my_plan.reset_navigate()
                # 清空目标角度缓冲区
                self.angle_buffer.clear()
                # 清空目标坐标缓冲区
                self.point_buffer.clear()
                # 重置阶段标志
                self.if_ready_calibrate = True
                self.my_order_manager.mode_apriltag()
                # 测试，直接完成矫正跳到下一个物体
                self.if_finish_calibrate = True 
        else:
            if self.if_finish_calibrate == True:
                return # 已经完成校准，直接返回

            target_point = self.my_art_protocol.apriltag_receive()
            if target_point:
                # 重置掉帧计数
                self.servo_lost_count = 0
                # self.angle_temp = target_point[2]
                corrected_x, corrected_y = self.pixel_to_real_world(target_point[0], target_point[1], 'close')
                # 测试
                self.my_uart3.write(f"Corrected X: {corrected_x:.2f} cm, Corrected Y: {corrected_y:.2f} cm, angle: {target_point[2]:.2f}\r\n")
                
                self.point_buffer.append((corrected_x, corrected_y))
                self.angle_buffer.append(self.record_angle + target_point[2])
            else:       
                self.servo_lost_count += 1
                # 连续丢失150帧apriltag坐标后（在1.5s内不再收到物体坐标信息），认为apriltag丢失，停止小车运动
                if self.servo_lost_count >= 150:
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.servo_lost_count = 0
                    self.if_lost_object = True

            if len(self.point_buffer) >= 10 and len(self.angle_buffer) >= 10:
                # 取中值作为当前的目标坐标和目标角度，抵抗偶然的极大解算异常
                sorted_x = sorted([p[0] for p in self.point_buffer])
                sorted_y = sorted([p[1] for p in self.point_buffer])
                sorted_angle = sorted(self.angle_buffer[2:])
                
                avg_x = sum(sorted_x[1:-1]) / len(sorted_x[1:-1]) if len(sorted_x) > 2 else sorted_x[len(sorted_x)//2]
                avg_y = sum(sorted_y[1:-1]) / len(sorted_y[1:-1]) if len(sorted_y) > 2 else sorted_y[len(sorted_y)//2]
                avg_angle = sum(sorted_angle[1:-1]) / len(sorted_angle[1:-1]) if len(sorted_angle) > 2 else sorted_angle[len(sorted_angle)//2]
            
                relative_angle = -math.atan2(-avg_x, avg_y) * 180.0 / PI
                # 世界坐标系下的真实角度 = 车体坐标系下的目标角度 + 小车当前的角度
                real_angle = avg_angle + relative_angle
                real_angle = (90.0 - real_angle + 180.0) % 360.0 - 180.0
                real_dist = math.sqrt(avg_x ** 2 + avg_y ** 2)
                real_x = real_dist * math.cos(real_angle * PI / 180.0)
                real_y = real_dist * math.sin(real_angle * PI / 180.0)

                # 里程计和姿态角硬复位  
                # 测试不矫正
                self.pose_data.reset_yaw(avg_angle)
                
                if self.car_position == 'L':
                    self.my_car.x_current = self.assist_car_pos[0] - real_x
                    self.my_car.y_current = self.assist_car_pos[1] - real_y
                elif self.car_position == 'U':
                    self.my_car.x_current = self.assist_car_pos[0] - real_x
                    self.my_car.y_current = self.assist_car_pos[1] - real_y
                elif self.car_position == 'R':
                    self.my_car.x_current = self.assist_car_pos[0] - real_x
                    self.my_car.y_current = self.assist_car_pos[1] - real_y
                elif self.car_position == 'D':
                    self.my_car.x_current = self.assist_car_pos[0] - real_x
                    self.my_car.y_current = self.assist_car_pos[1] + real_y

                # 测试
                self.my_uart3.write(f"Real X: {self.my_car.x_current:.2f} cm, Real Y: {self.my_car.y_current:.2f} cm, angle:{avg_angle:.2f}\r\n")

                self.angle_buffer.clear()
                self.point_buffer.clear()
                # 重置速度和转角
                self.target_rel_speed = 0
                self.target_rel_yaw = 0.0
                self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / PI
                self.my_order_manager.finish()
                self.if_finish_calibrate = True 

    # 重置apriltag矫正相关变量
    def reset_apriltag_calibrate(self):
        self.if_ready_calibrate = False
        self.if_finish_calibrate = False
        self.servo_lost_count = 0
        self.point_buffer.clear()
        self.angle_buffer.clear()

    # 用于准备视觉伺服和环绕
    def ready_servo_and_orbit(self, target_point, state = 'servo'):
        # 选择合适的里程计系数
        self.my_car.alpha_x = 0.9093
        self.my_car.alpha_y = 0.936709
        # 选择正常的视觉伺服pid参数
        self.servo_pid.servo_kp_x = self.servo_pid.servo_kp_normal_x
        self.servo_pid.servo_kp_y = self.servo_pid.servo_kp_normal_y
        self.servo_pid.servo_kd_x = self.servo_pid.servo_kd_normal_x
        self.servo_pid.servo_kd_y = self.servo_pid.servo_kd_normal_y
        # 控制小车面向物体进行视觉伺服控制
        self.current_servo_object = chr(target_point[2])
        # 根据物品种类选择伺服距离、环绕半径和搬运速度
        if self.current_servo_object == 'T':
            self.my_plan.error_x = self.my_plan.error_x_T
            self.final_dist = self.servo_pid.target_y_T
            self.object_radius = self.radius_T
            self.orbit_angle = self.angle_T
            self.my_plan.move_v_max = self.my_plan.move_v_max_T
        elif self.current_servo_object == 'S' or self.current_servo_object == 'E':
            self.my_plan.error_x = self.my_plan.error_x_S
            self.final_dist = self.servo_pid.target_y_S
            self.object_radius = self.radius_S
            self.orbit_angle = self.angle_S
            self.my_plan.move_v_max = self.my_plan.move_v_max_S
        elif self.current_servo_object == 'B' or self.current_servo_object == 'W':
            self.my_plan.error_x = self.my_plan.error_x_B
            self.final_dist = self.servo_pid.target_y_B
            self.object_radius = self.radius_B
            self.orbit_angle = self.angle_B
            self.my_plan.move_v_max = self.my_plan.move_v_max_B

        if state == 'servo':
            pass
        # 微调模式下伺服距离减少
        else:
            self.final_dist *= 0.4

        # 第一帧图像预测伺服点位
        self.last_car_x = self.my_car.x_current
        self.last_car_y = self.my_car.y_current
        self.calculate_dist(target_point[0], target_point[1], 'far')

# 红外跟随控制类
# 利用主车尾部两盏红外灯的逆透视坐标 (x,y) 和实际物理间距，
# 解算 IR 板中心位姿并控制小车实时跟随。
class IRFollow:
    def __init__(self, flash_sys, car, uart3, my_IR_protocol):
        """
        flash_sys: 参数存储系统对象
        car:       CarPose 对象，用于获取小车位姿和调用 move_ctrl
        uart3:     调试串口对象
        my_IR_protocol: IR 协议对象，用于接收 IR 坐标数据
        """
        self.flash_sys = flash_sys
        self.my_car = car
        self.my_uart3 = uart3
        self.my_IR_protocol = my_IR_protocol

        # ========== IR 跟随 PD 参数（从 flash 读取）==========
        self.kp_y = flash_sys.find_value("ir_kp_y")     # 距离环 (Y轴) 比例系数
        self.kd_y = flash_sys.find_value("ir_kd_y")     # 距离环 (Y轴) 微分系数
        self.kp_x = flash_sys.find_value("ir_kp_x")     # 横向环 (X轴) 比例系数
        self.kd_x = flash_sys.find_value("ir_kd_x")     # 横向环 (X轴) 微分系数

        # 目标跟车距离 (cm)
        self.target_dist = flash_sys.find_value("ir_target_dist")   
        # 目标跟车横移距离
        self.target_offset_x = flash_sys.find_value("ir_target_offset_x")
        # 目标红外灯角度
        self.target_angle = flash_sys.find_value("ir_target_angle")
        # 摄像头偏离小车正前方的角度
        self.art_to_car_angle = flash_sys.find_value("ir_art_to_car_angle")  

        # ========== 逆透视校正参数 ==========
        # L_actual 与 L_measured 比值的有效范围
        self.scale_min = 0.5  # 默认 0.5
        self.scale_max = 1.8  # 默认 1.8
        # 校正系数低通滤波权重（平滑突变）
        self.scale_smooth = 0.7  # 默认 0.7

        # ========== 控制输出 ==========
        self.output_speed = 0.0       # 合速度 (cm/s), 对应 move_ctrl 的 move_speed_target
        self.output_angle = 0.0       # 运动方向角 (度), 对应 move_ctrl 的 move_angle_target
        self.output_turn = 0.0        # 自转角速度 (度), 对应 move_ctrl 的 turn_angle_target

        # ========== 状态变量 ==========
        # 上一帧误差（用于微分计算）
        self.last_err_x = 0.0
        self.last_err_y = 0.0
        self.last_err_a = 0.0
        # 上一帧的尺度校正因子（用于低通平滑）
        self.last_scale = 1.0
        # 逆透视校正后的 IR 板中心坐标
        self.cx_corrected = 0.0
        self.cy_corrected = 0.0
        # IR 板朝向角 (度)
        self.board_heading = 0.0
        # 当前帧的逆透视校正因子
        self.current_scale = 1.0

        # ========== 丢帧与有效性判定 ==========
        self.ir_lost_count = 0            # 连续无效帧计数
        self.ir_lost_threshold = 50       # 连续丢帧阈值（约 0.5s，10ms 周期下）
        self.if_ir_lost = False           # 是否处于 IR 丢失状态
        self.if_frame_valid = False       # 当前帧是否有效

        # ========== 速度限幅与死区 ==========
        self.max_speed = flash_sys.find_value("ir_max_speed")      # 默认 220.0
        self.min_speed = flash_sys.find_value("ir_min_speed")      # 默认 25.0
        self.dead_dist_y = flash_sys.find_value("ir_dead_dist_y")  # Y 轴死区, 默认 1.5
        self.dead_dist_x = flash_sys.find_value("ir_dead_dist_x")  # X 轴死区, 默认 1.0
        self.dead_angle = flash_sys.find_value("ir_dead_angle")   # 角度死区, 默认 2.0

        gc.collect()

    # ==================================================================
    # 核心接口：每收到一帧 IR 坐标时调用
    #   输入: ir_light = [x1, y1, x2, y2] 两盏红外灯的图像坐标
    #   L_actual: 两灯实际物理间距 (cm)
    # 返回: True 表示本帧有效且控制量已更新, False 表示本帧无效
    # ==================================================================
    def compute(self, ir_light) -> bool:
        x1 = ir_light[0]
        y1 = ir_light[1]
        x2 = ir_light[2]
        y2 = ir_light[3]

        # ---------- 1. 原始测量值 ----------
        self.cx_raw = (x1 + x2) / 2.0          # IR 板中心 X (横向偏移)
        self.cy_raw = (y1 + y2) / 2.0          # IR 板中心 Y (纵向距离)

        dx = x2 - x1                            # 灯间 X 差
        dy = y2 - y1                            # 灯间 Y 差
        L_measured = math.sqrt(dx * dx + dy * dy)  # 图像测得的灯间距

        # IR 板朝向角：两灯连线的角度
        self.board_heading_raw = -math.atan2(-dx, dy) * 180.0 / PI

        # self.my_uart3.write(f"Raw CX: {self.cx_raw:.2f}, Raw CY: {self.cy_raw:.2f}, L_measured: {L_measured:.2f}, Board Heading: {self.board_heading_raw:.2f}\r\n")

        # ---------- 2. 帧有效性判定 ----------
        if L_measured < 0.01:
            # 两灯重合或检测异常
            self._mark_invalid()
            return False
        
        L_actual = 9.4
        scale_raw = L_actual / L_measured  # 逆透视尺度校正因子

        # 尺度因子超出合理范围 → 帧无效
        if scale_raw < self.scale_min or scale_raw > self.scale_max:
            self._mark_invalid()
            return False

        # ---------- 3. 低通滤波平滑尺度因子 ----------
        self.current_scale = (self.scale_smooth * self.last_scale +
                              (1.0 - self.scale_smooth) * scale_raw)
        self.last_scale = self.current_scale

        # ---------- 4. 逆透视失真校正 ----------
        # 用尺度校正因子修正中心坐标（逆透视的线性误差在不同距离下会缩放）
        self.cx_corrected = self.cx_raw * self.current_scale
        self.cy_corrected = self.cy_raw * self.current_scale

        # 板朝向角同样受畸变影响——两灯连线的角度在校正前后一致
        # （因为两个坐标等比例缩放不改变连线方向），所以直接使用原始值
        self.board_heading = self.board_heading_raw

        # ---------- 5. 帧有效，重置丢帧计数 ----------
        self.if_frame_valid = True
        self.if_ir_lost = False
        self.ir_lost_count = 0

        # ---------- 6. PD 控制计算 ----------
        self._pd_control()

        return True

    # ==================================================================
    # 标记无效帧并处理丢帧逻辑
    # ==================================================================
    def _mark_invalid(self):
        self.if_frame_valid = False
        self.ir_lost_count += 1

        if self.ir_lost_count >= self.ir_lost_threshold:
            self.if_ir_lost = True
            # 丢失后减速停车，避免盲跑
            self.output_speed = 0.0
            self.output_angle = 0.0
            self.output_turn = 0.0
            # 重置误差历史，防止找回后微分项跳变
            self.last_err_x = 0.0
            self.last_err_y = 0.0
            self.last_err_a = 0.0
        else:
            # 短暂丢帧期间，保持上一帧输出（靠里程计惯性维持）
            pass

    # ==================================================================
    # PD 反馈控制
    # 三环解耦: Y轴距离环 + X轴横向环 + 角度环
    # ==================================================================
    def _pd_control(self):
        # ======== Y轴距离环 ========
        err_y = self.cy_corrected - self.target_dist    # +: 太远需追近
        d_err_y = err_y - self.last_err_y
        output_y = self.kp_y * err_y + self.kd_y * d_err_y

        # ======== X轴横向环 ========
        err_x = self.cx_corrected - self.target_offset_x  # +: 偏右需右移
        d_err_x = err_x - self.last_err_x
        output_x = self.kp_x * err_x + self.kd_x * d_err_x

        # ======== 角度环：计算绝对偏航角目标 ========
        # 板相对小车的偏角误差
        err_a = self.target_angle - self.board_heading
        # 归一化到 [-180, 180]
        if err_a > 180.0:
            err_a -= 360.0
        elif err_a < -180.0:
            err_a += 360.0

        # 小车当前偏航角 (度)
        car_yaw_deg = self.my_car.now_yaw * 180.0 / PI

        # 板子相对朝向
        board_abs_heading = self.board_heading - self.target_angle

        # 世界坐标系下的绝对朝向 = 小车当前朝向 + 板子相对朝向
        # 测试不该变角度
        # self.output_turn = car_yaw_deg + board_abs_heading

        # 归一化到 [-180, 180]，供 angle_pid 使用
        # 测试不该变角度
        # self.output_turn = (self.output_turn + 180.0) % 360.0 - 180.0

        # ======== 合成全向运动指令 ========
        self.output_speed = math.sqrt(output_x * output_x + output_y * output_y)

        if self.output_speed > 0.01:
            self.output_angle = -math.atan2(-output_x, output_y) * 180.0 / PI
            # 补偿摄像头安装偏角
            self.output_angle = (self.output_angle + self.art_to_car_angle + 180.0) % 360.0 - 180.0
        else:
            self.output_angle = 0.0

        # ======== 死区：距离 + 横向 + 角度 都满足才停车 ========
        # 测试：此时不控制角度取消角度的死区控制
        if (abs(err_y) < self.dead_dist_y and
            abs(err_x) < self.dead_dist_x):
            # abs(err_a) < self.dead_angle):
            self.output_speed = 0.0
            self.output_angle = 0.0
            # output_turn 保持当前值不归零，让 angle_pid 自行维持姿态

        # ======== 速度限幅 ========
        self.output_speed = max(self.min_speed, min(self.output_speed, self.max_speed))

        # ======== 保存误差历史 ========
        self.last_err_x = err_x
        self.last_err_y = err_y
        self.last_err_a = err_a

    # ==================================================================
    # 重置状态（进入 IR 跟随模式时调用）
    # ==================================================================
    def reset_IR_follow(self):
        self.last_err_x = 0.0
        self.last_err_y = 0.0
        self.last_err_a = 0.0
        self.last_scale = 1.0
        self.current_scale = 1.0
        self.ir_lost_count = 0
        self.if_ir_lost = False
        self.if_frame_valid = False
        self.output_speed = 0.0
        self.output_angle = 0.0
        self.output_turn = 0.0  

    # 重置红外跟随角度
    def reset_follow_angle(self):
        self.output_turn = self.my_car.now_yaw * 180.0 / PI
