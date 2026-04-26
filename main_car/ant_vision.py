import math

# 视觉伺服控制类(PD控制器)
class VisionManager:
    def __init__(self, flash_sys, beep, math, pose_data, angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, kf_target_x_fil, kf_target_y_fil, my_uart3, car, protocol, order_manager, plan, state):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入数学常量对象
        self.MATH = math
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
        # 注入TOF测距对象，用于测距
        """
        self.my_tof = tof
        self.tof_distance = 0       # type: float # TOF测距值
        self.tof_buffer = []        # type: list  # TOF测距缓存列表
        self.tof_distance_fil = tof_distance_fil     # TOF测距滤波器对象
        """
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
        # 'T'为网球， 'S'为沙袋，'B'为玩具熊
        self.current_servo_object = ''
        # 当前伺服连续丢失物体的帧数
        self.servo_lost_count = 0
        # 视觉伺服失败的次数
        self.failed_servo_count = 0 
        # 最终小车停在物体前的距离（随着物体种类改变）
        self.final_dist = 0.0
        # PD控制相关变量
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")  # type: float  # 视觉伺服控制距离阈值
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")  # type: float  # 视觉伺服控制距离阈值
        self.apriltag_threshold_x = self.flash_sys.find_value("apriltag_threshold_x")  # type: float  # 视觉伺服控制距离阈值
        self.apriltag_threshold_y = self.flash_sys.find_value("apriltag_threshold_y")  # type: float  # 视觉伺服控制距离阈值
        self.target_rel_speed_x = 0          # type: int   # 伺服控制目标x速度
        self.target_rel_speed_y = 0          # type: int   # 伺服控制目标y速度
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")  # type: int   # 视觉伺服控制最大速度
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")  # type: int   # 视觉伺服控制最小速度 
        self.dist_threshold = self.flash_sys.find_value("dist_threshold")    # type: float # 物体距离多远认定为合理
        self.target_point = []                      # type: list   # 目标点像素坐标
        self.target_rel_speed = 0                   # type: int     # 目标速度
        self.target_rel_yaw = 0.0                   # type: float   # 目标航向角
        self.target_rel_turn_angle = 0.0            # type: float   # 目标转角

        # ================= 卡尔曼滤波与矫正相关变量 =================
        # 单应性矩阵（由cv2.findHomography求得，作用是将像素坐标转换为实际物理坐标，考虑了摄像头的内参和外参）
        self.H_matrix = [[ 1.73277793e+00, -4.03832162e-02, -1.43148863e+02],
                        [-2.48551759e-02, -1.47533261e+00,  1.77324371e+02],
                        [-1.12359403e-03,  5.60745066e-02,  1.00000000e+00]]
        # 为 X 和 Y 坐标分别建立卡尔曼滤波器
        # P: 估计误差协方差, Q: 过程噪声(越小越信任预测), R: 测量噪声(越大越信任滤波，抗抖动)
        self.kf_target_x = kf_target_x_fil 
        self.kf_target_y = kf_target_y_fil
        
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
        self.orbit_radius = 0.0            # type: float   # 环绕半径
        self.orbit_speed = 0               # type: int     # 环绕速度
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
        self.direct = 0     # 0为顺时针，1为逆时针

        # 延时计数器
        self.counter = 0       # type: int     # 延时计数器
        # 目标角度缓冲区
        self.angle_buffer = []     # type: list    # 目标角度缓冲区
        # 微调阶段
        self.adjust_stage = 1      
        # 边线矫正时小车位置
        self.car_position = 0
        # 临时用于测试的角度变量
        self.angle_temp = 0.0
        # 矫正次数
        self.calibrate_times = 0       # type: int     # 矫正次数
        # 标志位
        self.if_lost_object = False       # type: bool   # 是否丢失目标物体标志位
        self.if_send_servo_command = False   # type: bool   # 是否发送视觉伺服控制指令标志位
        self.finish_servo = False      # 是否完成视觉伺服控制标志位
        self.if_gain_dis = False       # type: bool   # 是否获取目标距离标志位
        self.finish_orbit = False      # type: bool   # 是否完成环绕控制标志位
        self.if_ready_calibrate = False       # type: bool  # 判断是否准备好进行校准标志位
        self.if_gain_calibrate_angle = False   # type: bool  # 判断是否获取校准角度标志位
        self.if_finish_calibrate = False       # type: bool  # 判断是否完成校准标志位

    # 用单应性矩阵将像素坐标转换为实际物理坐标（单位：cm）
    def pixel_to_real_world(self, u, v):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :return: 真实的物理坐标 (X_w, Y_w)
        """
        # 计算缩放因子
        w_prime = self.H_matrix[2][0] * u + self.H_matrix[2][1] * v + self.H_matrix[2][2]
        # 计算真实的物理坐标
        X_w = (self.H_matrix[0][0] * u + self.H_matrix[0][1] * v + self.H_matrix[0][2]) / w_prime
        Y_w = (self.H_matrix[1][0] * u + self.H_matrix[1][1] * v + self.H_matrix[1][2]) / w_prime

        return X_w, Y_w

    # 动态调整视觉伺服pid参数
    def adjust_pid_by_dist(self, dist):
        # 距离越近，Kp 越小，防止超调；
        scale = max(0.45, min(1.0, dist / 15.0)) # 15cm外全速，近处最少降至45%
        self.servo_pid.servo_kp_x = self.servo_pid.servo_normal_kp_x * scale
        self.servo_pid.servo_kp_y = self.servo_pid.servo_normal_kp_y * scale

    # 物体像素点坐标解算函数
    def calculate_dist(self, x: int, y: int):
        # correct_dist为经验修正值，考虑了车体直径和推杆长度
        correct_dist = 4.24
        # 将像素点坐标换算为相对坐标系下x和y方向上的实际偏移量
        self.relative_raw_x, self.relative_raw_y = self.pixel_to_real_world(x, y)
        self.relative_raw_y = self.relative_raw_y - correct_dist - self.final_dist
        # 根据小车记录的上一次坐标点进行矫正，避免因为小车移动导致的解算误差
        car_dist = math.sqrt((self.my_car.x_current - self.last_car_x) ** 2 + (self.my_car.y_current - self.last_car_y) ** 2)
        car_yaw = -math.atan2(-(self.my_car.x_current - self.last_car_x), (self.my_car.y_current - self.last_car_y)) * 180.0 / self.MATH.PI
        relative_yaw = car_yaw * self.MATH.PI / 180.0 - self.my_car.now_yaw
        # 限幅
        if relative_yaw > self.MATH.PI:
            relative_yaw -= 2 * self.MATH.PI
        elif relative_yaw < -self.MATH.PI:
            relative_yaw += 2 * self.MATH.PI
        self.relative_actual_x = self.relative_raw_x - (car_dist * math.sin(relative_yaw))
        self.relative_actual_y = self.relative_raw_y - (car_dist * math.cos(relative_yaw))
        self.actual_dist = math.sqrt(self.relative_actual_x ** 2 + self.relative_actual_y ** 2)
        # 计算物体相对于小车的绝对偏差
        now_yaw = self.my_car.now_yaw * 180 / self.MATH.PI
        rel_yaw = -math.atan2(-self.relative_actual_x, self.relative_actual_y) * 180.0 / self.MATH.PI
        actual_yaw = now_yaw + rel_yaw
        if actual_yaw > 180.0:
            actual_yaw -= 360.0
        elif actual_yaw < -180.0:
            actual_yaw += 360.0
        self.absolute_actual_x = self.actual_dist * math.sin(actual_yaw * self.MATH.PI / 180.0)
        self.absolute_actual_y = self.actual_dist * math.cos(actual_yaw * self.MATH.PI / 180.0)
        self.real_servo_point = [self.my_car.x_current + self.absolute_actual_x, self.my_car.y_current + self.absolute_actual_y]
        # 测试打印
        # self.my_uart3.write(f"{self.relative_raw_x},{self.relative_raw_y}\r\n")

    def visual_servo_control(self):
        # 选择合适的里程计系数
        self.my_car.alpha_x = 1.0
        self.my_car.alpha_y = 1.0
        if self.finish_servo == False:
            # 1. 尝试接收新一帧数据
            self.target_point = self.my_art_protocol.coordinate_receive()
            
            # 2. 判断是否收到有效的新视觉帧
            if self.target_point and self.target_point[2] == self.current_servo_object and self.target_point[1] >= self.dist_threshold:
                self.calculate_dist(self.target_point[0], self.target_point[1])

                # 记录下小车当前的坐标点
                self.last_car_x = self.my_car.x_current
                self.last_car_y = self.my_car.y_current

                # 重置掉帧计数
                self.servo_lost_count = 0
            else:
                self.servo_lost_count += 1
                # 彻底丢失保护
                if self.servo_lost_count >= 150:
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.if_lost_object = True
                    self.servo_lost_count = 0
                    return # 彻底丢失，跳出伺服逻辑
                
            # 用预测的点位，依赖惯导，进行pid控制
            now_error_x = self.real_servo_point[0] - self.my_car.x_current
            now_error_y = self.real_servo_point[1] - self.my_car.y_current
            dist = math.sqrt(now_error_x ** 2 + now_error_y ** 2)
            # 根据剩余距离动态调整pid参数
            self.adjust_pid_by_dist(dist)
            # ================= 高频控制解耦 =================
            if self.servo_lost_count <= 80:
                self.servo_pid.model_compute_pid(now_error_x, now_error_y)
                self.target_rel_speed_x = self.servo_pid.pwm_output_x
                self.target_rel_speed_y = self.servo_pid.pwm_output_y
            else:
                # 连续丢失超过一定帧数后，降低小车速度
                self.target_rel_speed = 50
                return 
                        
            # 测试打印
            # self.my_uart3.write(f"{now_error_x},{now_error_y}\r\n")

            # 调节停止阈值缩放系数
            if self.current_servo_object == 'W' or self.current_servo_object == 'B':
                threshold_scale = 2
            else:
                threshold_scale = 1.0
            finish_threshold_x = self.finish_threshold_x * threshold_scale
            finish_threshold_y = self.finish_threshold_y * threshold_scale

            # 4. 判断是否完成视觉伺服控制
            if abs(self.absolute_actual_x) <= finish_threshold_x and abs(self.absolute_actual_y) <= finish_threshold_y:
                self.target_rel_speed = 0
                self.target_rel_yaw = 0.0
                self.my_order_manager.finish()
                self.finish_servo = True
            else:
                # 原有的滤波和速度限制逻辑保持不变
                self.target_rel_speed_x = self.sin_servo_fil.filtering(self.target_rel_speed_x)
                self.target_rel_speed_y = self.cos_servo_fil.filtering(self.target_rel_speed_y)                                            
                self.target_rel_speed = int(math.sqrt(self.target_rel_speed_x ** 2 + self.target_rel_speed_y ** 2))
                # 计算目标角度，单位：度（注意避免除以0）
                self.target_rel_yaw = -math.atan2(-self.target_rel_speed_x, self.target_rel_speed_y) * 180.0 / self.MATH.PI
                if self.target_rel_yaw > 180.0:
                    self.target_rel_yaw -= 360.0
                elif self.target_rel_yaw < -180.0:
                    self.target_rel_yaw += 360.0  
                if self.target_rel_yaw > 40.0 or self.target_rel_yaw < -40.0:
                    self.target_rel_speed = int(self.target_rel_speed * 0.6)
                self.target_rel_speed = max(self.min_rel_speed, min(self.target_rel_speed, self.max_rel_speed))

    # 环绕控制函数，传入环绕物体旋转的目标角度（单位：度），顺时针为正，逆时针为负
    def orbit_control(self, target_angle: float):
        if self.if_gain_dis == False:
            self.my_car.alpha_x = 1.0
            self.my_car.alpha_y = 1.0
            # 保持静止采集tof数据
            self.orbit_speed = 0
            # 如果当前状态为环绕模式则根据物体半径设置环绕半径，如果在反环绕模式下则设置成上一次的值
            if self.my_state.state == self.my_state.ORBIT:
                self.orbit_radius = self.object_radius
            self.record_angle = self.my_car.now_yaw * 180 / self.MATH.PI
            self.target_angle = self.record_angle + target_angle
            # 限制目标角度在-180到180度之间
            if self.target_angle > 180.0:
                self.target_angle -= 360.0
            elif self.target_angle < -180.0:
                self.target_angle += 360.0
            # 确定旋转方向（顺时针还是逆时针）
            if target_angle >= 0.0:
                self.direct = 0
            else:
                self.direct = 1 
            self.current_dis = 0.0
            self.if_gain_dis = True
        else:
            if self.finish_orbit == False:
                # 更新当前小车的行驶距离
                self.current_dis += self.my_car.car_speed_x
                # 更新当前小车的目标转角及目标航向角
                self.orbit_turn_angle = -self.current_dis / self.orbit_radius * 180.0 / self.MATH.PI + self.record_angle
                if self.orbit_turn_angle >= 180.0:
                    self.orbit_turn_angle -= 360.0
                elif self.orbit_turn_angle <= -180.0:
                    self.orbit_turn_angle += 360.0
                    
                if self.direct == 0:
                    self.orbit_yaw = -90.0 + self.orbit_turn_angle
                elif self.direct == 1:
                    self.orbit_yaw = 90.0 + self.orbit_turn_angle
                    
                if self.orbit_yaw >= 180.0:
                    self.orbit_yaw -= 360.0
                elif self.orbit_yaw <= -180.0:
                    self.orbit_yaw += 360.0
                # 更新当前小车的速度
                diff = abs(self.target_angle - self.my_car.now_yaw * 180 / self.MATH.PI)
                if diff > 180.0:
                    diff = 360.0 - diff

                # 环绕速度规划：当小车与目标角度的差值大于30度时，保持最大速度；当差值小于30度时，线性降低速度，直到差值小于等于1度时停止
                if diff < 40.0:
                    self.orbit_speed = self.orbit_v_max - (self.orbit_v_max - self.orbit_v_min) * (40.0 - diff) / 40.0
                else:
                    self.orbit_speed = self.orbit_v_max
                # 速度限幅
                self.orbit_speed = max(self.orbit_v_min, min(self.orbit_speed, self.orbit_v_max))

                # 判断是否完成环绕
                if diff <= 1.0:	
                    self.orbit_speed = 0
                    self.orbit_turn_angle = self.my_car.now_yaw * 180 / self.MATH.PI
                    self.finish_orbit = True

    # apriltag辅助校准校准控制函数
    def apriltag_calibrate_control(self):
        """0代表下边线左侧,1代表下边线右侧, 2代表上边线左侧, 3代表上边线右侧"""
        if self.if_ready_calibrate == False:
            # 判断小车处于上下左右哪个边线，并微调小车位置使其更靠近边线（避免因惯性过大导致无法识别边线）
            # 进行两阶段的微调
            if self.adjust_stage == 1:
                if self.car_position == 1:
                    self.my_plan.navigate([[self.my_car.x_current + 5.0, self.my_car.y_current], [self.my_car.x_current + 5.0, 0.0]], self.my_car.now_yaw * 180.0 / self.MATH.PI)
                elif self.car_position == 3:
                    self.my_plan.navigate([[self.my_car.x_current + 5.0, self.my_car.y_current], [self.my_car.x_current + 5.0, 240.0]], self.my_car.now_yaw * 180.0 / self.MATH.PI)
                elif self.car_position == 0:
                    self.my_plan.navigate([[self.my_car.x_current - 5.0, self.my_car.y_current], [self.my_car.x_current - 5.0, 0.0]], self.my_car.now_yaw * 180.0 / self.MATH.PI)
                elif self.car_position == 2:
                    self.my_plan.navigate([[self.my_car.x_current - 5.0, self.my_car.y_current], [self.my_car.x_current - 5.0, 240.0]], self.my_car.now_yaw * 180.0 / self.MATH.PI)
                if self.my_plan.finish_navigate == True:
                    self.adjust_stage = 2
                    self.my_plan.finish_navigate = False
            elif self.adjust_stage == 2:
                if self.car_position == 1:
                    self.my_plan.navigate([[185.0, 0.0]], -90.0)
                elif self.car_position == 0:
                    self.my_plan.navigate([[135.0, 0.0]], 90.0)
                elif self.car_position == 2:
                    self.my_plan.navigate([[135.0, 240.0]], 90.0)
                elif self.car_position == 3:
                    self.my_plan.navigate([[185.0, 240.0]], -90.0)
                if self.my_plan.finish_navigate == True:
                    # 选择合适的里程计系数
                    self.my_car.alpha_x = 1.0
                    self.my_car.alpha_y = 1.0
                    # 选择矫正状态下的pid参数
                    self.servo_pid.servo_kp_x = self.servo_pid.servo_calibrate_kp_x
                    self.servo_pid.servo_kd_x = self.servo_pid.servo_calibrate_kd_x
                    self.servo_pid.servo_kp_y = self.servo_pid.servo_calibrate_kp_y
                    self.servo_pid.servo_kd_y = self.servo_pid.servo_calibrate_kd_y
                    # 伺服apriltag时固定目标点坐标（单位：像素），并且固定目标转角为0（即小车面向apriltag）
                    self.servo_pid.target_y = self.servo_pid.target_y_A
                    self.counter = 0
                    self.calibrate_times = 0
                    # 清空目标角度缓冲区
                    self.angle_buffer.clear()
                    # 重置阶段标志
                    self.adjust_stage = 1
                    self.if_ready_calibrate = True
                    self.my_plan.finish_navigate = False
                    self.target_rel_turn_angle = self.my_plan.turn_angle_target
                    self.my_order_manager.mode_apriltag()
                    
        else:
            target_point = self.my_art_protocol.apriltag_receive()
            if target_point:
                self.servo_lost_count = 0
                self.angle_temp = target_point[2]
                if self.if_gain_calibrate_angle == False or self.calibrate_times == 1:
                    if self.calibrate_times == 1:
                        # 计算目标转角(多次测量取平均值)
                        if self.car_position == 0 or self.car_position == 2:
                            self.angle_buffer.append(90.0 + target_point[2])
                        elif self.car_position == 1 or self.car_position == 3:
                            self.angle_buffer.append(-90.0 - target_point[2])
                    else:
                        now_yaw = self.my_car.now_yaw * 180.0 / self.MATH.PI
                        # 计算目标转角
                        if self.car_position == 0 or self.car_position == 2:
                            self.target_rel_turn_angle = now_yaw - target_point[2]
                        elif self.car_position == 1 or self.car_position == 3:
                            self.target_rel_turn_angle = now_yaw + target_point[2]
                        self.if_gain_calibrate_angle = True

                self.servo_pid.color_compute_pid(target_point[0], target_point[1])
                self.target_rel_speed_x = self.servo_pid.pwm_output_x
                self.target_rel_speed_y = self.servo_pid.pwm_output_y
                
                if self.if_finish_calibrate == False:
                    # 判断是否完成视觉伺服控制
                    diff = abs(self.target_rel_turn_angle - self.my_car.now_yaw * 180.0 / self.MATH.PI)
                    if diff > 180.0:
                        diff = 360.0 - diff
                    if ((abs(self.servo_pid.nowError_x) <= self.apriltag_threshold_x and abs(self.servo_pid.nowError_y) <= self.apriltag_threshold_y) and diff <= 1.0 and self.calibrate_times != 1) or len(self.angle_buffer) >= 10:
                        self.target_rel_speed = 0
                        self.target_rel_yaw = 0.0
                        self.calibrate_times += 1
                        # 完成两次矫正才算结束
                        if self.calibrate_times >= 2:
                            self.calibrate_times = 0
                            self.counter = 0
                            # 里程计和姿态角硬复位
                            self.pose_data.reset_yaw(sum(self.angle_buffer[2:]) / len(self.angle_buffer[2:]))
                            self.angle_buffer.clear()
                            if self.car_position == 0:
                                self.my_car.x_current = 137.0
                                self.my_car.y_current = 0.0
                            elif self.car_position == 1:
                                self.my_car.x_current = 183.0
                                self.my_car.y_current = 0.0
                            elif self.car_position == 2:
                                self.my_car.x_current = 137.0
                                self.my_car.y_current = 240.0
                            elif self.car_position == 3:
                                self.my_car.x_current = 183.0
                                self.my_car.y_current = 240.0
                            # 在切换模式前保持当前转角
                            self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / self.MATH.PI
                            self.my_order_manager.finish()
                            self.if_finish_calibrate = True
                    else:
                        # 计算综合目标速度和航向角
                        # 滤波
                        self.target_rel_speed_x = self.sin_servo_fil.filtering(self.target_rel_speed_x)
                        self.target_rel_speed_y = self.cos_servo_fil.filtering(self.target_rel_speed_y)                                            
                        # 计算目标角度，单位：度（注意避免除以0）
                        self.target_rel_yaw = -math.atan2(-self.target_rel_speed_x, self.target_rel_speed_y) * 180.0 / self.MATH.PI + self.target_rel_turn_angle
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
            else:
                self.servo_lost_count += 1
                # 连续丢失150帧apriltag坐标后（在1.5s内不再收到物体坐标信息），认为apriltag丢失，停止小车运动
                if self.servo_lost_count >= 150:
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.servo_lost_count = 0
                    self.if_lost_object = True