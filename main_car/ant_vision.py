import math

# 视觉伺服控制类(PD控制器)
class VisionManager:
    def __init__(self, flash_sys, beep, math, pose_data, angle_pid, servo_pid, sin_servo_fil, cos_servo_fil, my_uart3, car, protocol, order_manager, plan, state):
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

        # ================= 视觉伺服矫正相关变量 =================
        # 单应性矩阵（由cv2.findHomography求得，作用是将像素坐标转换为实际物理坐标，考虑了摄像头的内参和外参）
        self.close_H_matrix = [[ 1.73277793e+00, -4.03832162e-02, -1.43148863e+02],
                        [-2.48551759e-02, -1.47533261e+00,  1.77324371e+02],
                        [-1.12359403e-03,  5.60745066e-02,  1.00000000e+00]]
        
        self.far_H_matrix = [[ 1.73277793e+00, -4.03832162e-02, -1.43148863e+02],
                [-2.48551759e-02, -1.47533261e+00,  1.77324371e+02],
                [-1.12359403e-03,  5.60745066e-02,  1.00000000e+00]]
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
        self.if_lost_object = False       # type: bool   # 是否丢失目标物体标志位
        self.if_finish_servo = False      # 是否完成视觉伺服控制标志位
        self.if_gain_dis = False       # type: bool   # 是否获取目标距离标志位
        self.finish_orbit = False      # type: bool   # 是否完成环绕控制标志位
        self.if_ready_calibrate = False       # type: bool  # 判断是否准备好进行校准标志位
        self.if_finish_calibrate = False       # type: bool  # 判断是否完成校准标志位

    # 用单应性矩阵将像素坐标转换为实际物理坐标（单位：cm）
    def pixel_to_real_world(self, u, v, sign: str):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :param sign: 远近标志
        :return: 真实的物理坐标 (X_w, Y_w)
        """
        if self.my_state.state == self.my_state.SCAN or self.my_state.state == self.my_state.SERVO:
            if self.current_servo_object == 'T':
                object_h = 2.5
            elif self.current_servo_object in ['S', 'E']:
                object_h = 3.0
            elif self.current_servo_object in ['B', 'W']:
                object_h = 2.0
        else:
            # apriltag校准时忽略其高度
            object_h = 0.0
        # K为高度缩放系数，23.0为摄像头高度 
        K = (23.0 - object_h) / 23.0
        if sign == 'close':
            H_matrix = self.close_H_matrix
        else:
            H_matrix = self.far_H_matrix

        # 计算缩放因子
        w_prime = H_matrix[2][0] * u + H_matrix[2][1] * v + H_matrix[2][2]
        # 计算真实的物理坐标
        X_w = (H_matrix[0][0] * u + H_matrix[0][1] * v + H_matrix[0][2]) / w_prime * K
        Y_w = (H_matrix[1][0] * u + H_matrix[1][1] * v + H_matrix[1][2]) / w_prime * K

        return X_w, Y_w

    # 物体像素点坐标解算函数
    def calculate_dist(self, x: int, y: int, sign: str = 'far'):
        # correct_dist为经验修正值，考虑了车体直径和推杆长度
        correct_dist = 4.24
        # 将像素点坐标换算为相对坐标系下x和y方向上的实际偏移量
        self.relative_raw_x, self.relative_raw_y = self.pixel_to_real_world(x, y, sign)
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
        if self.if_finish_servo == False:
            if self.servo_stage == 2:
                self.target_point = self.my_art_protocol.coordinate_receive()
                if self.target_point and self.target_point[2] == self.current_servo_object and \
                    abs(self.target_point[0] - 80) < 40 and (self.target_point[1]) > 40:
                    self.calculate_dist(self.target_point[0], self.target_point[1], 'near')

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
                    
            now_error_x = self.real_servo_point[0] - self.my_car.x_current
            now_error_y = self.real_servo_point[1] - self.my_car.y_current
            if self.servo_lost_count <= 80:
                self.servo_pid.model_compute_pid(now_error_x, now_error_y)
                self.target_rel_speed_x = self.servo_pid.pwm_output_x
                self.target_rel_speed_y = self.servo_pid.pwm_output_y
            else:
                # 连续丢失超过一定帧数后，降低小车速度
                self.target_rel_speed = 50
                return 
            
            # 判断是否完成视觉伺服控制
            if (abs(now_error_x) <= self.finish_threshold_x and abs(now_error_y) <= self.finish_threshold_y and self.servo_stage == 1) or \
                (abs(self.absolute_actual_x) <= self.apriltag_threshold_x and abs(self.absolute_actual_y) <= self.apriltag_threshold_y and self.servo_stage == 2):
                if self.servo_stage == 1:
                    self.servo_stage = 2
                    # 记录下小车当前的坐标点
                    self.last_car_x = self.my_car.x_current
                    self.last_car_y = self.my_car.y_current
                elif self.servo_stage == 2:
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.my_order_manager.finish()
                    self.if_finish_servo = True
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
        """'L'代表左边线, 'R'代表右边线, 'U'代表上边线, 'D'代表下边线"""
        if self.if_ready_calibrate == False:
            # 准备阶段：调整小车位置和角度，面向apriltag
            if self.car_position == 'L':
                self.my_plan.navigate(target_turn_angle = -90.0)
            elif self.car_position == 'U':
                self.my_plan.navigate(target_turn_angle = 0.0)
            elif self.car_position == 'R':
                self.my_plan.navigate(target_turn_angle = 90.0)
            elif self.car_position == 'D':
                self.my_plan.navigate(target_turn_angle = 180.0)
            
            if self.my_plan.if_finish_navigate == True:
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
                # 清空目标角度缓冲区
                self.angle_buffer.clear()
                # 清空目标坐标缓冲区
                self.point_buffer.clear()
                # 重置阶段标志
                self.if_ready_calibrate = True
                # 重置速度和转角
                self.target_rel_speed = 0
                self.target_rel_yaw = 0.0
                self.target_rel_turn_angle = self.my_plan.turn_angle_target
                self.my_order_manager.mode_apriltag()                
        else:
            if self.if_finish_calibrate == False:
                target_point = self.my_art_protocol.apriltag_receive()
                if target_point:
                    # 重置掉帧计数
                    self.servo_lost_count = 0
                    # self.angle_temp = target_point[2]
                    corrected_x, corrected_y = self.pixel_to_real_world(target_point[0], target_point[1], 'close')
                    self.point_buffer.append((corrected_x, corrected_y))
                    self.angle_buffer.append(90.0 + target_point[2])
                else:       
                    self.servo_lost_count += 1
                    # 连续丢失150帧apriltag坐标后（在1.5s内不再收到物体坐标信息），认为apriltag丢失，停止小车运动
                    if self.servo_lost_count >= 150:
                        self.target_rel_speed = 0
                        self.target_rel_yaw = 0.0
                        self.servo_lost_count = 0
                        self.if_lost_object = True

                if len(self.point_buffer) >= 5 and len(self.angle_buffer) >= 5:
                    # 取平均值作为当前的目标坐标和目标角度，减少偶尔的解算异常带来的影响
                    avg_x = sum([p[0] for p in self.point_buffer]) / len(self.point_buffer)
                    avg_y = sum([p[1] for p in self.point_buffer]) / len(self.point_buffer)
                    avg_angle = sum(self.angle_buffer[2:]) / len(self.angle_buffer[2:])
                    relative_angle = math.atan2(-avg_x, avg_y) * 180.0 / self.MATH.PI
                    # 世界坐标系下的真实角度 = 车体坐标系下的目标角度 + 小车当前的角度
                    real_angle = avg_angle + relative_angle
                    if real_angle > 180.0:
                        real_angle -= 360.0
                    elif real_angle < -180.0:
                        real_angle += 360.0
                    real_dist = math.sqrt(avg_x ** 2 + avg_y ** 2)
                    real_x = real_dist * math.sin(real_angle * self.MATH.PI / 180.0)
                    real_y = real_dist * math.cos(real_angle * self.MATH.PI / 180.0)

                    # 里程计和姿态角硬复位
                    self.pose_data.reset_yaw(avg_angle)
                    
                    if self.car_position == 'L':
                        self.my_car.x_current = self.assist_car_pos[0] + real_y
                        self.my_car.y_current = self.assist_car_pos[1] - real_x
                    elif self.car_position == 'U':
                        self.my_car.x_current = self.assist_car_pos[0] - real_x
                        self.my_car.y_current = self.assist_car_pos[1] - real_y
                    elif self.car_position == 'R':
                        self.my_car.x_current = self.assist_car_pos[0] - real_y
                        self.my_car.y_current = self.assist_car_pos[1] + real_x
                    elif self.car_position == 'D':
                        self.my_car.x_current = self.assist_car_pos[0] + real_x
                        self.my_car.y_current = self.assist_car_pos[1] + real_y

                    self.angle_buffer.clear()
                    self.point_buffer.clear()
                    # 重置速度和转角
                    self.target_rel_speed = 0
                    self.target_rel_yaw = 0.0
                    self.target_rel_turn_angle = self.my_car.now_yaw * 180.0 / self.MATH.PI
                    self.my_order_manager.finish()
                    self.if_finish_calibrate = True 

    # 用于准备视觉伺服和环绕
    def ready_servo_and_orbit(self, target_point):
        # 选择合适的里程计系数
        self.my_car.alpha_x = 1.0
        self.my_car.alpha_y = 1.0
        # 控制小车面向物体进行视觉伺服控制
        self.target_rel_turn_angle = self.my_plan.turn_angle_target
        self.current_servo_object = target_point[2]
        # 根据物品种类选择伺服距离、环绕半径和搬运速度
        if self.current_servo_object == ord('T'):
            self.my_plan.error_x = self.my_plan.error_x_T
            self.final_dist = self.servo_pid.target_y_T
            self.object_radius = self.radius_T
            self.orbit_angle = self.angle_T
            self.my_plan.move_v_max = self.my_plan.move_v_max_T
        elif self.current_servo_object == ord('S') or self.current_servo_object == ord('E'):
            self.my_plan.error_x = self.my_plan.error_x_S
            self.final_dist = self.servo_pid.target_y_S
            self.object_radius = self.radius_S
            self.orbit_angle = self.angle_S
            self.my_plan.move_v_max = self.my_plan.move_v_max_S
        elif self.current_servo_object == ord('B') or self.current_servo_object == ord('W'):
            self.my_plan.error_x = self.my_plan.error_x_B
            self.final_dist = self.servo_pid.target_y_B
            self.object_radius = self.radius_B
            self.orbit_angle = self.angle_B
            self.my_plan.move_v_max = self.my_plan.move_v_max_B

        # 第一帧图像预测伺服点位
        self.last_car_x = self.my_car.x_current
        self.last_car_y = self.my_car.y_current
        self.calculate_dist(target_point[0], target_point[1])
        # 若预测距离过小（采用近距离的单应性矩阵进行解算，否则采用远距离的单应性矩阵进行解算（因为单应性矩阵的解算误差会随着距离增加而增加）
        if self.actual_dist <= 20.0:
            self.servo_stage = 2
        else:
            self.servo_stage = 1