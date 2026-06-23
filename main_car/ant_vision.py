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
        self.close_H_matrix = [[ 4.26029056e+00, -6.84019370e-02, -3.08421308e+02],
                            [ 4.43673677e-16, -4.32808717e+00,  4.32808717e+02],
                            [ 1.97122056e-17,  1.60411622e-01,  1.00000000e+00]]
        
        self.far_H_matrix = [[ 4.26029056e+00, -6.84019370e-02, -3.08421308e+02],
                            [ 4.43673677e-16, -4.32808717e+00,  4.32808717e+02],
                            [ 1.97122056e-17,  1.60411622e-01,  1.00000000e+00]]
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
        self.car_radius = 11.0   # 小车推杆到中心的距离
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
        self.analysed_objects = {
            'T':[],
            'S':[],
            'B':[],
            'W':[],
            'E':[],
        }
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

    # 用单应性矩阵将像素坐标转换为实际物理坐标（单位：cm）
    def pixel_to_real_world(self, u, v, sign: str):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :param sign: 远近标志
        :return: 真实的物理坐标 (X_w, Y_w)
        """

        # 默认值，防止 current_servo_object 为空或匹配不到时出现未赋值报错
        object_H = 0.0
        if self.my_state == CALIBRATE:
            object_H = 1.75
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

        K = (22.0 - object_H) / 22.0
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
        self.servo_pid.servo_kp_x = self.servo_pid.servo_normal_kp_x * scale
        self.servo_pid.servo_kp_y = self.servo_pid.servo_normal_kp_y * scale
    def reset_analysed_objects(self):
        self.analysed_objects = {
            'T':[],
            'S':[],
            'B':[],
            'W':[],
            'E':[],
        }
    def if_in_rect(self,x,y):
        rect_x_min = self.my_plan.plan_data.center_rect[0][0]-5
        rect_x_max = self.my_plan.plan_data.center_rect[3][0]+5
        rect_y_min = self.my_plan.plan_data.center_rect[0][1]-5
        rect_y_max = self.my_plan.plan_data.center_rect[3][1]+5
        if x < rect_x_min or x > rect_x_max or\
            y < rect_y_min or y > rect_y_max:
            return False
        return True
    def analyse_object_coordinate(self, package, if_cover=False):
        for i in package[1]:
            object_kind = chr(i[0]) if isinstance(i[0], int) else i[0]
            if object_kind not in self.analysed_objects:
                continue
            self.current_servo_object = object_kind
            new_p = self.calc_object_global_pos(i[1], i[2])
            new_x = new_p[0]
            new_y = new_p[1]
            if not self.if_in_rect(new_x,new_y):continue
            if if_cover:
                for j in range(len(self.analysed_objects[object_kind])):
                    jj = self.analysed_objects[object_kind][j]
                    if (new_x - jj[0])**2 + (new_y - jj[1])**2 < 64:
                        new_x = (new_x + jj[0]) / 2
                        new_y = (new_y + jj[1]) / 2
                        self.analysed_objects[object_kind].pop(j)
                        break
            self.analysed_objects[object_kind].append((new_x, new_y))

    # 物体像素点坐标解算函数
    def calculate_dist(self, x: int, y: int, sign: str = 'far'):
        # 将像素点坐标换算为相对坐标系下x和y方向上的实际偏移量
        self.relative_raw_x, self.relative_raw_y = self.pixel_to_real_world(x, y, sign)
        self.relative_raw_y = self.relative_raw_y - self.final_dist
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
    def calc_object_global_pos(self, pixel_x, pixel_y, sign='far'):
        # 像素点 -> 车体坐标系下真实坐标
        rel_x, rel_y = self.pixel_to_real_world(pixel_x, pixel_y, sign)
        rel_y+=13
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
                self.if_lost_object = True
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
            # self.my_order_manager.finish()
            # 选择正常伺服状态下的pid参数
            self.servo_pid.servo_kp_x = self.servo_pid.servo_normal_kp_x
            self.servo_pid.servo_kd_x = self.servo_pid.servo_normal_kd_x
            self.servo_pid.servo_kp_y = self.servo_pid.servo_normal_kp_y
            self.servo_pid.servo_kd_y = self.servo_pid.servo_normal_kd_y
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

    # 环绕控制函数，传入环绕物体旋转的目标世界坐标系角度（单位：度）（范围：-180到180）
    def orbit_control(self, target_angle: float, direct = None):
        if self.if_orbit_ready == False:
            # 选择合适的里程计系数（无负压）
            self.my_car.alpha_x = 0.951256
            self.my_car.alpha_y = 0.922584
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
            accel_range = min(25.0, self.total_orbit_angle / 2.0)   # 加速区间（度）
            decel_range = min(25.0, self.total_orbit_angle / 2.0)   # 减速区间（度）
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
    # 用于准备视觉伺服和环绕
    def ready_servo_and_orbit(self, target_point, state = "servo"):
        # 选择合适的里程计系数（无负压）
        self.my_car.alpha_x = 0.951256
        self.my_car.alpha_y = 0.922584
        # 选择正常伺服状态下的pid参数
        self.servo_pid.servo_kp_x = self.servo_pid.servo_normal_kp_x
        self.servo_pid.servo_kd_x = self.servo_pid.servo_normal_kd_x
        self.servo_pid.servo_kp_y = self.servo_pid.servo_normal_kp_y
        self.servo_pid.servo_kd_y = self.servo_pid.servo_normal_kd_y
        
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
            self.final_dist *= 0.7

        # 第一帧图像预测伺服点位
        self.last_car_x = self.my_car.x_current
        self.last_car_y = self.my_car.y_current
        self.calculate_dist(target_point[0], target_point[1], 'far')


# 搬运控制类
class MoveControl:
    def __init__(self, beep, photo, car, plan,path, plan_data,move_plan, vision_manager: VisionManager, state, main_protocol, art_protocol, order_manager, assist_protocol):
        self.my_beep = beep
        self.my_photo = photo
        self.vision_manager = vision_manager
        self.my_plan = plan
        self.my_path = path
        self.plan_data = plan_data
        self.my_car = car
        self.my_state = state
        self.my_main_protocol = main_protocol
        self.my_art_protocol = art_protocol
        self.my_order_manager = order_manager
        self.my_assist_protocol = assist_protocol
        self.move_plan = move_plan
        self.now_object_pt = [0.0, 0.0]
        self.record_angle = 0.0  # 记录的角度(记录小车的最初的角度)

        self.navigate_buffer = []
        self.navigate_distance=18
        self.__angle=30
        self.surrounding_points = {
            'LU': [],
            'LD': [],
            'RU': [],
            'RD': [],
            'LDD': [],
            'RDD': [],
        }
        self.now_barriar = []
        self.moving_point = []   # 搬运途径点
        self.angle_buffer = []   # 角度缓冲区
        self.next_point = []     # 下一目标点
        self.adjust_point = []   # 微调目标点
        self.moving_idx = 0      # 搬运途径点索引
        self.move_dir = 0

        self.num_send_orbit_point=0

        self.current_state = ORBIT  # 当前状态：0为环绕，1为视觉伺服，2为搬运， 3为微调
        self.if_send_orbit_command = False  # 是否发送过环绕控制指令
        self.if_start_orbit = False  # 是否开始环绕
        self.if_finish_move = False  # 是否完成搬运
        self.plan_path = []
        self.send_point = []
        gc.collect()

    # 更新物体当前坐标，已知物体在小车正前方的距离 dist
    def update_object_pos(self):
        # 当前车头朝向 (弧度)
        now_yaw = self.my_car.now_yaw
        dist = self.vision_manager.final_dist + self.vision_manager.car_radius
        # 已知世界坐标系下向北(+Y)为0度，向东(+X)为90度
        # 车头指向的正方向向量为 (sin(now_yaw), cos(now_yaw))
        self.now_object_pt = [
            self.my_car.x_current + dist * math.sin(now_yaw),
            self.my_car.y_current + dist * math.cos(now_yaw)
        ]
    def calculate_object_pos(self,point):#用扫描的一帧计算位置
        self.now_object_pt = self.vision_manager.calc_object_global_pos(point[0],point[1])
    # 构建搬运途径点列表
    def build_moving_point(self,point):
        current_index = self.plan_data.current_index
        current_object = self.vision_manager.current_servo_object
        self.moving_point.clear()
        self.moving_point.append(self.now_object_pt[:])  # 物体位置（使用切片拷贝，避免引用污染）
        if self.plan_data.if_rogue_plan:
            object_message = self.plan_data.rogue_planning[current_index]
            move_step=object_message[3]
        else:
            move_step=[]
        for item in move_step: # 搬运途径点
            if item[0] == 'x':
                self.moving_point.append([item[1], self.moving_point[-1][1]])
            elif item[0] == 'y':
                self.moving_point.append([self.moving_point[-1][0], item[1]])
        if current_object == 'T':
            self.moving_point.append([self.moving_point[-1][0], 240.0])
            self.move_dir = 0
        elif current_object in ['S', 'E']:
            self.moving_point.append([0.0, self.moving_point[-1][1]])
            self.move_dir = -90
        elif current_object in ['B', 'W']:
            self.moving_point.append([320.0, self.moving_point[-1][1]])
            self.move_dir = 90
    # 判断小车编队到下一目标点时的转向（返回基于小车坐标系的相对朝向）
    def judge_next_turn(self, current_pt, next_pt, ref_yaw=None):
        if ref_yaw is None:ref_yaw = self.record_angle
        else:ref_yaw = ref_yaw * PI / 180.0
        dx = next_pt[0] - current_pt[0]
        dy = next_pt[1] - current_pt[1]
        # 将世界坐标系下的差值投影到小车坐标系 (按 Y轴为车头前方，X轴为车身右侧 进行转换)
        # 根据世界坐标向北为0度，向东为90度的定义推导的旋转变换
        cy = dx * math.sin(ref_yaw) + dy * math.cos(ref_yaw)
        cx = dx * math.cos(ref_yaw) - dy * math.sin(ref_yaw)
        if abs(cx) > abs(cy):
            if cx > 0:return 90.0  # 车身右侧
            else:return -90.0  # 车身左侧
        else:
            if cy > 0:return 0.0  # 车头前方
            else:return 180.0  # 车尾后方
    def get_object_square_points(self,car_angle,L):#寻找物体周围点位
        a=self.navigate_distance
        if car_angle == 0:
            forward = (0, 1)
            right = (1, 0)
        elif car_angle == 90:
            forward = (1, 0)
            right = (0, -1)
        elif car_angle == 180:
            forward = (0, -1)
            right = (-1, 0)
        elif car_angle == -90:
            forward = (-1, 0)
            right = (0, 1)
        else:raise ValueError("car_angle must be one of 0, 90, 180, -90")
        fx, fy = forward
        rx, ry = right
        lx, ly = -rx, -ry
        LU = [self.now_object_pt[0] + lx * a + fx * a, self.now_object_pt[1] + ly * a + fy * a]
        LD = [self.now_object_pt[0] + lx * a - fx * a, self.now_object_pt[1] + ly * a - fy * a]
        RU = [self.now_object_pt[0] + rx * a + fx * a, self.now_object_pt[1] + ry * a + fy * a]
        RD = [self.now_object_pt[0] + rx * a - fx * a, self.now_object_pt[1] + ry * a - fy * a]
        # 在 LD/RD 基础上，继续向靠近小车方向移动 L，也就是 -forward
        LDD = [LD[0] - fx * L, LD[1] - fy * L]
        RDD = [RD[0] - fx * L, RD[1] - fy * L]
        self.surrounding_points =  {
            'LU': LU,
            'LD': LD,
            'RU': RU,
            'RD': RD,
            'LDD': LDD,
            'RDD': RDD,
        }
    # 搬运前的准备
    def ready_move(self,point,new_side = None):
        if not point or len(point) < 2:return False
        #self.now_object_pt = self.vision_manager.calc_object_global_pos(point[0],point[1])
        self.now_object_pt = point[:]
        if not self.vision_manager.if_in_rect(self.now_object_pt[0],self.now_object_pt[1]):
            return False
        self.moving_idx = 0
        self.current_state = ORBIT
        self.if_finish_move = False
        self.if_start_orbit = False
        self.navigate_buffer.clear()
        self.my_plan.reset_navigate()
        self.my_plan.reset_navigate_angle()
        # 重置搬运点索引
        self.moving_idx = 0
        # 构建搬运途径点列表
        self.build_moving_point(point)
        # 记录小车当前角度
        self.record_angle = self.my_car.now_yaw  # 保持弧度制供 judge_next_turn 默认使用
        current_yaw_deg = self.record_angle * 180.0 / PI
        if not new_side:
            if current_yaw_deg > -45.0 and current_yaw_deg <= 45.0: current_turn_deg = 0.0
            elif current_yaw_deg > 45.0 and current_yaw_deg <= 135.0:current_turn_deg = 90.0
            elif current_yaw_deg > 135.0 or current_yaw_deg <= -135.0:current_turn_deg = 180.0
            elif current_yaw_deg > -135.0 and current_yaw_deg <= -45.0:current_turn_deg = -90.0
        else:
            if new_side =='D':current_turn_deg = 0.0
            elif new_side =='U':current_turn_deg = 180
            elif new_side =='L':current_turn_deg = 90       
            else:current_turn_deg = -90
        self.angle_buffer.clear()
        self.get_object_square_points(current_turn_deg,15)
        # 初始参考偏航角就是当前小车所在方向（度数）
        current_ref_yaw_deg = current_turn_deg
        for i in range(len(self.moving_point) - 1):
            # 获取基于 current_ref_yaw_deg 作为参照方向时的相对转向角度
            # turn_angle 可能是返回 0.0 (前方), 90.0 (右), -90.0 (左), 180.0 (后)
            turn_angle = self.judge_next_turn(self.moving_point[i], self.moving_point[i + 1], ref_yaw=current_ref_yaw_deg)
            # 世界坐标系下小车在下一步运动期望到达的实际偏航角
            target_turn = current_ref_yaw_deg + turn_angle
            # 角度限幅到 [-180, 180)
            target_turn = (target_turn + 180.0) % 360.0 - 180.0
            angle_l=(target_turn + self.__angle + 180.0) % 360.0 - 180.0
            angle_r=(target_turn - self.__angle + 180.0) % 360.0 - 180.0
            M_PAth = []
            if turn_angle == 0.0:
                m_PAth = [self.surrounding_points['LD']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_l,current_ref_yaw_deg]
            elif turn_angle == 90.0:
                m_PAth = [self.surrounding_points['LD'],self.surrounding_points['LU']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_l,current_ref_yaw_deg]
            elif turn_angle == 180.0:
                m_PAth = [self.surrounding_points['RD'],self.surrounding_points['RU']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_l,current_ref_yaw_deg]
            elif turn_angle == -90.0:
                m_PAth = [self.surrounding_points['RD'],self.surrounding_points['RU']]
                S_PAth = [self.vision_manager.current_servo_object,self.now_object_pt]
                ANGle = [angle_r,current_ref_yaw_deg]
            if new_side:
                if new_side =='D':self.my_path.plan_path(m_PAth[0][0],self.my_plan.plan_data.center_rect[0][1])
                elif new_side =='U':self.my_path.plan_path(m_PAth[0][0],self.my_plan.plan_data.center_rect[3][1])
                elif new_side =='L':self.my_path.plan_path(self.my_plan.plan_data.center_rect[0][0],m_PAth[0][1])   
                else:self.my_path.plan_path(self.my_plan.plan_data.center_rect[3][0],m_PAth[0][1])
                M_PAth = self.my_path.ready_path + m_PAth
            else:
                M_PAth = m_PAth
            self.navigate_buffer.append({
                            'MAIN_P':M_PAth,
                            'SLA_P':S_PAth,
                            'ANGLE':ANGle,
                        })
        self.moving_point.pop(0)  # 移除起点
        if self.vision_manager.current_servo_object == 'T':
            self.my_plan.error_x = self.my_plan.error_x_T
            self.final_dist = self.vision_manager.servo_pid.target_y_T
            self.object_radius = self.vision_manager.radius_T
            self.orbit_angle = self.vision_manager.angle_T
            self.my_plan.move_v_max = self.my_plan.move_v_max_T
        elif self.vision_manager.current_servo_object == 'S' or self.vision_manager.current_servo_object == 'E':
            self.my_plan.error_x = self.my_plan.error_x_S
            self.final_dist = self.vision_manager.servo_pid.target_y_S
            self.object_radius = self.vision_manager.radius_S
            self.orbit_angle = self.vision_manager.angle_S
            self.my_plan.move_v_max = self.my_plan.move_v_max_S
        elif self.vision_manager.current_servo_object == 'B' or self.vision_manager.current_servo_object == 'W':
            self.my_plan.error_x = self.my_plan.error_x_B
            self.final_dist = self.vision_manager.servo_pid.target_y_B
            self.object_radius = self.vision_manager.radius_B
            self.orbit_angle = self.vision_manager.angle_B
            self.my_plan.move_v_max = self.my_plan.move_v_max_B
        return True
    # 重置环绕控制标志位
    def reset_orbit(self):
        self.if_send_orbit_command = False
        self.if_start_orbit = False
        self.vision_manager.if_orbit_ready = False
        self.vision_manager.if_finish_orbit = False
        self.num_send_orbit_point=0
        self.surrounding_points.clear()
        self.navigate_buffer.clear()
        self.my_plan.reset_navigate()

    # 重置搬运控制相关变量
    def reset_move(self):
        self.moving_idx = 0
        self.moving_point.clear()
        self.angle_buffer.clear()
        self.next_point.clear()
        self.adjust_point.clear()
        self.current_state = ORBIT
        self.reset_orbit()
        self.if_finish_move = False
        gc.collect()

    # 重置小车里程计
    def reset_car_pos(self):
        current_object = self.vision_manager.current_servo_object
        light_to_center = 8.5  # 光电管到车体中心的距离
        COS = 0.707
        if current_object == 'T':
            self.my_car.y_current = 240.0 - light_to_center * COS
        elif current_object in ['S', 'E']:
            self.my_car.x_current = 0.0 + light_to_center * COS
        elif current_object in ['B', 'W']:
            self.my_car.x_current = 320.0 - light_to_center * COS
    # 计算微调的目标点
    def calculate_adjustment_point(self, fixed_dist = 5.0):
        # 当前车头朝向 (弧度)
        now_yaw = self.my_car.now_yaw
        # 已知世界坐标系下向北(+Y)为0度，向东(+X)为90度
        # 车头指向的正方向向量为 (sin(now_yaw), cos(now_yaw))
        # 逆着车头方向，就是向车身正后方偏移 fixed_dist 的距离
        target_x = self.my_car.x_current - fixed_dist * math.sin(now_yaw)
        target_y = self.my_car.y_current - fixed_dist * math.cos(now_yaw)
        self.adjust_point = [target_x, target_y]
    def calculate_move_path(self):
        objects=self.now_barriar
        if self.move_dir==0 or self.move_dir==180:
            if self.my_car.now_yaw>0:swell_dir=-90
            else:swell_dir=90
        elif self.move_dir==-90 or self.move_dir==90:
            if self.my_car.now_yaw>-PI/2 and self.my_car.now_yaw<PI/2:swell_dir=180
            else:swell_dir=0
        else: return False
        plan_path = self.move_plan.plan_move(self.move_dir,swell_dir,objects)
        if len(plan_path) == 2:
            self.send_point=[0,0]
        elif len(plan_path) == 3:
            self.send_point=[plan_path[1][0]-self.my_car.x_current,plan_path[1][1]-self.my_car.y_current]
        else: return False
        self.plan_path = plan_path[1:]
        return True 
    # 状态过渡函数
    def state_transition(self):
        global counter
        if self.current_state == ORBIT:
            if not self.if_send_orbit_command:#若还未发消息
                self.if_send_orbit_command = True
                NAV_T=self.navigate_buffer[self.moving_idx]
                self.my_main_protocol.send_path(NAV_T['SLA_P'][0],NAV_T['ANGLE'][1],NAV_T['SLA_P'][1])
            if self.vision_manager.if_send_order == False:#若还未打开摄像头
                # 打开摄像头
                self.my_order_manager.mode_target()
                self.vision_manager.if_send_order = True#从车完成后开始视觉
            
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and\
            target_point[1] >= 40.0:
                self.vision_manager.ready_servo_and_orbit(target_point, 'adjust')
                
                self.vision_manager.reset_servo_angle()
                self.current_state = ADJUST

                self.reset_orbit()  # 重置环绕相关变量
                self.vision_manager.if_send_order = False
        elif self.current_state == ADJUST:
            # 延时50ms再进行状态过渡，确保小车已经稳定在视觉伺服的起始位置，避免过早进入搬运状态导致丢失目标
            if counter >= 5:
                order = self.my_main_protocol.get_slave_state()
                if order == "finish":
                    self.my_beep.test()
                    counter = 0
                    self.vision_manager.if_finish_servo = False
                    #self.handle_next_point()
                    if self.calculate_move_path():
                        self.my_main_protocol.send_path('M',self.move_dir,self.send_point)
                        # 在最后一个搬运点前给辅助车发送具体坐标
                        self.my_plan.reset_navigate_angle()
                        self.my_plan.reset_navigate()
                        self.current_state = MOVE
                    else:
                        self.if_finish_move = True#直接退出return
                elif order == "lost":
                    counter = 0
                    self.if_finish_move = True
            else:
                counter += 1
        elif self.current_state == MOVE:
            # 如果当前搬运点是最后一个额外增加的终点指令，说明已经完成搬运
            #self.moving_idx += 1
            #if self.moving_idx >= len(self.moving_point):
            self.if_finish_move = True
            return
            '''
            if self.vision_manager.if_send_order == False:
                # 打开摄像头
                self.my_order_manager.mode_target()
                self.vision_manager.if_send_order = True
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.vision_manager.current_servo_object and\
            target_point[1] >= 40.0:
                self.vision_manager.ready_servo_and_orbit(target_point, 'servo')
                self.vision_manager.if_send_order = False
                self.my_plan.reset_navigate()   # 重置导航相关变量
                
                self.vision_manager.reset_servo_angle()
                self.current_state = SERVO
            '''
        elif self.current_state == SERVO:
            self.vision_manager.if_finish_servo = False
            self.vision_manager.reset_orbit_angle()
            self.current_state = ORBIT

    # 搬运控制函数
    def moving(self):
        if self.if_finish_move:
            return
        if self.current_state == ORBIT:
            if self.if_start_orbit == False:
                NAV_T=self.navigate_buffer[self.moving_idx]
                if NAV_T:
                    self.if_start_orbit = True
                    self.if_send_orbit_command = False
                    self.my_plan.navigate(NAV_T['MAIN_P'],NAV_T['ANGLE'][0])
            else:
                if not self.if_send_orbit_command and self.my_plan.finished_dist >= 15:
                    NAV_T=self.navigate_buffer[self.moving_idx]
                    self.my_main_protocol.send_path(NAV_T['SLA_P'][0],NAV_T['ANGLE'][1],NAV_T['SLA_P'][1])
                    self.if_send_orbit_command = True
                self.my_plan.navigate(self.navigate_buffer[self.moving_idx]['MAIN_P'],self.navigate_buffer[self.moving_idx]['ANGLE'][0])
                if self.my_plan.if_finish_navigate == True:
                    self.state_transition()
        elif self.current_state == ADJUST:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
        elif self.current_state == MOVE:
            #self.my_plan.navigate(path = [self.next_point])
            self.my_plan.navigate(path = self.plan_path)
            self.my_photo.update_photo_state()
            if self.my_photo.current_state == OutLine:
                self.reset_car_pos()
                self.my_photo.reset_photo()
                self.my_beep.test()
                self.my_plan.if_finish_navigate = True
            if self.my_plan.if_finish_navigate == True:
                self.state_transition()
        elif self.current_state == SERVO:
            self.vision_manager.visual_servo_control()
            if self.vision_manager.if_finish_servo == True:
                self.state_transition()
