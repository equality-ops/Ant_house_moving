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

# 状态机
class StateMachine:
    def __init__(self):        
        self.if_move_easy_object = False   # 是否搬运过易搬运物体的标志位（搬运过易搬运物体后在返回起点时不避开矩形区域）
        self.state = READY_NAVIGATE  # 初始状态为准备导航状态
        gc.collect()

# 路径和速度规划相关常量
class PlanData:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 地图固定点坐标
        # fixed_point[0]为从车起点，fixed_point[1]为矩形框左下方顶点，fixed_point[2]为矩形框右上方顶点, fixed_point[3]为从车返回点
        self.fixed_point = [[15.0, -14.0], [95.0, 55.0], [225.0, 185.0], [35.0, -25.0]]  # type: list

        gc.collect()
    

# 导航规划类
class NavigationPlan:
    def __init__(self, flash_sys, fan, plan_data: PlanData, car, state: StateMachine, order_manager, my_uart3, beep, art_protocol):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入无刷负压控制对象
        self.my_fan = fan
        # 注入小车位置对象
        self.my_car = car
        # 注入状态机对象
        self.my_state = state
        # 注入无线通信对象
        self.my_uart3 = my_uart3
        # 注入指令管理对象
        self.my_order_manager = order_manager
        # 注入蜂鸣器对象
        self.my_beep = beep
        # 注入openart串口解析对象
        self.my_art_protocol = art_protocol

        # 速度规划相关常量
        self.min_start_v = self.flash_sys.find_value("min_start_v")  # type: int  # 最小制动速度
        self.long_v_max = self.flash_sys.find_value("long_v_max")    # type: int  # 长距离时的最大速度
        self.acc_coef = 0.0          # 加速距离系数
        self.acc_normal_coef = self.flash_sys.find_value("acc_normal_coef")     # 正常导航的加速距离系数
        self.acc_move_coef = self.flash_sys.find_value("acc_move_coef")         # 搬运状态下的加速距离系数
        self.dec_coef = self.flash_sys.find_value("dec_coef")          # 减速距离系数
        self.move_v_max = 0.0     # 根据物体种类选择搬运速度
        self.find_line_v_max = self.flash_sys.find_value("find_line_v_max")  # 光电管寻找边界时的最大速度
        self.move_v_max_T = self.flash_sys.find_value("move_v_max_T")# type: int  # 搬运网球时的最大速度
        self.move_v_max_S = self.flash_sys.find_value("move_v_max_S")# type: int  # 搬运沙包时的最大速度
        self.move_v_max_B = self.flash_sys.find_value("move_v_max_B")# type: int  # 搬运玩具熊时的最大速度  

        self.waypoint_v = []  # type: list  # 目标速度列表

        # 路径规划相关变量
        self.target_x = 0.0         # type: float
        self.target_y = 0.0         # type: float
        self.target_v = 0.0           # type: float  # 目标速度
        self.v_peak = 0.0           # type: float  # 当前路径段的理论最高速度
        self.target_yaw = 0.0            # type: float
        self.turn_angle_target = 0.0     # type: float
        # 判断小车是否到达目标点的阈值
        self.final_threshold = self.flash_sys.find_value("final_threshold")  # type: float
        self.branch_threshold = self.flash_sys.find_value("branch_threshold")  # type: float
        self.finished_dist = 0.0    # type: float
        self.rest_dist = 0.0        # type: float
        self.usable_len = 0.0         # type: float  # 当前路径段的可用长度（扣除提前到达阈值后的剩余距离）
        self.segment_start_dist = 0.0   # 当前路径段的起始点与过渡点之间的距离
        self.d_acc = 0.0                # 当前路径段的加速距离
        self.d_dec = 0.0                # 当前路径段的减速距离
        # 用于搬运你物体时矫正里程计的误差
        self.error_x_T = self.flash_sys.find_value("error_x_T")       # type: float
        self.error_x_S = self.flash_sys.find_value("error_x_S")       # type: float
        self.error_x_B = self.flash_sys.find_value("error_x_B")       # type: float
        self.error_x = 0.0

        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 目标路径
        self.path = []      # type: list
        # 标志位
        self.arrive_flag = False            # type: bool  # 判断是否到达目标点标志位
        self.if_finish_turn = False         # type: bool  # 判断是否完成转角调整标志位
        self.if_send_path = False           # type: bool  # 判断是否向从车发送路径标志位
        self.if_finish_navigate = False     # type: bool  # 判断是否完成导航标志位
        self.if_near_line = False           # type: bool  # 判断是否接近边界标志位
   
    # 离线预计算速度表 (根据中继点附近曲率推算最佳过渡速度)
    def pre_calculate_profile(self, path: list):
        # 打开无刷负压风扇
        '''
        if self.current_object in ['R', 'P']:
            self.my_fan.set_fan_signal()
        else:
            self.my_fan.fan_off()
        '''
        self.path = path[:] # 复制路径列表
        self.path.insert(0, [self.my_car.x_current, self.my_car.y_current])  # 在路径前添加主车起点
        if len(self.path) < 2: return
        
        # 根据当前状态选择合适的加速距离系数
        if self.my_state.state == MOVE:
            self.acc_coef = self.acc_move_coef
        else:
            self.acc_coef = self.acc_normal_coef

        n = len(self.path)
        self.waypoint_v = [self.min_start_v] * n

        for i in range(1, n - 1):
            yaw_in = -math.atan2(-(self.path[i][0] - self.path[i-1][0]), self.path[i][1] - self.path[i-1][1]) * 180.0 / PI
            yaw_out = -math.atan2(-(self.path[i+1][0] - self.path[i][0]), self.path[i+1][1] - self.path[i][1]) * 180.0 / PI
            
            delta_yaw = abs(yaw_out - yaw_in)
            if delta_yaw > 180.0: delta_yaw = 360.0 - delta_yaw
            
            # 当航向角变化超过一定角度时，强制设定通过该点的最大速度
            speed_factor = max(0.0, 1.0 - (delta_yaw / 180.0))
            # 再缩放0.2系数，让速度更保守一些，增加过弯安全裕量
            self.waypoint_v[i] = self.min_start_v + speed_factor * (self.long_v_max - self.min_start_v) * 0.7

        # 【前向推演：固有加速距离限制】
        for i in range(0, n - 1):
            seg_dist = math.sqrt((self.path[i+1][0] - self.path[i][0])**2 + (self.path[i+1][1] - self.path[i][1])**2)
            # 同样扣除提前到达阈值，这部分距离不能用来加速
            threshold = self.final_threshold if (i == n - 2) else self.branch_threshold
            # 3.0为安全裕量
            usable_dist = max(0.0, seg_dist - threshold - 3.0)
            max_reachable_v = self.waypoint_v[i] + (usable_dist / self.acc_coef)
            if self.waypoint_v[i+1] > max_reachable_v:
                self.waypoint_v[i+1] = max_reachable_v

        # 【反向推演：固有刹车减速距离限制】
        for i in range(n - 2, -1, -1):
            seg_dist = math.sqrt((self.path[i+1][0] - self.path[i][0])**2 + (self.path[i+1][1] - self.path[i][1])**2)
            # 考虑最后一段和中间段不同的“提前到达”阈值作为刹车缓冲区的扣除
            threshold = self.final_threshold if (i == n - 2) else self.branch_threshold
            # 3.0为安全裕量
            safe_dist = max(0.0, seg_dist - threshold - 3.0)
            max_safe_v = self.waypoint_v[i+1] + (safe_dist / self.dec_coef)
            if self.waypoint_v[i] > max_safe_v:
                self.waypoint_v[i] = max_safe_v

        self.aimed_point_index = 0
        self.if_finish_navigate = False
        # 计算第一段路径的加减速参数
        self.plan_acc_dec()
        self.target_v = self.waypoint_v[0]
        # 初始目标角直接看向第一个点
        self.target_yaw = -math.atan2(-(self.path[1][0] - self.path[0][0]), self.path[1][1] - self.path[0][1]) * 180.0 / PI
        # 固定系数（负压状态下）
        self.my_car.alpha_x = 0.939524
        self.my_car.alpha_y = 0.915747


    # 根据当前过渡距离计算加减速距离
    def plan_acc_dec(self):
        # 修正：应该测量到下一个目标点(aimed_point_index + 1)的实物距离作为这段的总长
        target_pt = self.path[self.aimed_point_index + 1]
        self.segment_start_dist = math.sqrt((target_pt[0] - self.my_car.x_current)**2 + (target_pt[1] - self.my_car.y_current)**2)

        v_start = self.waypoint_v[self.aimed_point_index]
        v_end = self.waypoint_v[self.aimed_point_index + 1]

        # 修正：判断当前段的死区阈值，将S型的终点提前，得到真正的加减速“可用空间”
        is_last_segment = (self.aimed_point_index == len(self.path) - 2)
        threshold = self.final_threshold if is_last_segment else self.branch_threshold
        self.usable_len = max(0.01, self.segment_start_dist - threshold - 3.0) # 3.0为安全裕量

        if self.usable_len <= 0.1:
            self.v_peak = v_end
            self.d_acc = 0.0
            self.d_dec = 0.0
            return

        v_cruise = float(self.long_v_max)

        # 基于绝对速度变化量反算理论需要的加减速物理距离
        d_acc_req = self.acc_coef * max(0.0, v_cruise - v_start)
        d_dec_req = self.dec_coef * max(0.0, v_cruise - v_end)
        
        # 空间是否充裕判定
        if d_acc_req + d_dec_req > self.usable_len:
            # 空间不足，触发削峰逻辑
            if self.acc_coef + self.dec_coef > 0:
                self.v_peak = (self.usable_len + self.acc_coef * v_start + self.dec_coef * v_end) / (self.acc_coef + self.dec_coef)
            else:
                self.v_peak = v_cruise
            self.v_peak = max(self.v_peak, max(v_start, v_end))
        else:
            # 空间极其充裕，直接锁死在最高速阶段巡航
            self.v_peak = v_cruise

        # 平滑加减速阶段衔接区域
        self.d_acc = self.acc_coef * max(0.0, self.v_peak - v_start)
        self.d_dec = self.dec_coef * max(0.0, self.v_peak - v_end)
        if self.d_acc + self.d_dec > self.usable_len and (self.d_acc + self.d_dec) > 0:
            scale = self.usable_len / (self.d_acc + self.d_dec)
            self.d_acc *= scale
            self.d_dec *= scale

    def _calculate_position_s_curve(self):
        # 多项式平滑插值函数 Gentle-Smoothstep
        def smoothstep(t):
            t = max(0.0, min(1.0, t))
            # k 是融合系数，范围 0 到 1
            # k = 0 就是原来的三次方程 (中间最陡)
            # k = 1 就是纯匀速直线 (没有加减速过渡)
            # 推荐使用 0.3 ~ 0.5 之间，这里默认用 0.4
            k = 0.5
            cubic = 3 * (t ** 2) - 2 * (t ** 3)
            return k * t + (1 - k) * cubic

        v_start = self.waypoint_v[self.aimed_point_index]
        v_end = self.waypoint_v[self.aimed_point_index + 1]

        v_cruise = self.long_v_max
        # 在搬运状态下，小车如果接近边界需要降低速度便于光电管寻线
        if self.my_state.state == MOVE:
            near_line_threshold = 20.0  # 距离边界的阈值，单位：cm
            
            if self.my_car.y_current >= 240.0 - near_line_threshold:
                ratio = (240.0 - self.my_car.y_current) / near_line_threshold
                
                # 使用平方映射，使得减速更加剧烈，在较远处就开始显著降速
                ratio = max(0.0, min(1.0, ratio))
                ratio = ratio * ratio * ratio
            elif self.my_car.y_current <= near_line_threshold:
                ratio = self.my_car.y_current / near_line_threshold

                # 使用平方映射，使得减速更加剧烈，在较远处就开始显著降速
                ratio = max(0.0, min(1.0, ratio))
                ratio = ratio * ratio * ratio
            else:
                ratio = 1.0
                
            v_cruise = self.find_line_v_max + (self.move_v_max - self.find_line_v_max) * ratio

        # s 直接基于我们之前算出的 usable_len 限制
        s = self.segment_start_dist - self.rest_dist
        s_usable = max(0.0, min(s, self.usable_len))  # 强制束缚在可用区间内

        if s_usable <= self.d_acc:
            if self.d_acc <= 1e-3: v_out = self.v_peak
            else: v_out = v_start + (self.v_peak - v_start) * smoothstep(s_usable / self.d_acc)
        elif s_usable >= self.usable_len - self.d_dec:
            if self.d_dec <= 1e-3: v_out = v_end
            else:
                s_dec = s_usable - (self.usable_len - self.d_dec)
                v_out = self.v_peak + (v_end - self.v_peak) * smoothstep(s_dec / self.d_dec)
        else:
            v_out = self.v_peak
            
        # 这里改为输出 float，有助于你的底层PID或者跟随器能取得更平滑的参考速度
        return max(float(self.min_start_v), min(float(v_cruise), float(v_out)))
        
    # 实时导航执行函数
    def navigate_step(self):
        """
        实时执行：包含闭环航向解算与速度规划
        """
        # 更新小车当前位置
        car_x = self.my_car.x_current
        car_y = self.my_car.y_current

        # 更新小车到起点的距离
        self.finished_dist = math.sqrt((self.path[0][0] - car_x)**2 + (self.path[0][1] - car_y)**2)
        
        if self.aimed_point_index >= len(self.path) - 1:
            self.target_v = 0
            return self.target_v, self.target_yaw

        target_pt = self.path[self.aimed_point_index + 1]
        self.rest_dist = math.sqrt((target_pt[0] - car_x)**2 + (target_pt[1] - car_y)**2)
        
        # =======================================================
        # 1. 速度控制模块
        # =======================================================
        self.target_v = self._calculate_position_s_curve()

        # =======================================================
        # 2. 闭环航向角解算模块
        # =======================================================
        self.target_yaw = -math.atan2(-(target_pt[0] - car_x), target_pt[1] - car_y) * 180.0 / PI
        
        # 输出限幅在 [-180, 180] 内
        if self.target_yaw > 180: self.target_yaw -= 360
        elif self.target_yaw < -180: self.target_yaw += 360
        
        # =======================================================
        # 4. 到达判断
        # =======================================================
        is_last_segment = (self.aimed_point_index == len(self.path) - 2)

        if not is_last_segment and self.rest_dist <= self.branch_threshold:
            self.aimed_point_index += 1
            # 计算当前路径的加减速参数
            self.plan_acc_dec() 
        elif is_last_segment and self.rest_dist <= self.final_threshold:
            # 到达目标点关闭负压风扇
            # self.my_fan.fan_off()
            # 重置导航标志位
            self.if_finish_navigate = True
            self.stop()

    # 停止小车运动
    def stop(self):
        self.target_v = 0
        self.target_yaw = 0.0

    # 按照传入路径及进行惯性导航
    # 如果传入的目标转角不为none，则进行转角规划，否则不进行转角规划（用于路径点之间的过渡）
    def navigate(self, path = None, target_turn_angle = None):
        # 先进行转角调整使得路径规划与导航更稳定
        if self.if_finish_navigate == False:
            if self.if_finish_turn == False:
                if target_turn_angle is not None:
                    self.target_v = 0
                    self.turn_angle_target = target_turn_angle
                    # 通过角度环限幅削弱转角调整的力度，帮助小车稳定完成转角调整
                    self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.low_pwmout_limitmax
                    
                    # 在未完成转角调整时，持续进行转角调整
                    diff = abs(self.turn_angle_target - self.my_car.now_yaw * 180 / PI)
                    if diff > 180.0:
                        diff = 360.0 - diff

                    if diff <= 0.9:
                        # 若不传入路径则当前导航已完成
                        if path is None:
                            self.if_finish_navigate = True
                        else:
                            self.if_finish_turn = True
                            self.pre_calculate_profile(path)
                        # 恢复正常的角度环限幅
                        self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax
                else:
                    self.turn_angle_target = self.my_car.now_yaw * 180.0 / PI
                    if path is None:
                        # 处理传入路径和角度都为空的情况
                        self.if_finish_navigate = True
                    else:
                        # 如果没有目标转角，直接认为转角调整完成
                        self.if_finish_turn = True  
                        self.pre_calculate_profile(path)
                    
                    # 没有进行转角调整也要恢复原状
                    self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax
                    return 
            else:
                self.navigate_step()
        else:
            self.stop()
            self.if_finish_turn = False
            self.aimed_point_index = 0
            self.path.clear()
                
    # 重置导航及速度规划相关标志位
    def reset_navigate(self):
        self.target_v = 0.0
        self.if_finish_turn = False
        self.if_finish_navigate = False
        self.aimed_point_index = 0
        self.path.clear()

    # 重置小车导航姿态角
    def reset_navigate_angle(self):
        self.turn_angle_target = self.my_car.now_yaw * 180.0 / PI