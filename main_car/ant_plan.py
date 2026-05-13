import math

# 状态机制
class StateMachine:
    def __init__(self):
        # state 模式：
        self.READT_NAVIGATE = 0   # 准备导航状态
        self.NAVIGATE = 1       # 导航状态
        self.SCAN = 2           # 扫描状态
        self.SERVO = 3          # 视觉伺服状态
        self.ORBIT = 4          # 环绕状态
        self.MOVE = 5           # 搬运状态
        self.CALIBRATE = 6      # 校准状态
        self.RETURN = 7		    # 返回状态
        self.STOP = 8           # 停止状态
        
        self.if_move_easy_object = False   # 是否搬运过易搬运物体的标志位（搬运过易搬运物体后在返回起点时不避开矩形区域）
        self.state = self.NAVIGATE  # 初始状态为准备导航状态
        self.state_work = -1 # 阶段变量

# 小车位置循环链表
class CarPosition:
    def __init__(self):
        self.car_pos_list = ['D', 'R', 'U', 'L']  # 小车位置循环链表，顺时针记录小车在四条边的位置关系
        self.current_idx = 0  # 当前索引，初始时小车在下边沿（'D'）
        self.L = len(self.car_pos_list) # 链表长度

    # 向前或向后移动链表索引
    def move(self, step):
        return (self.current_idx + step) % self.L
    
    # 根据小车位置更新当前小车位置索引
    def update_idx(self, car_pos):
        self.current_idx = self.car_pos_list.index(car_pos)

    # 返回小车当前位置
    def get_position(self):
        return self.car_pos_list[self.current_idx]

# 路径和速度规划相关常量
class Plan_data:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 地图固定点坐标
        # fixed_point[0]为主车起点，[1]为从车在下边沿的待命区，[2]为从车在上边沿的待命区
        self.fixed_point = [[35.0, -15.0], [160.0, 20.0], [160.0, 220.0]]  # type: list
        # 为测试里程计方便
        # self.fixed_point = [[0.0, -0.0], [110.0, 50.0], [210.0, 190.0], [210.0, 50.0], [110.0, 190.0], [160.0, 20.0], [160.0, 220.0]]  # type: list
        # 矩形区域四角点坐标
        self.rectangle_corners = [[110.0, 70.0], [110.0, 170.0], [210.0, 70.0], [210.0, 170.0]] 

        # 硬写物品路径规划（每次发车前进行硬写路径规划）
        # rogue_planning[0]记录下边沿的物体，rogue_planning[1]记录上边沿的物体

        # y坐标靠近下边沿:70，中等:85，靠近中心:100    
        # 靠近上边沿:170，中等:155，靠近中心:140 
        # T是网球，S是红沙包，E是蓝沙包，W是白熊，B是棕熊

        # 示例：[(160.0, 85.0), 'E', [x, x]]
        self.rogue_planning = self.flash_sys.find_value("rogue_planning")  # type: list 
        self.obstacles = [item[0] for item in self.rogue_planning]  # type: list  # 障碍物坐标列表
        self.current_index = 0          # 当前搬运物体索引         
        self.moved_objects_num = 0      # 已搬运物体数量
        self.total_objects_num = len(self.rogue_planning)   # 需要搬运的物体总数

        # 已到达的目标点索引
        self.aimed_point_index = 0    # type: int
        # 当前避障路径中的目标点索引
        self.current_aimed_point_index = 0    # type: int
        # 时间计数器
        self.time_counter = 0          # type: int
        # 路径点切换时间阈值（用于过渡）
        self.plan_point_transition_T = self.flash_sys.find_value("plan_point_transition_T")
        # 长短距离标志
        self.LONG_DISTANCE = 1
        self.MID_DISTANCE = 2
        self.SHORT_DISTANCE = 3

class Plan:
    def __init__(self, flash_sys, plan_data: Plan_data, math, car, state: StateMachine, order_manager, my_uart3, beep, art_protocol, sin_diff_fil, cos_diff_fil):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入数学常量对象
        self.MATH = math
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
        # 注入主车正余弦滑动平均滤波器对象
        self.sin_diff_fil = sin_diff_fil
        self.cos_diff_fil = cos_diff_fil

        # 速度规划相关常量
        self.min_start_v = self.flash_sys.find_value("min_start_v")           # type: int  # 最小制动速度
        self.dead_zone_v = self.flash_sys.find_value("dead_zone_v")           # type: int  # 死区启动速度
        self.long_v_max = self.flash_sys.find_value("long_v_max")             # type: int  # 长距离时的最大速度
        self.short_v_max = self.flash_sys.find_value("short_v_max")           # type: int  # 短距离时的最大速度
        self.transit_v = self.flash_sys.find_value("transit_v")               # type: int  # 过渡阶段速度
        self.move_v_max = 0   # 根据物体种类选择搬运速度                        # type: int  # 搬运物品时的最大速度
        self.move_v_max_T = self.flash_sys.find_value("move_v_max_T")         # type: int  # 搬运网球时的最大速度
        self.move_v_max_S = self.flash_sys.find_value("move_v_max_S")         # type: int  # 搬运沙包时的最大速度
        self.move_v_max_B = self.flash_sys.find_value("move_v_max_B")         # type: int  # 搬运玩具熊时的最大速度
        self.scan_v_max = self.flash_sys.find_value("scan_v_max")             # type: int  # 扫描时的最大速度
        self.BOOST = 1                  # type: int  # 死区启动标志位
        self.TRANSIT = 2                # type: int  # 过渡阶段标志位
        self.DEC = 3                    # type: int  # 减速阶段标志位
        self.STOP = 4                   # type: int  # 停止标志位
        self.v_target = 0               # type: int  # 目标速度
        # 速度规划阶段变量
        self.v_max = 0                  # type: int    # 本次移动规划的最大速度
        self.j = 0                      # type: float  # 加加速度    
        self.dec_distance = 0.0         # type: float  # 减速距离
        self.stage = self.STOP          # type: int    # 速度规划阶段标志位
        self.finish_building = False    # type: int    # 检验减速速度表是否构建完成的标志位
        # 死区启动相关变量
        self.elapsed_time = 0           # type: int   # 死区启动已用时间计数器
        self.boost_duration = 0         # type: int   # 死区启动持续时间计数器
        self.boost_time_threshold = 60  # type: int   # 死区启动时间阈值
        self.dec_speed_index = 0        # type: int   # 减速速度表索引
        # 路径规划相关变量
        self.ideal_target_x = 0.0        # type: float
        self.ideal_target_y = 0.0        # type: float
        self.real_target_x = 0.0         # type: float
        self.real_target_y = 0.0         # type: float
        self.target_yaw = 0.0            # type: float
        self.turn_angle_target = 0.0     # type: float
        self.error_correct_x = 0.0       # type: float
        self.error_correct_y = 0.0       # type: float
        self.calibrate_angle = 0.0       # type: float # 摄像头识别到的矫正角度
        self.navigate_counter = 0        # type: int   # 导航用时计数器
        # 判断小车是否到达目标点的阈值
        self.plan_arrive_threshold = self.flash_sys.find_value("plan_arrive_threshold")  # type: float
        self.total_distance = 0.0       # type: float
        self.finished_distance = 0.0    # type: float
        # 用于搬运你物体时矫正里程计的误差
        self.error_x_T = self.flash_sys.find_value("error_x_T")       # type: float
        self.error_x_S = self.flash_sys.find_value("error_x_S")       # type: float
        self.error_x_B = self.flash_sys.find_value("error_x_B")       # type: float
        self.error_x = 0.0
        # 到终点的剩余距离
        self.rest_distance = 0.0        # type: float
        # 当前与下一避障目标点的距离
        self.current_rest_dis = 0.0     # type: float
        # 到过度点的剩余距离
        self.rest_transition_distance = 0.0       # type: float
        # 目标路径
        self.path_points = []      # type: list
        # 当前路径（用于避开矩形区域）
        self.current_path = []     # type: list
        # 距离长短标志位
        self.dis_flag = None
        # 标志位
        self.arrive_flag = False            # type: bool  # 判断是否到达目标点标志位
        self.if_pass_transit_point = False  # type: bool  # 判断是否到达过渡点标志位
        self.transition_flag = False        # type: bool  # 判断是否过渡完成标志位
        self.if_finish_turn = False         # type: bool  # 判断是否完成转角调整标志位
        self.if_send_path = False           # type: bool  # 判断是否向从车发送路径标志位
        self.if_set_path = False            # type: bool  # 判断是否设置路径标志位
        self.finish_navigate = False        # type: bool  # 判断是否完成导航标志位
        self.if_elude = False               # type: bool  # 判断是否避障标志位
    
    def _is_line_clear(self, x1, y1, x2, y2, rl, rt, rr, rb):
        """判断线段 (x1,y1)-(x2,y2) 是否不穿过矩形 (rl, rb, rr, rt)"""
        # 1. 如果起点或终点在矩形内，判定为不安全
        if rl < x1 < rr and rb < y1 < rt: return False
        if rl < x2 < rr and rb < y2 < rt: return False
        
        # 2. 判断线段是否与矩形的四条边相交
        # 这里为了极致性能，可以使用简化版的线段相交判定
        # 如果线段的包围盒完全在矩形某一边，则一定不相交
        lines = [((rl, rb), (rl, rt)), ((rl, rt), (rr, rt)), 
                ((rr, rt), (rr, rb)), ((rr, rb), (rl, rb))]
        
        for p3, p4 in lines:
            if self._intersect(x1, y1, x2, y2, p3[0], p3[1], p4[0], p4[1]):
                return False
        return True

    def _intersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        """利用叉乘判断两条线段是否相交"""
        def ccw(ax, ay, bx, by, cx, cy):
            return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)
        return ccw(x1, y1, x3, y3, x4, y4) != ccw(x2, y2, x3, y3, x4, y4) and \
            ccw(x1, y1, x2, y2, x3, y3) != ccw(x1, y1, x2, y2, x4, y4)
    
    # 构建减速速度表
    def build_dec_speed_list(self, i):
        if self.finish_building == False:
            # 计算加加速度
            temp_dec_distance = self.dec_distance / self.my_car.speed_conversion_gamma
            self.j = (self.v_max ** 3) / (temp_dec_distance ** 2) 
            # 计算减速总时间
            if self.j == 0:
                self.half_time = 0
            else:
                self.half_time = math.sqrt(self.v_max / self.j)
            self.total_time = 2 * self.half_time
            # 计算减速距离对应的速度点个数
            self.dec_lenth = int(temp_dec_distance) * 10 + 1
            # 将标志位设为True
            self.finish_building = True
        else:
            i = i / self.dec_lenth * self.total_time
            if i >= self.total_time / 2:
                v = int(-0.5 * self.j * (i ** 2) + 2 * self.j * i * self.half_time - self.j * (self.half_time ** 2))
            else:
                v = int(0.5 * self.j * (i ** 2))
            return v

    # 速度规划函数
    def planning_speed(self):
        if self.arrive_flag == False:
            if self.stage == self.STOP:
                self.stage = self.BOOST
            elif self.stage == self.BOOST:
                self.elapsed_time += 1
                if self.elapsed_time <= self.boost_time_threshold:
                    # 计算目标速度
                    self.v_target = self.dead_zone_v + int(((self.elapsed_time / self.boost_time_threshold) ** 2) * (self.v_max - self.dead_zone_v))
                else:
                    self.v_target = self.v_max
                    self.stage = self.TRANSIT
                    self.elapsed_time = 0
            elif self.stage == self.TRANSIT:
                if self.current_rest_dis < 30.0 and self.if_pass_transit_point == False and self.my_state.state != self.my_state.MOVE:
                    self.v_target = int(self.current_rest_dis/ 20.0 * (self.v_max - self.transit_v) + self.transit_v)
                else:
                    if self.v_target < self.v_max:
                        if self.elapsed_time <= self.boost_time_threshold:
                        # 缓慢恢复到巡航速度
                            self.v_target = self.transit_v + int(((self.elapsed_time / self.boost_time_threshold) ** 2) * (self.v_max - self.transit_v))
                            self.elapsed_time += 1
                        else:
                            self.v_target = self.v_max
                            self.elapsed_time = 0
                    else:
                        self.v_target = self.v_max
                        self.elapsed_time = 0   
                if self.rest_distance < self.dec_distance:
                    self.stage = self.DEC
            elif self.stage == self.DEC:
                if self.rest_distance < self.dec_distance:
                    self.dec_speed_index = int((self.rest_distance / self.dec_distance) * self.dec_lenth)
                    self.v_target = self.build_dec_speed_list(self.dec_speed_index)
                    
                if self.v_target <= self.min_start_v:
                    self.v_target = self.min_start_v
                    self.dec_speed_index = 0
        else:
            self.v_target = 0
            self.elapsed_time = 0
            self.stage = self.STOP
            self.finish_building = False

    # 矩形区域避障函数
    def path_planning(self, target_x: float, target_y: float):
        # 1. 基础坐标
        x1, y1 = self.my_car.x_current, self.my_car.y_current
        x2, y2 = target_x, target_y
        
        # 2. 矩形膨胀 (考虑车体半径 R + 安全余量)
        # 假设 self.plan_data.rectangle_corners 顺序为: [左下, 左上, 右上, 右下]
        R = self.my_car.car_radius + 5  # 这里的 5 是安全间隙
        r_left = self.plan_data.rectangle_corners[0][0] - R
        r_right = self.plan_data.rectangle_corners[2][0] + R
        r_bottom = self.plan_data.rectangle_corners[0][1] - R
        r_top = self.plan_data.rectangle_corners[1][1] + R
        
        # 3. 快速相交判定 (AABB Check)
        # 如果路径的包围盒与矩形包围盒不重叠或者起点或终点在矩形框内，直接返回空列表（直接前往目标点）
        if (max(x1, x2) < r_left or min(x1, x2) > r_right or
            max(y1, y2) < r_bottom or min(y1, y2) > r_top) or\
            (r_left < x1 < r_right and r_bottom < y1 < r_top) or (r_left < x2 < r_right and r_bottom < y2 < r_top):
            return []

        # 4. 精确判定：使用叉乘判断线段是否穿过矩形
        # 如果起点或终点已经在矩形内部，或者线段确实穿过矩形边缘
        if not self._is_line_clear(x1, y1, x2, y2, r_left, r_top, r_right, r_bottom):
            
            # 5. 寻找最优中继点 (膨胀后的四个角点)
            inflated_corners = [
                (r_left, r_bottom), (r_left, r_top), 
                (r_right, r_top), (r_right, r_bottom)
            ]
            fit_points_1 = []
            fit_points_2 = []
            current_idx = 0
            best_corner = None
            min_total_dist = float('inf')   # 无穷大
            
            for cx, cy in inflated_corners:
                if self._is_line_clear(x1, y1, cx, cy, r_left, r_top, r_right, r_bottom):
                    # 记录与起点连线不与矩形相交的角点
                    fit_points_1.append((cx, cy))

            for cx, cy in fit_points_1:
                if self._is_line_clear(cx, cy, x2, y2, r_left, r_top, r_right, r_bottom):
                    # 若当前角点与终点连线不与矩形相交则取当前角点为中继点
                    fit_points_2.append((cx, cy))
                    
            if len(fit_points_2) > 1:
                for cx, cy in fit_points_2:
                    # 计算路程总长: dist(A, Corner) + dist(Corner, B)
                    d = math.sqrt((cx-x1)**2 + (cy-y1)**2) + math.sqrt((x2-cx)**2 + (y2-cy)**2)
                    if d < min_total_dist:
                        min_total_dist = d
                        best_corner = (cx, cy)
            elif len(fit_points_2) == 1:
                best_corner = fit_points_2[0]
            else:
                else_points = [pt for pt in inflated_corners if pt not in fit_points_1]
                # 若fit_point_1中的所有中继点与终点都与矩形相交，再增加一个中继点完成路径规划
                for cx, cy in fit_points_1:
                    for ex, ey in else_points:
                        # 计算路径 (ex,ey)->终点 是否可行
                        if self._is_line_clear(ex, ey, x2, y2, r_left, r_top, r_right, r_bottom):
                            # 在else_points中找到一个可行的相邻中继点
                            if ex - cx <= 1e-6 or ey - cy <= 1e-6:
                                d = math.sqrt((cx-x1)**2 + (cy-y1)**2) + math.sqrt((ex-cx)**2 + (ey-cy)**2) + math.sqrt((x2-ex)**2 + (y2-ey)**2)
                                if d < min_total_dist:
                                    min_total_dist = d
                                    best_corner = (ex, ey)

            return [best_corner] if best_corner else []
        
        return []

    # 设置目标点坐标
    def set_target_point(self, x: float, y: float):
        # 重置当前路径和相关索引
        self.plan_data.current_aimed_point_index = 0
        
        # 当if_elude标志位为True时才进行避障
        if self.if_elude:
            # 进行避障路径规划
            self.current_path = self.path_planning(x, y)
        else:   
            self.current_path = []

        # 理想条件下的目标坐标
        self.ideal_target_x = x
        self.ideal_target_y = y

        # 根据当前规划好的避障路径来估计航向
        if len(self.current_path) > 0:
            dx = self.ideal_target_x - self.current_path[-1][0]
            dy = self.ideal_target_y - self.current_path[-1][1]
        else:
            dx = self.ideal_target_x - self.my_car.x_current
            dy = self.ideal_target_y - self.my_car.y_current
        
        blurry_yaw = -math.atan2(-dx, dy) * 180.0 / self.MATH.PI - self.my_car.now_yaw * 180.0 / self.MATH.PI  # 计算大致航向角，单位：度
        if blurry_yaw > 180.0:
            blurry_yaw -= 360.0
        elif blurry_yaw < -180.0:
            blurry_yaw += 360.0
        # 根据大致航向角选择合适的坐标修正量（解决因惯性造成的打滑问题）
        if blurry_yaw >= -30.0 and blurry_yaw < 30.0:
            self.error_correct_x = 0.0
            self.error_correct_y = 0.0
        elif blurry_yaw >= 30.0 and blurry_yaw < 60.0:
            self.error_correct_x = 0.0
            self.error_correct_y = 0.0
        elif blurry_yaw >= 60.0 and blurry_yaw < 120.0:
            self.error_correct_x = 0.0
            self.error_correct_y = 0.0
        elif blurry_yaw >= 120.0 and blurry_yaw < 150.0:
            self.error_correct_x = 0.0
            self.error_correct_y = -0.0
        elif blurry_yaw >= 150.0 and blurry_yaw <= 180.0 or blurry_yaw >= -180.0 and blurry_yaw < -150.0:
            self.error_correct_x = 0.0
            self.error_correct_y = -0.0
        elif blurry_yaw >= -150.0 and blurry_yaw < -120.0:
            self.error_correct_x = -0.0
            self.error_correct_y = -0.0
        elif blurry_yaw >= -120.0 and blurry_yaw < -60.0:
            self.error_correct_x = -0.0
            self.error_correct_y = -0.0
        elif blurry_yaw >= -60.0 and blurry_yaw < -30.0:
            self.error_correct_x = -0.0
            self.error_correct_y = 0.0
        
        # 实际条件下的目标坐标
        self.real_target_x = self.ideal_target_x + self.error_correct_x
        self.real_target_y = self.ideal_target_y + self.error_correct_y

        # 将终点加入避障路径
        self.current_path.append((self.real_target_x, self.real_target_y))

        # 实际距离坐标点的直线距离
        total_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)

        if self.my_state.state != self.my_state.MOVE:
            x_transit_dis = abs(self.my_car.x_current - self.current_path[self.plan_data.current_aimed_point_index][0])
            y_transit_dis = abs(self.my_car.y_current - self.current_path[self.plan_data.current_aimed_point_index][1])    

            # 依据到过渡点的距离计算里程计系数
            if x_transit_dis >= 50.0:
                self.my_car.alpha_x = 0.966702
            elif x_transit_dis >= 10.0:
                self.my_car.alpha_x = 1.0
            else:
                self.my_car.alpha_x = 1.0

            if y_transit_dis >= 50.0:
                self.my_car.alpha_y = 0.932782
            elif y_transit_dis >= 10.0:
                self.my_car.alpha_y = 0.955172
            else:
                self.my_car.alpha_y = 1.0
        else:
            self.my_car.alpha_x = 1.0
            self.my_car.alpha_y = 0.961538

        # 计算减速距离（长距离或者搬运、扫描模式时减速距离为20，短距离时为0且短距离时速度恒定）
        if total_distance >= 50.0 or self.my_state.state == self.my_state.MOVE or self.my_state.state == self.my_state.SCAN or self.my_state.state == self.my_state.RETURN:
            # 根据当前模式设置减速距离和加速时间阈值
            if self.my_state.state == self.my_state.MOVE:
                self.v_max = self.move_v_max
                self.boost_time_threshold = 40
                self.dec_distance = 0.5
            elif self.my_state.state == self.my_state.SCAN:
                self.v_max = self.scan_v_max
                self.boost_time_threshold = 30
                self.dec_distance = 10.0
            elif self.my_state.state == self.my_state.RETURN:
                # 回城模式时将速度调到最大以尽快返回起点
                self.v_max = self.long_v_max
                self.boost_time_threshold = 30
                self.dec_distance = 25.0
            else:
                # 当需要避障时放慢速度
                if len(self.current_path) > 1:
                    self.v_max = self.transit_v + 40
                else:
                    self.v_max = self.long_v_max
                self.boost_time_threshold = 60
                self.dec_distance = 30.0
            # 创建s型曲线减速速度表
            self.build_dec_speed_list(0)
            self.dis_flag = self.plan_data.LONG_DISTANCE
        # 中距离移动
        elif total_distance >= 10.0 and total_distance < 50.0:
            # 设置减速距离和加速时间阈值
            self.dec_distance = 9.0
            self.boost_time_threshold = 20
            self.v_max = self.short_v_max
            # 创建s型曲线减速速度表
            self.build_dec_speed_list(0)
            self.dis_flag = self.plan_data.MID_DISTANCE
        else:
            self.v_target = self.dead_zone_v
            self.dis_flag = self.plan_data.SHORT_DISTANCE

        self.if_pass_transit_point = False
        self.arrive_flag = False

    # 更新已完成和剩余距离并判断是否到达目标点
    def update_distance(self):
        if self.plan_data.current_aimed_point_index < len(self.current_path) - 1:
            self.current_rest_dis = math.sqrt((self.my_car.x_current - self.current_path[self.plan_data.current_aimed_point_index][0]) ** 2 + (self.my_car.y_current - self.current_path[self.plan_data.current_aimed_point_index][1]) ** 2)
            if self.current_rest_dis < 5.0:
                self.plan_data.current_aimed_point_index += 1
                x_transit_dis = abs(self.my_car.x_current - self.current_path[self.plan_data.current_aimed_point_index][0])
                y_transit_dis = abs(self.my_car.y_current - self.current_path[self.plan_data.current_aimed_point_index][1])    

                # 依据到过渡点的距离计算里程计系数
                if x_transit_dis >= 50.0:
                    self.my_car.alpha_x = 0.966702
                elif x_transit_dis >= 10.0:
                    self.my_car.alpha_x = 1.0
                else:
                    self.my_car.alpha_x = 1.0

                if y_transit_dis >= 50.0:
                    self.my_car.alpha_y = 0.932782
                elif y_transit_dis >= 10.0:
                    self.my_car.alpha_y = 0.955172
                else:
                    self.my_car.alpha_y = 1.0
        else:
            self.if_pass_transit_point = True

        self.rest_distance = math.sqrt((self.real_target_x - self.my_car.x_current) ** 2 + (self.real_target_y - self.my_car.y_current) ** 2)
        # 当剩余距离小于阈值时，推断小车已经到达目标点
        if self.rest_distance <= self.plan_arrive_threshold and self.if_pass_transit_point == True:
            self.arrive_flag = True
            self.transition_flag = False
            self.finished_distance = 0.0
            self.rest_distance = 0.0
            self.dec_distance = 0.0

        if self.dis_flag != self.plan_data.SHORT_DISTANCE:
            # 每次更新距离后进行速度规划计算
            self.planning_speed()

    # 计算目标航向角
    def compute_target_yaw(self, target_x, target_y):
        # 只有在需要避障时开启航向角滤波以平滑过渡避障点
        # if self.plan_data.current_aimed_point_index < len(self.current_path) - 1 and self.my_state.state == self.my_state.MOVE:
        if self.plan_data.current_aimed_point_index < len(self.current_path) - 1: 
            dx = self.sin_diff_fil.filtering(target_x - self.my_car.x_current)
            dy = self.cos_diff_fil.filtering(target_y - self.my_car.y_current)
        else:
            dx = target_x - self.my_car.x_current
            dy = target_y - self.my_car.y_current
        # 计算目标角度，单位：度（注意避免除以0）
        self.target_yaw = -math.atan2(-dx, dy) * 180.0 / self.MATH.PI        
            
    # 计算小车需要转向的角度（一般为0）
    def compute_turn_angle_target(self, turn_angle_target: float):
        self.turn_angle_target = turn_angle_target

    # 用于路径之间的过渡，保证小车平稳
    def path_transition(self):
        self.v_target = 0
        self.plan_data.time_counter += 1
        # 最终的过渡时间为 plan_point_transition_T * plan_calculate_T(单位：ms)
        if self.plan_data.time_counter >= self.plan_data.plan_point_transition_T:
            self.plan_data.time_counter = 0	
            self.transition_flag = True

    # 停止小车运动
    def stop(self):
        self.v_target = 0
        self.target_yaw = 0.0

    # 按照传入路径及进行惯性导航
    # 如果传入的目标转角不为none，则进行转角规划，否则不进行转角规划（用于路径点之间的过渡）
    def navigate(self, path = None, target_turn_angle = None, if_elude = None):
        # 先进行转角调整使得路径规划与导航更稳定
        if self.if_finish_turn == False and self.finish_navigate == False:
            if target_turn_angle is not None:
                self.v_target = 0
                self.turn_angle_target = target_turn_angle
                # 通过角度环限幅削弱转角调整的力度，帮助小车稳定完成转角调整
                self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.low_pwmout_limitmax
            else:
                self.turn_angle_target = self.my_car.now_yaw * 180.0 / self.MATH.PI
                self.if_finish_turn = True  # 如果没有目标转角，直接认为转角调整完成
                # 处理传入路径和角度都为空的情况
                if path is None:
                    self.finish_navigate = True
                    self.if_finish_turn = False
                self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax
                return 

            # 在未完成转角调整时，持续进行转角调整
            diff = abs(self.turn_angle_target - self.my_car.now_yaw * 180 / self.MATH.PI)
            if diff > 180.0:
                diff = 360.0 - diff
            if diff <= 0.9:
                self.if_finish_turn = True
                # 若不传入路径则当前导航已完成
                if path is None:
                    self.finish_navigate = True
                    self.if_finish_turn = False
                # 恢复正常的角度环限幅
                self.my_car.angle_pid.pwmout_limitmax = self.my_car.angle_pid.high_pwmout_limitmax

        if self.if_set_path == False and self.finish_navigate == False and self.if_finish_turn == True:
            # 路径初始化
            self.path_points = path if path is not None else []
            self.if_set_path = True
            if if_elude == 'Y':
                self.if_elude = True
            else:
                self.if_elude = False
            # 设置第一个目标点
            self.set_target_point(self.path_points[0][0], self.path_points[0][1])
            self.compute_target_yaw(self.current_path[self.plan_data.current_aimed_point_index][0], self.current_path[self.plan_data.current_aimed_point_index][1])  
        # 规划好路径后再进行导航
        if self.if_set_path == True:
            # 判断是否还有未到达的目标点
            if self.plan_data.aimed_point_index < len(self.path_points):
                # 判断是否到达下一个目标点
                if self.arrive_flag == False:
                    self.update_distance()
                    if self.arrive_flag == True:
                        # 到达目标点后，更新目标点索引
                        self.plan_data.aimed_point_index += 1
                        # 进行路径过渡
                        self.path_transition()   
                    else:
                        # 计算目标航向角
                        self.compute_target_yaw(self.current_path[self.plan_data.current_aimed_point_index][0], self.current_path[self.plan_data.current_aimed_point_index][1])       
                else:
                    # 判断此时是否完成路径过渡
                    if self.transition_flag == False:
                        self.path_transition()
                    else:
                        # 如果还有下一个目标点，设置下一个目标点坐标
                        if self.plan_data.aimed_point_index < len(self.path_points):
                            self.set_target_point(self.path_points[self.plan_data.aimed_point_index][0], self.path_points[self.plan_data.aimed_point_index][1])
                            # 计算目标航向角
                            self.compute_target_yaw(self.current_path[self.plan_data.current_aimed_point_index][0], self.current_path[self.plan_data.current_aimed_point_index][1])
                        else:
                            self.stop()
            else:
                self.stop()
                self.if_finish_turn = False
                self.navigate_counter = 0
                self.plan_data.aimed_point_index = 0
                self.dec_speed_index = 0
                self.path_points.clear()
                self.if_set_path = False
                self.transition_flag = False
                self.finish_navigate = True
                self.stage = self.STOP
                self.finish_building = False

    # 重置导航及速度规划相关标志位
    def reset_navigate(self):
        # 导航
        self.finish_navigate = False
        self.arrive_flag = False
        self.dec_speed_index = 0
        self.aimed_point_index = 0
        self.path_points.clear()
        self.if_set_path = False
        self.if_finish_turn = False
        self.transition_flag = False
        # 速度规划
        self.elapsed_time = 0
        self.stage = self.STOP
        self.finish_building = False

    # 几何避障算法
    def is_path_blocked(self, car_pos, target_pos, safety_margin=3.0):
        """
        判断从 car_pos 到 target_pos 的路径是否被 obstacles 遮挡
        car_pos: (x, y)
        target_pos: (x, y)
        obstacles: [(x, y, r), ...] 障碍物坐标和半径
        """
        # 障碍物半径
        obstacle_radius = 3.0 # type: float

        x1, y1 = car_pos
        x2, y2 = target_pos
        
        # 线段向量
        dx = x2 - x1
        dy = y2 - y1
        line_len_sq = dx*dx + dy*dy
        
        if line_len_sq == 0: return False # 起点终点重合

        for ox, oy in self.plan_data.obstacles:
            # 1. 计算障碍物到线段的投影比例 t
            # 公式：t = [(O-A) · (B-A)] / |B-A|^2
            t = ((ox - x1) * dx + (oy - y1) * dy) / line_len_sq
            
            # 2. 限制 t 在 [0, 1] 范围内，确保距离是在“线段”上
            if t < 0 or t > 1:
                continue
            
            # 3. 找到线段上距离障碍物最近的点坐标
            nearest_x = x1 + t * dx
            nearest_y = y1 + t * dy
            
            # 4. 计算欧几里得距离
            dist = math.sqrt((ox - nearest_x)**2 + (oy - nearest_y)**2)
            
            # 5. 碰撞判定：距离小于 障碍半径 + 车体半径 + 额外安全系数
            if dist < (self.my_car.car_radius + obstacle_radius + safety_margin):
                # 排除目标物体本身（防止把自己当成障碍）
                if math.sqrt((ox - x2)**2 + (oy - y2)**2) < 1: 
                    continue
                return True # 路径被挡
            
        return False # 路径畅通     
    