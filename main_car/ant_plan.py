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
class PlanData:
    def __init__(self, flash_sys):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 地图固定点坐标
        # fixed_point[0]为主车起点，[1]为从车在下边沿的待命区，[2]为从车在上边沿的待命区
        self.fixed_point = [[35.0, -15.0], [160.0, 20.0], [160.0, 220.0]]  # type: list
        
        # 中心物品摆放的矩形区域
        self.center_rect = [[110.0, 70.0], [110.0, 170.0], [210.0, 70.0], [210.0, 170.0]] 

        # 路径规划相关常量
        self.FIELD_W = 320.0  # 地图宽度
        self.FIELD_H = 240.0  # 地图高度
        self.OBSTACLE_R = 15.0  # 圆形障碍物默认半径 (直径 30cm -> 半径 15cm)
        self.CUBE_LENTH = 10.0   # 立方体障碍物长度
        self.CUBE_WIDE = 5.0  # 立方体障碍物宽度
        self.INF = 1000000000.0  # 无穷大
        self.SAFE_MARGIN = 15.0  # 小车安全裕量 (质点膨胀半径)

        self.rectangle_obstacles = self.create_expanded_rect(160.0, 120.0, 100.0, 100.0)  # 中心禁区矩形障碍物（已膨胀）
        self.cube = self.flash_sys.find_value("cube_obstacles")  # 立方体障碍物中心坐标列表（未膨胀）
        self.circle = self.flash_sys.find_value("circle")  # 信标障碍物中心坐标列表
        # 将矩形障碍区进行膨胀（先后顺序不能改变）
        self.rectangles = [self.create_expanded_rect(x[0], x[1], self.CUBE_LENTH, self.CUBE_WIDE) for x in self.cube]
        self.rectangles.append(self.rectangle_obstacles)  # 将中心禁区矩形障碍物加入矩形障碍物列表

        # 硬写物品路径规划（每次发车前进行硬写路径规划）
        # rogue_planning[0]记录下边沿的物体，rogue_planning[1]记录上边沿的物体
        # y坐标靠近下边沿:70，中等:85，靠近中心:100    
        # 靠近上边沿:170，中等:155，靠近中心:140 
        # T是网球，S是红沙包，E是蓝沙包，W是白熊，B是棕熊
        # 示例：[(160.0, 85.0), 'E', [x, x]]
        self.rogue_planning = self.flash_sys.find_value("rogue_planning")  # type: list 
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

    # 辅助函数：由于原代码矩形检测未膨胀，这里手动对外扩充矩形顶点
    def create_expanded_rect(self, x_center, y_center, width, height):
        hw = width / 2.0 + self.SAFE_MARGIN
        hh = height / 2.0 + self.SAFE_MARGIN
        return [
            (x_center - hw, y_center - hh),
            (x_center + hw, y_center - hh),
            (x_center + hw, y_center + hh),
            (x_center - hw, y_center + hh)
        ]

# 路径规划类
class PathPlan:
    def __init__(self, plan_data: PlanData):
        self.Data = plan_data

    # 路径规划主函数
    def plan_path(self, circle, retangle, x0, y0, x1, y1):
        circles = self._normalize_points(circle)
        rects = self._normalize_rectangles(retangle)
        start = (float(x0), float(y0))
        end = (float(x1), float(y1))
        
        # 物理障碍物加安全裕量的总膨胀半径
        block_r = float(self.Data.OBSTACLE_R) + float(self.Data.SAFE_MARGIN)

        # 验证起点和终点
        if not self._point_valid(start, circles, rects, block_r):
            return []
        if not self._point_valid(end, circles, rects, block_r):
            return []
        # 检查直连
        if self._line_valid(start, end, circles, rects, block_r):
            return [[x0, y0], [x1, y1]]

        # 初始化节点列表
        nodes = [start, end]
        # 添加圆形中继点 (核心修改部分)
        self._add_circle_nodes_fixed(nodes, circles, block_r)
        # 添加矩形中继点
        self._add_rectangle_nodes(nodes, rects, self.Data.SAFE_MARGIN)
        # 剔除无效点和重复点            
        nodes = self._unique_valid_nodes(nodes, circles, rects, block_r)

        n = len(nodes)
        dist = [self.Data.INF] * n
        prev = [-1] * n
        used = [False] * n
        dist[0] = 0.0

        # Dijkstra 算法实现
        for _ in range(n):
            u = -1
            best = self.Data.INF
            for i in range(n):
                if (not used[i]) and dist[i] < best:
                    best = dist[i]
                    u = i
            if u < 0 or u == 1:
                break
            used[u] = True

            for v in range(n):
                if used[v] or v == u:
                    continue
                if self._line_valid(nodes[u], nodes[v], circles, rects, block_r):
                    w = self._distance(nodes[u], nodes[v])
                    if dist[u] + w < dist[v]:
                        dist[v] = dist[u] + w
                        prev[v] = u

        if prev[1] < 0:
            return []

        # 重建路径
        path = []
        i = 1
        while i >= 0:
            path.append(nodes[i])
            i = prev[i]
        path.reverse()
        return self._path_to_list(self._smooth_path(path, circles, rects, block_r))

    # 初始化圆形障碍物列表
    def _normalize_points(self, points):
        out = []
        if not points: return out
        for p in points:
            if len(p) >= 2:
                out.append((float(p[0]), float(p[1])))
        return out

    # 初始化矩形障碍物列表
    def _normalize_rectangles(self,rects):
        if not rects: return []
        if len(rects) == 4 and len(rects[0]) == 2 and isinstance(rects[0][0], (int, float)):
            return [self._sort_polygon(self._normalize_points(rects))]
        out = []
        for rect in rects:
            pts = self._normalize_points(rect)
            if len(pts) >= 4:
                out.append(self._sort_polygon(pts))
        return out

    # 对多边形顶点进行排序，确保顺时针或逆时针顺序
    def _sort_polygon(self, poly):
        cx, cy = 0.0, 0.0
        for p in poly: cx += p[0]; cy += p[1]
        cx /= len(poly); cy /= len(poly)
        items = []
        for p in poly: items.append((math.atan2(p[1] - cy, p[0] - cx), p))
        items.sort()
        out = []
        for item in items: out.append(item[1])
        return out

    # 围绕圆心生成 8 个中继点
    def _add_circle_nodes_fixed(self, nodes, circles, block_r):
        num_points = 8
        angle_step = 2.0 * math.pi / num_points
        
        # 距离计算必须使得弦的距离中心距离大于 block_r 才能被视为有效线段
        # d * cos(angle_step / 2) > block_r  ==>  d = block_r / cos(angle_step / 2) + margin
        node_radius = block_r / math.cos(angle_step / 2.0) + 1.0
        
        for c in circles:
            for i in range(num_points):
                angle = i * angle_step
                # 根据角度计算中继点坐标
                px = c[0] + math.cos(angle) * node_radius
                py = c[1] + math.sin(angle) * node_radius
                nodes.append((px, py))

    # 将矩形的四个角点添加为中继点
    def _add_rectangle_nodes(self, nodes, rects, margin):
        """原代码中的矩形节点生成逻辑"""
        d = float(margin) + 1.0
        for rect in rects:
            cx, cy = 0.0, 0.0
            for p in rect: cx += p[0]; cy += p[1]
            cx /= len(rect); cy /= len(rect)
            for p in rect:
                vx, vy = p[0] - cx, p[1] - cy
                l = math.sqrt(vx * vx + vy * vy)
                
                if l == 0.0: nodes.append(p)
                else: nodes.append((p[0] + vx / l * d, p[1] + vy / l * d))

    # 对所有生成的候选节点进行过滤和去重
    def _unique_valid_nodes(self, nodes, circles, rects, block_r):
        out = []
        for p in nodes:
            if not self._inside_field(p): continue
            if not self._point_valid(p, circles, rects, block_r): continue
            duplicate = True
            for q in out:
                if abs(p[0] - q[0]) < 0.001 and abs(p[1] - q[1]) < 0.001:
                    duplicate = False; break
            if duplicate: out.append(p)
        return out

    # 判断点p是否有效（不在任何障碍物内，并且在场地内）
    def _point_valid(self, p, circles, rects, block_r):
        if not self._inside_field(p): return False
        for c in circles:
            if self._distance(p, c) <= block_r: return False
        for rect in rects:
            if self._point_in_poly(p, rect): return False
        return True

    # 判断线段ab是否与任何障碍物相交（ab不穿过障碍物）
    def _line_valid(self, a, b, circles, rects, block_r):
        for c in circles:
            if self._dist_point_to_seg(c, a, b) <= block_r: return False
        for rect in rects:
            if self._segment_hits_poly(a, b, rect): return False
        return True

    # 判断点p是否在场地内
    def _inside_field(self, p):
        return p[0] >= 0.0 and p[0] <= self.Data.FIELD_W and p[1] >= 0.0 and p[1] <= self.Data.FIELD_H

    # 计算两点间距离
    def _distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    # 计算点p到线段ab的距离
    def _dist_point_to_seg(self, p, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        den = dx * dx + dy * dy
        if den < 1e-6: return self._distance(p, a)
        t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / den
        if t < 0.0: t = 0.0
        elif t > 1.0: t = 1.0
        return self._distance(p, (ax + t * dx, ay + t * dy))

    # 向量ab与ac的叉积
    def _cross(self, a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    # 判断a, b, p三点是否共线且p在a,b之间
    def _on_segment(self, a, b, p):
        return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and
                min(a[1], b[1]) <= p[1] <= max(a[1], b[1]) and
                abs(self._cross(a, b, p)) < 0.000001)

    # 判断ab, cd两线段是否相交
    def _seg_intersect(self, a, b, c, d):
        c1, c2 = self._cross(a, b, c), self._cross(a, b, d)
        c3, c4 = self._cross(c, d, a), self._cross(c, d, b)
        if c1 * c2 < 0.0 and c3 * c4 < 0.0: return True
        if abs(c1) < 0.000001 and self._on_segment(a, b, c): return True
        if abs(c2) < 0.000001 and self._on_segment(a, b, d): return True
        if abs(c3) < 0.000001 and self._on_segment(c, d, a): return True
        if abs(c4) < 0.000001 and self._on_segment(c, d, b): return True
        return False

    # 判断点p是否在多边形poly内
    def _point_in_poly(self, p, poly):
        inside = False; j = len(poly) - 1
        for i in range(len(poly)):
            pi, pj = poly[i], poly[j]
            if self._on_segment(pi, pj, p): return True
            if ((pi[1] > p[1]) != (pj[1] > p[1])):
                x = (pj[0] - pi[0]) * (p[1] - pi[1]) / (pj[1] - pi[1]) + pi[0]
                if p[0] < x: inside = not inside
            j = i
        return inside

    # 判断线段ab是否与多边形poly相交
    def _segment_hits_poly(self, a, b, poly):
        if self._point_in_poly(a, poly) or self._point_in_poly(b, poly): return True
        j = len(poly) - 1
        for i in range(len(poly)):
            if self._seg_intersect(a, b, poly[j], poly[i]): return True
            j = i
        return False

    # 判断路径是否需要平滑，如果需要则进行平滑处理
    def _smooth_path(self, path, circles, rects, block_r):
        if len(path) <= 2: return path
        out = [path[0]]; i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._line_valid(path[i], path[j], circles, rects, block_r): break
                j -= 1
            out.append(path[j]); i = j
        return out

    # 将返回的坐标点转换为列表形式
    def _path_to_list(self, path):
        return [[p[0], p[1]] for p in path]



class NavigationPlan:
    def __init__(self, flash_sys, plan_data: PlanData, math, car, state: StateMachine, order_manager, my_uart3, beep, art_protocol, sin_diff_fil, cos_diff_fil):
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