from micropython import const
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
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
PREDICT = const(10)       # 预测状态

# ── 路径规划状态标志位 ──
STATUS_OK            = const(0)   # 正常
STATUS_OUT_OF_BOUNDS = const(1)   # 检测到越界物体，已归类到最近格子
STATUS_CONFLICT      = const(2)   # 检测到同格冲突，已消解重分配
STATUS_GRID_FULL     = const(3)   # 格子全满，无法规划，plan 为空

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
        # fixed_point[0]为主车起点，fixed_point[1]为矩形框左下方顶点，fixed_point[2]为矩形框右上方顶点, 
        # fixed_point[3]为主车返回点, [4]为从车返回点
        self.fixed_point = [[35.0, -40.2], [95.0, 55.0], [225.0, 185.0], [15.0, -50.0], [35.0, -50.0]]  # type: list
        # 扫描路径
        self.scan_point = [[130.0, 55.0], [190.0, 55.0]]
        self.finished_num = 0    # 已完成搬运的物体数量      
        # 物体总数
        self.total_objects_num = self.flash_sys.find_value("total_objects_num")  

        gc.collect()

@dataclass
class Object:
    index: int
    x: float
    y: float
    kind: str
    grid_row: int = -1
    grid_col: int = -1
    snapped_x: float = 0.0
    snapped_y: float = 0.0

# plan_path 返回类型: (plan列表, status状态码)
# plan列表中每个元素为 (center_x, center_y, kind, direction)

class PathPlanner:
    """九宫格物体搬运路径规划器"""
    # 常量定义
    UP = 'U'
    DOWN = 'D'
    def __init__(self, 
                 box_left: float = 110.0, 
                 box_bottom: float = 70.0, 
                 box_right: float = 210.0, 
                 box_top: float = 170.0,
                 grid_rows: int = 3,
                 grid_cols: int = 3,
                 push_down_y: float = -20.0,
                 push_up_y: float = 260.0):
        
        # 场地边界
        self.box_left = box_left
        self.box_bottom = box_bottom
        self.box_right = box_right
        self.box_top = box_top
        
        # 网格配置
        self.rows = grid_rows
        self.cols = grid_cols
        self.cell_width = (box_right - box_left) / grid_cols
        self.cell_height = (box_top - box_bottom) / grid_rows
        
        # 搬运目标边界
        self.push_down_y = push_down_y
        self.push_up_y = push_up_y

        self.total_ob_info = []  # 所有目标物体信息
        self.status = STATUS_OK  # 当前规划状态码

    def _get_grid_pos(self, x: float, y: float) -> Tuple[int, int, float, float]:
        """O(1) 复杂度通过数学映射直接计算格子索引和中心点"""
        col = int((x - self.box_left) / self.cell_width)
        row = int((y - self.box_bottom) / self.cell_height)

        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))

        cx = self.box_left + (col + 0.5) * self.cell_width
        cy = self.box_bottom + (row + 0.5) * self.cell_height

        return row, col, cx, cy

    def _find_nearest_cell(self, x: float, y: float) -> Tuple[int, int, float, float]:
        """为越界物体查找距离最近的九宫格格子（按格子中心点欧氏距离）"""
        best = (0, 0, 0.0, 0.0)
        best_dist = float('inf')
        for row in range(self.rows):
            for col in range(self.cols):
                cx = self.box_left + (col + 0.5) * self.cell_width
                cy = self.box_bottom + (row + 0.5) * self.cell_height
                d = math.hypot(x - cx, y - cy)
                if d < best_dist:
                    best_dist = d
                    best = (row, col, cx, cy)
        return best

    def _is_outside_box(self, x: float, y: float) -> bool:
        """判断物体坐标是否在矩形框外"""
        return x < self.box_left or x > self.box_right or y < self.box_bottom or y > self.box_top

    def _find_nearest_empty_cell(self, x: float, y: float, occupancy: set) -> Optional[Tuple[int, int, float, float]]:
        """为被挤出的物体查找最近的空余格子，若无空余则返回 None"""
        best = None
        best_dist = float('inf')
        for row in range(self.rows):
            for col in range(self.cols):
                if (row, col) in occupancy:
                    continue
                cx = self.box_left + (col + 0.5) * self.cell_width
                cy = self.box_bottom + (row + 0.5) * self.cell_height
                d = math.hypot(x - cx, y - cy)
                if d < best_dist:
                    best_dist = d
                    best = (row, col, cx, cy)
        return best

    def plan_path(self, objects_input: List[Tuple[float, float, str]]):
        """核心规划算法（含越界归类、同格冲突消解、满格保护）

        返回 (plan, status):
            plan:   [(center_x, center_y, kind, direction), ...]  按搬运顺序排列
                    direction 为 'U'(上边界260) 或 'D'(下边界-20)
            status: STATUS_OK(0) / OUT_OF_BOUNDS(1) / CONFLICT(2) / GRID_FULL(3)
        """
        status = STATUS_OK
        objects: List[Object] = []
        grid_occupancy: Dict[Tuple[int, int], List[Object]] = {}

        # ===== Phase 1: 初始吸附 + 越界检测 =====
        for i, (x, y, kind) in enumerate(objects_input):
            kind = kind.upper()
            is_outside = self._is_outside_box(x, y)

            if is_outside:
                row, col, cx, cy = self._find_nearest_cell(x, y)
                status = STATUS_OUT_OF_BOUNDS
            else:
                row, col, cx, cy = self._get_grid_pos(x, y)

            obj = Object(index=i, x=x, y=y, kind=kind,
                         grid_row=row, grid_col=col, snapped_x=cx, snapped_y=cy)
            objects.append(obj)

            key = (row, col)
            grid_occupancy.setdefault(key, []).append(obj)

        # ===== Phase 2: 同格冲突检测与消解 =====
        occupied_cells: set = set(grid_occupancy.keys())

        for (row, col), objs in list(grid_occupancy.items()):
            if len(objs) <= 1:
                continue

            # 按距离格子中心排序，最近的保留
            cx = self.box_left + (col + 0.5) * self.cell_width
            cy = self.box_bottom + (row + 0.5) * self.cell_height
            objs.sort(key=lambda o: math.hypot(o.x - cx, o.y - cy))

            displaced = objs[1:]

            status = max(status, STATUS_CONFLICT)

            for obj in displaced:
                result = self._find_nearest_empty_cell(obj.x, obj.y, occupied_cells)

                if result is None:
                    # 情况3: 格子全满 → 不进行 plan
                    return [], STATUS_GRID_FULL

                new_row, new_col, new_cx, new_cy = result
                old_row, old_col = obj.grid_row, obj.grid_col

                obj.grid_row = new_row
                obj.grid_col = new_col
                obj.snapped_x = new_cx
                obj.snapped_y = new_cy

                grid_occupancy[(old_row, old_col)].remove(obj)
                grid_occupancy.setdefault((new_row, new_col), []).append(obj)
                occupied_cells.add((new_row, new_col))

        # ===== Phase 3: 路径规划 =====
        col_groups: Dict[int, List[Object]] = {c: [] for c in range(self.cols)}
        for obj in objects:
            col_groups[obj.grid_col].append(obj)

        plan: List[Tuple[float, float, str, str]] = []
        near_boundary = self.DOWN
        far_boundary = self.UP

        active_cols = [c for c in range(self.cols - 1, -1, -1) if col_groups[c]]

        for idx, col in enumerate(active_cols):
            col_objects = col_groups[col]
            
            # 按距离当前靠近边界的远近排序（近的优先）
            col_objects.sort(key=lambda o: o.grid_row if near_boundary == self.DOWN else -o.grid_row)

            for i, obj in enumerate(col_objects):
                is_last_in_col = (i == len(col_objects) - 1)

                if is_last_in_col:
                    direction = far_boundary
                    next_near, next_far = far_boundary, near_boundary
                else:
                    direction = near_boundary
                    next_near, next_far = near_boundary, far_boundary

                plan.append((obj.snapped_x, obj.snapped_y, obj.kind, direction))

                if is_last_in_col:
                    near_boundary, far_boundary = next_near, next_far

        # 确保全局最后一个动作是向下 (DOWN)
        if plan:
            last = plan[-1]
            if last[3] != self.DOWN:
                plan[-1] = (last[0], last[1], last[2], self.DOWN)

        self.total_ob_info = plan
        self.status = status
    
# 导航规划类
class NavigationPlan:
    def __init__(self, flash_sys, plan_data: PlanData, fan, car, state: StateMachine, order_manager, my_uart3, beep, art_protocol):
        # 注入flash系统对象
        self.flash_sys = flash_sys
        # 注入路径规划数据对象
        self.plan_data = plan_data
        # 注入无刷负压
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
        self.scan_rate = self.flash_sys.find_value("scan_rate")      # type: int  # 扫描状态下的速度降低比例
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
        self.if_finish_navigate = False              # type: bool  # 判断是否完成导航标志位
        self.if_second_verify = False              # type: bool  # 判断是否进行第二次验证视觉

    # 离线预计算速度表 (根据中继点附近曲率推算最佳过渡速度)
    def pre_calculate_profile(self, path: list):
        # 打开无刷负压风扇
        """
        if self.current_object in ['R', 'P']:
            self.my_fan.set_fan_signal()
        else:
            self.my_fan.fan_off()
        """    
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
            # 我加负压了，给我拉满过弯速度
            self.waypoint_v[i] = self.min_start_v + speed_factor * (self.long_v_max - self.min_start_v) * 0.5

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
        self.my_car.alpha_x = 0.949102
        self.my_car.alpha_y = 0.950803
        

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

        if self.my_state.state == MOVE:
            v_cruise = self.move_v_max
        elif self.my_state.state == SCAN:
            v_cruise = self.long_v_max * self.scan_rate
        else:
            v_cruise = self.long_v_max

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
                
            v_target = self.find_line_v_max + (self.move_v_max - self.find_line_v_max) * ratio
            # 在搬运模式下为保证加速阶段一致设置恒定速度
            return v_target
        elif self.my_state.state == SCAN:
            # 在经过第二次视觉验证后减速预测目标点位
            if self.if_second_verify:
                v_cruise = self.find_line_v_max
            else:
                v_cruise = self.long_v_max * self.scan_rate  # 扫描状态下的巡航速度降低为长距离最大速度的指定比例
        else:
            v_cruise = self.long_v_max

        # 调试输出当前巡航速度
        # self.my_uart3.write(f"v: {v_cruise}, object: {self.current_object}, type: {type(self.current_object)}\n")  

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
            self.aimed_point_index += 1
            # 清空上一次小车速度
            self.my_car.clear_last_car_speed()
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

                    if diff <= 1.5:
                        # 若不传入路径则当前导航已完成
                        if path is None:
                            self.if_finish_navigate = True
                        else:
                            self.if_finish_turn = True
                            self.pre_calculate_profile(path)
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
        self.finished_dist = 0.0
        self.aimed_point_index = 0
        self.path.clear()

    # 重置小车导航姿态角
    def reset_navigate_angle(self):
        self.turn_angle_target = self.my_car.now_yaw * 180.0 / PI