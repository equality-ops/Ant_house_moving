import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

# ── 路径规划状态标志位 ──
STATUS_OK            = 0   # 正常
STATUS_OUT_OF_BOUNDS = 1   # 检测到越界物体，已归类到最近格子
STATUS_CONFLICT      = 2   # 检测到同格冲突，已消解重分配
STATUS_GRID_FULL     = 3   # 格子全满，无法规划，plan 为空

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

    def plan_path(self, objects_input: List[Tuple[float, float, str]]) -> Tuple[List[Tuple[float, float, str, str]], int]:
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

        return plan, status


# ================= 自测验证 =================
if __name__ == "__main__":
    planner = PathPlanner()

    def print_result(plan, status: int, test_name: str):
        """根据 status 标志位打印测试结果"""
        status_names = {
            STATUS_OK:            "OK(正常)",
            STATUS_OUT_OF_BOUNDS: "OUT_OF_BOUNDS(越界归类)",
            STATUS_CONFLICT:      "CONFLICT(冲突消解)",
            STATUS_GRID_FULL:     "GRID_FULL(满格终止)",
        }
        print(f"\n{'█'*55}")
        print(f"  {test_name}")
        print(f"{'█'*55}")
        print(f"  → status={status} ({status_names.get(status, '?')}), plan步数={len(plan)}")
        for i, (cx, cy, kind, d) in enumerate(plan):
            dir_str = "上(260)" if d == 'U' else "下(-20)"
            print(f"    步骤{i+1}: {kind} 中心({cx:.1f},{cy:.1f}) → {dir_str}")

    # ─── 测试1: 正常情况 ───
    plan, status = planner.plan_path([
        (130.0, 90.0, 'T'),
        (190.0, 120.0, 'S'),
        (160.0, 155.0, 'B'),
        (120.0, 160.0, 'W'),
        (200.0, 80.0, 'E'),
    ])
    print_result(plan, status, "测试1: 正常情况")