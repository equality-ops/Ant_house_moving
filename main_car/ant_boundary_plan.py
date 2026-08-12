import math
import gc
from array import array

# The boundary planner only needs axis-aligned rectangles.  Keep their bounds
# in one module-level contiguous buffer instead of creating four point lists
# for every object on every planning pass.  32 covers 9 objects plus the
# configured circles/rectangles with room for normal field configurations.
_RECT_POOL_CAPACITY = 32
_RECT_POOL_STRIDE = 4
_RECT_POOL = array('f', [0.0] * (_RECT_POOL_CAPACITY * _RECT_POOL_STRIDE))

side_to_dir = {'D':0,'L':90,'U':180,'R':-90}
class BoundaryPathPlanner:
    __slots__ = ('Data', 'my_car', 'my_plan', 'flash_sys', 'sigal_swell_size', 'bothway_swell_size',
                 'SAFE_MARGIN', 'near_area', 'avoid_width', 'forward_push_value', 'rects', 'ready_path',
                 '_all_rects_work', '_forward_rects_work', '_node_dist_work', '_node_point_work',
                 '_fixed_barrier_cache')

    def __init__(self, plan_data, car, my_plan,flash_sys):
        self.Data = plan_data
        self.my_car = car
        self.my_plan = my_plan
        self.flash_sys = flash_sys
        self.sigal_swell_size = self.flash_sys.find_value("sigal_swell_size")#单向膨胀
        self.bothway_swell_size = self.flash_sys.find_value("bothway_swell_size")#双向膨胀
        self.SAFE_MARGIN = self.flash_sys.find_value("MOVE_SAFE_MARGIN")#四周膨胀半径
        self.near_area = self.flash_sys.find_value("NEAR_AREA")
        self.avoid_width = self.flash_sys.find_value("AVOID_WIDTH")
        self.forward_push_value = self.flash_sys.find_value("FORWARD_PUSH_VALUE")
        # All planner instances in one MicroPython runtime share this fixed
        # buffer.  Only one transport decision is evaluated at a time.
        self.rects = _RECT_POOL
        self.rects_length = 0
        self._rect_overflow = False
        self.ready_path = []
        self._node_dist_work = []
        self._node_point_work = []
        gc.collect()

    def _append_swelled_axis_rect(self, cx, cy, half_w, half_h,
                                  swell_angle, swell_size, direction):
        """Write one expanded axis-aligned rectangle into the fixed pool."""
        if self.rects_length >= _RECT_POOL_CAPACITY:
            self._rect_overflow = True
            return False
        min_x = cx - half_w
        max_x = cx + half_w
        min_y = cy - half_h
        max_y = cy + half_h
        if swell_angle == -90:
            min_x -= swell_size
        elif swell_angle == 0:
            max_y += swell_size
        elif swell_angle == 90:
            max_x += swell_size
        elif swell_angle == 180:
            min_y -= swell_size
        elif swell_angle == 1 or swell_angle == -1:
            _, right = self._forward_right(self._normalize_dir(direction))
            if right[0] > 0:
                min_x -= swell_size
                max_x += swell_size
            elif right[0] < 0:
                min_x -= swell_size
                max_x += swell_size
            elif right[1] > 0:
                min_y -= swell_size
                max_y += swell_size
            else:
                min_y -= swell_size
                max_y += swell_size
        base = self.rects_length * _RECT_POOL_STRIDE
        pool = self.rects
        pool[base] = min_x
        pool[base + 1] = min_y
        pool[base + 2] = max_x
        pool[base + 3] = max_y
        self.rects_length += 1
        return True

    def special_swell_barriers(self, objects_, swell_angle, skip_idx=None,
                               direction=None):
        if swell_angle == 1 or swell_angle== -1:swell_size = self.bothway_swell_size
        else:swell_size = self.sigal_swell_size
        circle_r = float(self.Data.OBSTACLE_R)
        safe_margin = self.SAFE_MARGIN
        circles = self.Data.circle
        raw_rects = self.Data.rectangles
        objects = objects_ if objects_ is not None else ()
        self.rects_length = 0
        self._rect_overflow = False

        for obj_idx in range(len(objects)):
            if skip_idx is not None and obj_idx == skip_idx:
                continue
            obj = objects[obj_idx]
            if len(obj) >= 4:
                cx, cy = float(obj[0]), float(obj[1])
                half_w = float(obj[2]) / 2.0 + safe_margin
                half_h = float(obj[3]) / 2.0 + safe_margin
                self._append_swelled_axis_rect(
                    cx, cy, half_w, half_h, swell_angle,
                    swell_size, direction)
        for circle in circles:
            if len(circle) >= 2:
                cx, cy = float(circle[0]), float(circle[1])
                self._append_swelled_axis_rect(
                    cx, cy, circle_r + safe_margin, circle_r + safe_margin,
                    swell_angle, swell_size, direction)
        raw_rect_count = len(raw_rects)
        for rect_idx in range(raw_rect_count):
            if rect_idx == raw_rect_count - 1:
                continue
            rect = raw_rects[rect_idx]
            if len(rect) < 4:
                continue
            min_x = max_x = float(rect[0][0])
            min_y = max_y = float(rect[0][1])
            for point_idx in range(1, len(rect)):
                p = rect[point_idx]
                px, py = float(p[0]), float(p[1])
                if px < min_x: min_x = px
                elif px > max_x: max_x = px
                if py < min_y: min_y = py
                elif py > max_y: max_y = py
            self._append_swelled_axis_rect(
                (min_x + max_x) / 2.0, (min_y + max_y) / 2.0,
                (max_x - min_x) / 2.0, (max_y - min_y) / 2.0,
                swell_angle, swell_size, direction)
        return not self._rect_overflow

    def _filter_forward_rects(self, start, direction):
        x, y = start
        pool = self.rects
        write_idx = 0
        for read_idx in range(self.rects_length):
            src = read_idx * _RECT_POOL_STRIDE
            min_x, min_y = pool[src], pool[src + 1]
            max_x, max_y = pool[src + 2], pool[src + 3]
            use_rect = False
            if direction == 0 and max_y >= y:
                use_rect = True
            elif direction == 180 and min_y <= y:
                use_rect = True
            elif direction == 90 and max_x >= x:
                use_rect = True
            elif direction == -90 and min_x <= x:
                use_rect = True
            if use_rect:
                dst = write_idx * _RECT_POOL_STRIDE
                if dst != src:
                    pool[dst] = min_x
                    pool[dst + 1] = min_y
                    pool[dst + 2] = max_x
                    pool[dst + 3] = max_y
                write_idx += 1
        self.rects_length = write_idx
    def plan_move(self, direction, swell_dir, objects,x=None,y=None,skip_idx=None,limit_angle = None):
        if x is None or y is None:
            x, y = self.my_car.x_current, self.my_car.y_current
        if not self.special_swell_barriers(objects, swell_dir, skip_idx, direction):
            self.ready_path = []
            return self.ready_path
        self._filter_forward_rects((x, y), direction)
        self.ready_path = self.plan_one_turn(direction,limit_angle,x,y)
        return self.ready_path
    def plan_one_turn(self, direction,limit_angle,x=None,y=None):
        if x is None or y is None:x,y=self.my_car.x_current,self.my_car.y_current
        path_left = self._plan_one_turn_with_avoid(direction, -1,x,y)
        path_right = self._plan_one_turn_with_avoid(direction, 1,x,y)
        if limit_angle:
            path_left = self._limit_path_angle(path_left, direction, limit_angle)
            path_right = self._limit_path_angle(path_right, direction, limit_angle)
        if not path_left:return path_right
        if not path_right:return path_left
        if self._path_cost(path_left) <= self._path_cost(path_right):return path_left
        return path_right

    def _limit_path_angle(self, path, direction, limit_angle):
        if not path:
            return []
        push_yaw = math.atan2(path[1][0] - path[0][0],
                              path[1][1] - path[0][1]) * 180.0 / math.pi
        push_angle = abs(push_yaw - direction)
        if push_angle > 180:
            push_angle = 360 - push_angle
        if push_angle > limit_angle:
            return []
        return path

    def _plan_one_turn_with_avoid(self, direction, avoid_dir,x,y):
        direction = self._normalize_dir(direction)
        avoid_dir = 1 if avoid_dir >= 0 else -1
        start = (float(x), float(y))
        rects = self.rects
        start = self._nearest_valid(start, rects)
        direct_end = self._project_to_boundary(start, direction)
        if self._move_allowed(start, direct_end, direction, avoid_dir) and self._line_valid(start, direct_end, rects):
            return [[start[0], start[1]], [direct_end[0], direct_end[1]]]

        node_dists = self._node_dist_work
        node_points = self._node_point_work
        node_dists.clear()
        node_points.clear()
        fwd, right = self._forward_right(direction)
        for rect_idx in range(self.rects_length):
            p = self._avoid_corner_node(rect_idx, direction, avoid_dir)
            if not self._ahead_or_level(start, p, direction):
                continue
            if not self._same_avoid_side_or_level(start, p, direction, avoid_dir):
                continue
            side_dist = abs((p[0] - start[0]) * right[0] + (p[1] - start[1]) * right[1])
            self._insert_sorted_node(node_dists, node_points, side_dist, p)
        start_side = start[0] * right[0] + start[1] * right[1]
        for i in range(len(node_points)):
            ref_node = node_points[i]
            ref_side = ref_node[0] * right[0] + ref_node[1] * right[1]
            for j in range(i + 1):
                aim = node_points[j]
                aim_side = aim[0] * right[0] + aim[1] * right[1]
                den = aim_side - start_side
                if abs(den) < 0.000001:
                    continue
                t = (ref_side - start_side) / den
                if t < 0.0:
                    continue
                p = (start[0] + (aim[0] - start[0]) * t,
                     start[1] + (aim[1] - start[1]) * t)
                if not self.my_plan._inside_field(p):
                    continue
                if self._one_turn_candidate_cost(start, p, direction, avoid_dir, rects) < self.Data.INF:
                    end = self._project_to_boundary(p, direction)
                    return [[start[0], start[1]], [p[0], p[1]],
                            [end[0], end[1]]]
        return []

    def _insert_sorted_node(self, node_dists, node_points, side_dist, p):
        idx = 0
        while idx < len(node_dists) and node_dists[idx] <= side_dist:
            idx += 1
        node_dists.insert(idx, side_dist)
        node_points.insert(idx, p)

    def _avoid_corner_node(self, rect_idx, direction, avoid_dir):
        d = 2.0
        fwd, right = self._forward_right(direction)
        base = rect_idx * _RECT_POOL_STRIDE
        min_x, min_y = self.rects[base], self.rects[base + 1]
        max_x, max_y = self.rects[base + 2], self.rects[base + 3]
        cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0

        # The best point is the corner on the requested avoidance side and
        # closest to the car along the reverse push direction.
        if right[0] * avoid_dir > 0.0:
            best_x = max_x
        elif right[0] * avoid_dir < 0.0:
            best_x = min_x
        else:
            best_x = cx
        if right[1] * avoid_dir > 0.0:
            best_y = max_y
        elif right[1] * avoid_dir < 0.0:
            best_y = min_y
        else:
            best_y = cy
        if fwd[0] > 0.0:
            best_x = min_x
        elif fwd[0] < 0.0:
            best_x = max_x
        elif fwd[1] > 0.0:
            best_y = min_y
        else:
            best_y = max_y

        vx, vy = best_x - cx, best_y - cy
        length = math.sqrt(vx * vx + vy * vy)
        if length < 0.000001:
            return (best_x, best_y)
        return (best_x + vx / length * d,
                best_y + vy / length * d)

    def _one_turn_candidate_cost(self, start, p, direction, avoid_dir, rects):
        if not self._point_valid(p, rects):
            return self.Data.INF
        end = self._project_to_boundary(p, direction)
        if not self._move_allowed(start, p, direction, avoid_dir):
            return self.Data.INF
        if not self._move_allowed(p, end, direction, avoid_dir):
            return self.Data.INF
        if not self._line_valid(start, p, rects):
            return self.Data.INF
        if not self._line_valid(p, end, rects):
            return self.Data.INF

        return self.my_plan._distance(start, p) + self.my_plan._distance(p, end)

    def _path_cost(self, path):
        if not path:
            return self.Data.INF
        cost = 0.0
        for i in range(len(path) - 1):
            cost += self.my_plan._distance(path[i], path[i + 1])
        return cost

    def _normalize_dir(self, direction):
        if direction in (0, 90, 180, -90):
            return int(direction)
        raise ValueError("direction must be one of 0, 90, 180, -90")

    def _forward_right(self, direction):
        if direction == 0:
            return (0.0, 1.0), (1.0, 0.0)
        if direction == 90:
            return (1.0, 0.0), (0.0, -1.0)
        if direction == 180:
            return (0.0, -1.0), (-1.0, 0.0)
        return (-1.0, 0.0), (0.0, 1.0)

    def _project_to_boundary(self, p, direction):
        if direction == 0:
            return (p[0], self.Data.FIELD_H+20)
        if direction == 180:
            return (p[0], -20)
        if direction == 90:
            return (self.Data.FIELD_W+20, p[1])
        return (-20, p[1])

    def _nearest_valid(self, p, rects):
        px = max(0.0, min(float(p[0]), self.Data.FIELD_W))
        py = max(0.0, min(float(p[1]), self.Data.FIELD_H))
        p = (px, py)
        if self._point_valid(p, rects):
            return p

        radius = 2.0
        max_r = max(self.Data.FIELD_W, self.Data.FIELD_H)
        while radius < max_r:
            count = int(radius) + 8
            for i in range(count):
                a = 2.0 * math.pi * i / count
                q = (px + math.cos(a) * radius, py + math.sin(a) * radius)
                if self._point_valid(q, rects):
                    return q
            radius += 2.0
        return p

    def _point_valid(self, p, rects):
        if not self.my_plan._inside_field(p):
            return False
        px, py = p[0], p[1]
        pool = self.rects
        for rect_idx in range(self.rects_length):
            base = rect_idx * _RECT_POOL_STRIDE
            if (pool[base] <= px <= pool[base + 2] and
                    pool[base + 1] <= py <= pool[base + 3]):
                return False
        return True

    def _move_allowed(self, a, b, direction, avoid_dir):
        fwd, right = self._forward_right(direction)
        dx, dy = b[0] - a[0], b[1] - a[1]
        forward_len = dx * fwd[0] + dy * fwd[1]
        side_len = dx * right[0] + dy * right[1]
        return forward_len >= -0.001 and side_len * avoid_dir >= -0.001

    def _ahead_or_level(self, start, p, direction):
        fwd, _ = self._forward_right(direction)
        dx, dy = p[0] - start[0], p[1] - start[1]
        return dx * fwd[0] + dy * fwd[1] >= -0.001

    def _same_avoid_side_or_level(self, start, p, direction, avoid_dir):
        _, right = self._forward_right(direction)
        dx, dy = p[0] - start[0], p[1] - start[1]
        return (dx * right[0] + dy * right[1]) * avoid_dir >= -0.001

    def _line_valid(self, a, b, rects):
        for rect_idx in range(self.rects_length):
            if self._segment_hits_rect(a, b, rect_idx):
                return False
        return True

    def _segment_hits_rect(self, a, b, rect_idx):
        """Inclusive segment/AABB test without allocating polygon points."""
        base = rect_idx * _RECT_POOL_STRIDE
        pool = self.rects
        min_x, min_y = pool[base], pool[base + 1]
        max_x, max_y = pool[base + 2], pool[base + 3]
        ax, ay = a[0], a[1]
        dx, dy = b[0] - ax, b[1] - ay
        t0, t1 = 0.0, 1.0
        p, q = -dx, ax - min_x
        if abs(p) < 0.000001:
            if q < 0.0:
                return False
        else:
            t = q / p
            if p < 0.0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t

        p, q = dx, max_x - ax
        if abs(p) < 0.000001:
            if q < 0.0:
                return False
        else:
            t = q / p
            if p < 0.0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t

        p, q = -dy, ay - min_y
        if abs(p) < 0.000001:
            if q < 0.0:
                return False
        else:
            t = q / p
            if p < 0.0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t

        p, q = dy, max_y - ay
        if abs(p) < 0.000001:
            if q < 0.0:
                return False
        else:
            t = q / p
            if p < 0.0:
                if t > t1:
                    return False
                if t > t0:
                    t0 = t
            else:
                if t < t0:
                    return False
                if t < t1:
                    t1 = t
        return True
obj_wideness={'T':4.0,'S':3.0,'E':3.0,'B':3.0,'W':3.0,}
obj_height={'T':4.0,'S':3.0,'E':3.0,'B':3.0,'W':3.0,}
class objects_planner:
    __slots__ = ('flash_sys', 'my_write', 'Data', 'my_car', 'my_plan', 'my_BoundaryPath',
                 'objects_score', 'barrier', 'now_objects', 'target_score', 'plan_target', 'path',
                 'best_path', 'judge_state', 'last_sandbag_idx', 'now_idx', 'run_speed', 'nine_grid',
                 'target_objects')

    def __init__(self,my_flash_sys,my_write, plan_data, car, my_plan, my_BoundaryPath : BoundaryPathPlanner):
        self.flash_sys = my_flash_sys
        self.my_write = my_write
        self.Data = plan_data
        self.my_car = car
        self.my_plan = my_plan
        self.my_BoundaryPath = my_BoundaryPath
        self.objects_score = []
        self.barrier = []
        self.now_objects = []
        self.target_score = []
        self.plan_target = []
        self.path = []
        self.best_path = [0,0]
        self.judge_state = 0#0:未开始，1:正在进行，2:已结束
        self.last_sandbag_idx = -1
        self.now_idx = 0
        self.run_speed = self.flash_sys.find_value("long_v_max")
        self.nine_grid = [['','',''],
                          ['','',''],
                          ['','',''],]
        gc.collect()
    def set_barriers(self,barriers):
        
        for i in self.now_objects:
            # Vision results can occasionally be incomplete.  Ignore malformed
            # entries here instead of indexing into them and crashing the task.
            if not isinstance(i, (list, tuple)) or len(i) < 3 or i[0] not in obj_wideness:
                continue
            w,h=obj_wideness[i[0]],obj_height[i[0]]
            barriers.append([i[1],i[2],w,h])
    def reset_judge(self):
        self.last_sandbag_idx = -1
        self.path = []
        self.objects_score = []
        self.target_objects = []
        self.now_objects = []
        self.judge_state = 0
        self.barrier = []
        self.best_path = [0,0]
        self.target_score = []
        self.plan_target = []
        self.now_idx = 0
        gc.collect()
    def judge_side_in(self,side,now_object):
        def _if_p_block_p(p,p_):
            avoid_width = self.my_BoundaryPath.avoid_width
            near_area = self.my_BoundaryPath.near_area
            if side == 'D':
                if p_[1]>p[1]-near_area:return False
                if abs(p_[0]-p[0])>avoid_width:return False
            elif side == 'U':
                if p_[1]<p[1]+near_area:return False
                if abs(p_[0]-p[0])>avoid_width:return False
            elif side == 'L':
                if p_[0]>p[0]-near_area:return False
                if abs(p_[1]-p[1])>avoid_width:return False
            elif side == 'R':
                if p_[0]<p[0]+near_area:return False
                if abs(p_[1]-p[1])>avoid_width:return False
            return True
        for j in self.now_objects:
            i=now_object
            if i == j:continue
            if _if_p_block_p([i[1],i[2]],[j[1],j[2]]):
                return False
        return True
    def nine_grid_postion_to_idx(self, x, y=None):
        """Return [row, col] for an exact nine-grid center, or [] if absent."""
        if y is None:
            if not isinstance(x, (list, tuple)) or len(x) != 2:
                return []
            x, y = x

        center_x = self.Data.center_x
        center_y = self.Data.center_y
        length = self.Data.lenth
        if length <= 0:
            return []

        col = int(round((x - center_x) / length)) + 1
        row = int(round((y - center_y) / length)) + 1
        if row < 0 or row > 2 or col < 0 or col > 2:
            return []

        expected_x = center_x + (col - 1) * length
        expected_y = center_y + (row - 1) * length
        if abs(x - expected_x) > 1e-6 or abs(y - expected_y) > 1e-6:
            return []
        return [row, col]

    def nine_grid_idx_to_postion(self, idx, col=None):
        """Return [x, y] for a [row, col] index, or [] if the index is invalid."""
        if col is None:
            if not isinstance(idx, (list, tuple)) or len(idx) != 2:
                return []
            row, col = idx
        else:
            row = idx

        if not isinstance(row, int) or not isinstance(col, int):
            return []
        if row < 0 or row > 2 or col < 0 or col > 2:
            return []

        center_x = self.Data.center_x
        center_y = self.Data.center_y
        length = self.Data.lenth
        if length <= 0:
            return []
        return [center_x + (col - 1) * length,
                center_y + (row - 1) * length]

    def generate_nine_grid(self):
        """Fill the 3x3 grid with object kinds from snapped object coordinates."""
        self.nine_grid = [['', '', ''], ['', '', ''], ['', '', '']]
        for obj in self.now_objects:
            if not obj or len(obj) < 3:
                continue
            idx = self.nine_grid_postion_to_idx(obj[1],obj[2])
            if idx:
                self.nine_grid[idx[0]][idx[1]] = obj[0]
        return self.nine_grid
    def judge_side_in_nine_grid(self,obj,dir,k):
        if not obj or len(obj) < 3:
            return False
        now_pt = self.nine_grid_postion_to_idx(obj[1],obj[2])
        if not now_pt:
            return False
        now_pt[0] += dir[0] * k
        now_pt[1] += dir[1] * k
        while now_pt[0] < 3 and now_pt[0] >= 0 and now_pt[1] < 3 and now_pt[1] >= 0:
            if self.nine_grid[now_pt[0]][now_pt[1]] != '':
                return False
            now_pt[0] += dir[0] * k
            now_pt[1] += dir[1] * k
        return True
    def judge_side_in_nine_grid_idx(self,idx,dir,k):
        if not idx or len(idx) < 2:
            return False
        idx = idx[:]
        idx[0] += dir[0] * k
        idx[1] += dir[1] * k
        while idx[0] < 3 and idx[0] >= 0 and idx[1] < 3 and idx[1] >= 0:
            if self.nine_grid[idx[0]][idx[1]] != '':
                return False
            idx[0] += dir[0] * k
            idx[1] += dir[1] * k
        return True
    def find_nine_grid_blank(self,obj,push_dir,in_dir):
        now_pt = self.nine_grid_postion_to_idx(obj[1],obj[2])
        if not now_pt:
            return [None, 0]
        i = now_pt[:]
        i[0] -= push_dir[0]
        i[1] -= push_dir[1]
        num = 0
        k = push_dir[0] + push_dir[1]
        use_big_rect = True
        while i[0] < 3 and i[0] >= 0 and i[1] < 3 and i[1] >= 0:
            if self.nine_grid[i[0]][i[1]] != '':
                use_big_rect = False
                break
            num += k
            if self.judge_side_in_nine_grid_idx(i,in_dir,-1):
                # in_dir uses [row, col], while world coordinates use [x, y].
                target_edge = [self.Data.center_x-in_dir[1]*1.5*self.Data.lenth,
                                self.Data.center_y-in_dir[0]*1.5*self.Data.lenth]#反向寻找进入边界
                p2 = self.nine_grid_idx_to_postion(i)
                if in_dir[0] == 0:p2 = [target_edge[0],p2[1]]
                else:p2 = [p2[0],target_edge[1]]
                now_xy = self.nine_grid_idx_to_postion(now_pt)
                return [[[min(now_xy[0],p2[0]),min(now_xy[1],p2[1])],[max(now_xy[0],p2[0]),max(now_xy[1],p2[1])]],num]
            i[0] -= push_dir[0]
            i[1] -= push_dir[1]
        if use_big_rect:return [[],0]
        else :return [None,0]
    def judge_object_character(self,objects,car_side):
        if self.judge_state == 0:
            # Keep only records with the fields used by the planner.  A partial
            # detection must not make the state machine fail during indexing.
            self.now_objects = [obj for obj in (objects or [])
                                if isinstance(obj, (list, tuple)) and len(obj) >= 3]
            self.set_barriers(self.barrier)#将物体转化为障碍形式并存储在self.barrier中
            self.generate_nine_grid()
            self.judge_state = 1
            return False
        elif self.judge_state == 1:#筛选出能直接搬运的物体
            idx=0
            self.target_objects = []
            for target in self.now_objects:
                could_select = True
                if target[0] == 'S' or target[0] == 'E':
                    push_dir = [0,-1]#推动正方向
                    self.last_sandbag_idx += 1
                    target_side = 'R'
                    if car_side == 'R' or car_side == 'L':could_select = False
                elif target[0] == 'T':
                    push_dir = [1,0]#推动正方向
                    target_side = 'D'
                    if car_side == 'D' or car_side == 'U':could_select = False
                else:
                    push_dir = [0,1]#推动正方向
                    target_side = 'L'
                    if car_side == 'L' or car_side == 'R':could_select = False
                if car_side == 'L':in_dir = [0,1]#进入正方向
                elif car_side == 'R':in_dir = [0,-1]
                elif car_side == 'D':in_dir = [1,0]
                else: in_dir = [-1,0]
                
                if self.judge_side_in_nine_grid(target,in_dir,-1):
                    self.target_objects.append([idx,target[0],target[1],target[2],car_side,[],0])#序号，物体种类，x,y,目标边,空表示用原矩形
                if not self.judge_side_in_nine_grid(target,push_dir,1):could_select = False
                if could_select:
                    rect,num = self.find_nine_grid_blank(target,push_dir,in_dir)
                    if rect != None:
                        self.target_objects.append([idx,target[0],target[1],target[2],target_side,rect,num])#序号，物体种类，x,y,目标边
                idx+=1
            self.judge_state = 2
            return False
        elif self.judge_state == 2:#计算每个目标物体的评分
            
            if self.now_idx>=len(self.target_objects): self.judge_state = 3
            else:
                i = self.target_objects[self.now_idx]
                score = 0
                dir,sdir=self.judge_push_direction(i[1])
                if dir < side_to_dir[car_side]+0.1 and dir > side_to_dir[car_side]-0.1:score+=self.my_BoundaryPath.forward_push_value
                # 根据物体的种类调整目标点的位置，S和E向前移动10，B和W向后移动10，T向上移动10
                sx,sy = i[2],i[3]
                if i[1] in ['S', 'E']:  
                    sx += 10.0
                elif i[1] in ['B', 'W']:
                    sx -= 10.0
                elif i[1] in ['T']:
                    sy -= 10.0 
                if car_side == i[4]:
                    path = self.my_BoundaryPath.plan_move(dir, sdir, self.barrier, sx, sy, skip_idx=i[0])
                else:
                    has_planned = False
                    for j in range(self.now_idx):
                        if i[0] == self.target_objects[j][0]:
                            path = [[sx,sy]]+self.path[j]
                            has_planned = True
                            break
                    if not has_planned:
                        path = self.my_BoundaryPath.plan_move(dir, sdir, self.barrier, sx, sy, skip_idx=i[0])
                push_distance,push_angle= 1000,90
                if (not path) or len(path) <= 1: 
                    self.path.append([])
                    score+=10000
                else:
                    if len(path) == 2:
                        p_,p__=path[0],path[1]
                        push_distance = self.calculate_distance(p_,p__)
                    else:
                        p_,p__,p___=path[0],path[1],path[2]
                        push_distance = self.calculate_distance(p_,p__)+self.calculate_distance(p__,p___)
                    push_yaw = math.atan2(p__[0] - p_[0], p__[1] - p_[1]) * 180.0 / math.pi
                    self.path.append(path[1:])
                    push_angle = abs(push_yaw - dir)
                    if push_angle > 180:
                        push_angle = 360 - push_angle
                #旋转加分
                if (i[1] == 'S' or i[1] == 'E'):
                    if i[4] !='R':score+=1000
                    if self.last_sandbag_idx == 0:score+=1000
                elif (i[1] == 'T') and i[4] !='D':score+=1000
                elif (i[1] == 'W' or i[1] == 'B') and i[4] !='L':score+=1000
                # 大角度搬运路径加分
                if abs(push_angle) > 55: 
                    if i[1] == 'T': 
                        score+=10000
                    else:
                        score+=5000
                if car_side == i[4]:
                    dx_car = i[2] - self.my_car.x_current
                    dy_car = i[3] - self.my_car.y_current
                    distance_from_car = math.sqrt(dx_car * dx_car + dy_car * dy_car)
                else:
                    RECT = i[5]
                    if not RECT:#使用大矩阵
                        RECT = [self.Data.center_rect[0],self.Data.center_rect[3]]
                    P = {'D':((RECT[0][0]+RECT[1][0])/2,RECT[0][1]),
                         'L':(RECT[0][0],(RECT[0][1]+RECT[1][1])/2),
                         'U':((RECT[0][0]+RECT[1][0])/2,RECT[1][1]),
                         'R':(RECT[1][0],(RECT[0][1]+RECT[1][1])/2),}
                    x1 = i[2] - P[i[4]][0]
                    x2 = self.my_car.x_current - P[i[4]][0]
                    y1 = i[3] - P[i[4]][1]
                    y2 = self.my_car.y_current - P[i[4]][1]
                    distance_from_car = math.sqrt(x1 * x1 + y1 * y1) + math.sqrt(x2 * x2 + y2 * y2)
                dis_score = 9.69*180/self.run_speed
                score += push_distance + push_angle*push_angle +distance_from_car*dis_score
                self.my_write.write_str("object {} push_dis:{} angle:{} dis:{}\n".format(i[1], push_distance, push_angle*push_angle, distance_from_car*dis_score))
                self.target_score.append(score)
                self.now_idx+=1
            return False
        elif self.judge_state == 3:#选择评分最低的物体作为目标
            new_path = []
            for i in range(len(self.target_score)):
                if self.target_score[i] == min(self.target_score):
                    self.plan_target = self.target_objects[i]
                    if self.path[i]:
                        raw_x, raw_y = self.plan_target[2], self.plan_target[3]
                        if self.plan_target[1] in ['S', 'E']:
                            raw_x += 10.0
                        elif self.plan_target[1] in ['B', 'W']:
                            raw_x -= 10.0
                        elif self.plan_target[1] == 'T':
                            raw_y -= 10.0
                        dx = self.path[i][0][0] - raw_x
                        dy = self.path[i][0][1] - raw_y
                        self.best_path = [dx,dy]
                if self.path[i]:
                    new_path.append(self.path[i][0])
            self.path = new_path
            return True
    def find_target(self):
        if self.objects_score:
            Target = self.objects_score[0]
            for i in self.objects_score:
                if i[1]==Target[1]:
                    if i[2]<Target[2]:Target = i
                elif i[1]<Target[1]:Target = i
            return Target
        else:
            return []
    def calculate_distance(self,p1,p2):
        return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    def judge_need_area(self,sp):
        if sp=='T': return {'DL':[],'DR':[]}
        elif sp=='S' or sp=='E': return {'DR':[],'UR':[]}
        elif sp=='B' or sp=='W': return {'DL':[],'UL':[]}
        else :return {}
    def judge_push_direction(self,sp):
        if sp=='T': return 0,-1
        elif sp=='S' or sp=='E': return -90,1
        elif sp=='B' or sp=='W': return 90,1
        else :return {}
    def judge_UDRL_area(self,p,p_):
        if p[0]>p_[0]:
            if p[1]>p_[1]: return 'DL'
            else:return 'UL'
        else :
            if p[1]>p_[1]: return 'DR'
            else:return 'UR'
    def judge_running_area(self,p,p_,barriar,sp):
        dx = p[0]-p_[0]
        dy = p[1]-p_[1]
        if dy!=0 and (dx<5 or abs(dx-5)/abs(dy)<=0.1):
            if dy>0:barriar['D'].append(sp)
            else:barriar['U'].append(sp)
        if dx!=0 and (dy<5 or abs(dy-5)/abs(dx)<=0.1):
            if dy>0:barriar['L'].append(sp)
            else:barriar['R'].append(sp)
