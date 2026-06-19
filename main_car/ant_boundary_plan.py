import math
import gc


class BoundaryPathPlanner:
    def __init__(self, plan_data, car, my_plan):
        self.Data = plan_data
        self.my_car = car
        self.my_plan = my_plan
        self.rects = []
        self.ready_path = []
        gc.collect()

    def special_swell_barriers(self, objects_, swell_angle):
        swell_size = 20.0
        circle_r = float(self.Data.OBSTACLE_R)
        circles = self.Data.circle[:]
        raw_rects = self.Data.rectangles[:]
        if raw_rects:
            raw_rects.pop(-1)
        objects = objects_[:] if objects_ else []
        rects = []

        def make_rect(cx, cy, half_w, half_h):
            return [
                (cx - half_w, cy - half_h),
                (cx + half_w, cy - half_h),
                (cx + half_w, cy + half_h),
                (cx - half_w, cy + half_h)
            ]

        def swell_rect(rect):
            out = []
            for p in rect:
                x, y = float(p[0]), float(p[1])
                if swell_angle == -90:
                    if x < rect[0][0] + 0.001:
                        x -= swell_size
                elif swell_angle == 0:
                    if y > rect[0][1] + 0.001:
                        y += swell_size
                elif swell_angle == 90:
                    if x > rect[0][0] + 0.001:
                        x += swell_size
                elif swell_angle == 180:
                    if y < rect[2][1] - 0.001:
                        y -= swell_size
                out.append((x, y))
            return out

        for obj in objects:
            if len(obj) >= 4:
                cx, cy = float(obj[0]), float(obj[1])
                half_w = float(obj[2]) / 2.0
                half_h = float(obj[3]) / 2.0
                rects.append(swell_rect(make_rect(cx, cy, half_w, half_h)))

        for circle in circles:
            if len(circle) >= 2:
                cx, cy = float(circle[0]), float(circle[1])
                rects.append(swell_rect(make_rect(cx, cy, circle_r, circle_r)))

        for rect in raw_rects:
            if len(rect) >= 4:
                rects.append(swell_rect(rect))
        return rects

    def plan_move(self, direction, swell_dir, objects):
        self.rects = self.special_swell_barriers(objects, swell_dir)
        self.ready_path = self.plan_one_turn(direction)
        return self.ready_path

    def plan_one_turn(self, direction):
        path_left = self._plan_one_turn_with_avoid(direction, -1)
        path_right = self._plan_one_turn_with_avoid(direction, 1)
        if not path_left:
            return path_right
        if not path_right:
            return path_left
        if self._path_cost(path_left) <= self._path_cost(path_right):
            return path_left
        return path_right

    def _plan_one_turn_with_avoid(self, direction, avoid_dir):
        direction = self._normalize_dir(direction)
        avoid_dir = 1 if avoid_dir >= 0 else -1
        start = (float(self.my_car.x_current), float(self.my_car.y_current))
        rects = self.rects[:]
        start = self._nearest_valid(start, rects)
        direct_end = self._project_to_boundary(start, direction)
        if self._move_allowed(start, direct_end, direction, avoid_dir) and self._line_valid(start, direct_end, rects):
            return self.my_plan._path_to_list([start, direct_end])

        raw_corner_nodes = []
        for rect in rects:
            raw_corner_nodes.extend(self._rect_corner_nodes(rect))

        aim_nodes = []
        ref_nodes = []
        for p in raw_corner_nodes:
            if self._ahead_or_level(start, p, direction):
                ref_end = self._project_to_boundary(p, direction)
                if self._move_allowed(p, ref_end, direction, avoid_dir):
                    ref_nodes.append(p)
                if self._same_avoid_side_or_level(start, p, direction, avoid_dir):
                    aim_nodes.append(p)

        candidates = aim_nodes[:]
        candidates.extend(self._one_turn_intersections(start, aim_nodes, ref_nodes, direction))
        candidates = self._unique_valid_nodes(candidates, rects)

        best_path = []
        best_cost = self.Data.INF
        for p in candidates:
            end = self._project_to_boundary(p, direction)
            if not self._move_allowed(start, p, direction, avoid_dir):
                continue
            if not self._move_allowed(p, end, direction, avoid_dir):
                continue
            if not self._line_valid(start, p, rects):
                continue
            if not self._line_valid(p, end, rects):
                continue

            cost = self.my_plan._distance(start, p) + self.my_plan._distance(p, end)
            if cost < best_cost:
                best_cost = cost
                best_path = [start, p, end]

        return self.my_plan._path_to_list(best_path)

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

    def _rect_corner_nodes(self, rect):
        d = 2.0
        out = []
        cx = sum(p[0] for p in rect) / len(rect)
        cy = sum(p[1] for p in rect) / len(rect)
        for p in rect:
            vx, vy = p[0] - cx, p[1] - cy
            length = math.sqrt(vx * vx + vy * vy)
            if length < 1e-6:
                out.append(p)
            else:
                out.append((p[0] + vx / length * d,
                            p[1] + vy / length * d))
        return out

    def _one_turn_intersections(self, start, aim_nodes, ref_nodes, direction):
        _, right = self._forward_right(direction)
        out = []
        start_side = start[0] * right[0] + start[1] * right[1]

        for aim in aim_nodes:
            aim_side = aim[0] * right[0] + aim[1] * right[1]
            den = aim_side - start_side
            if abs(den) < 0.000001:
                continue

            for boundary_ref in ref_nodes:
                ref_side = boundary_ref[0] * right[0] + boundary_ref[1] * right[1]
                t = (ref_side - start_side) / den
                if t < 0.0:
                    continue
                px = start[0] + (aim[0] - start[0]) * t
                py = start[1] + (aim[1] - start[1]) * t
                p = (px, py)
                if self.my_plan._inside_field(p):
                    out.append(p)
        return out

    def _project_to_boundary(self, p, direction):
        if direction == 0:
            return (p[0], self.Data.FIELD_H)
        if direction == 180:
            return (p[0], 0.0)
        if direction == 90:
            return (self.Data.FIELD_W, p[1])
        return (0.0, p[1])

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

    def _unique_valid_nodes(self, nodes, rects):
        out = []
        for p in nodes:
            p = (float(p[0]), float(p[1]))
            if not self._point_valid(p, rects):
                continue
            duplicated = False
            for q in out:
                if abs(p[0] - q[0]) < 0.001 and abs(p[1] - q[1]) < 0.001:
                    duplicated = True
                    break
            if not duplicated:
                out.append(p)
        return out

    def _point_valid(self, p, rects):
        if not self.my_plan._inside_field(p):
            return False
        for rect in rects:
            if self.my_plan._point_in_poly(p, rect):
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
        for rect in rects:
            if self.my_plan._segment_hits_poly(a, b, rect):
                return False
        return True
