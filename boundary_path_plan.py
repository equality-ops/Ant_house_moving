import math


FIELD_X_MAX = 320.0
FIELD_Y_MAX = 240.0
INF = 1000000000.0


class BoundaryPathPlanner:
    def __init__(self, field_x=FIELD_X_MAX, field_y=FIELD_Y_MAX,
                 safe_margin=13.0, circle_radius=16.0):
        self.field_x = float(field_x)
        self.field_y = float(field_y)
        self.safe_margin = float(safe_margin)
        self.circle_radius = float(circle_radius)

    def plan(self, x, y, direction, avoid_dir, objects=None, circles=None):
        objects = objects or []
        circles = circles or []
        direction = self._normalize_dir(direction)
        avoid_dir = 1 if avoid_dir >= 0 else -1

        start = (float(x), float(y))
        rects = self._build_rects(objects, circles)
        start = self._nearest_valid(start, rects)

        nodes = [start]
        nodes.extend(self._boundary_nodes(direction, start, rects))
        self._add_rect_nodes(nodes, rects)
        nodes = self._unique_valid_nodes(nodes, rects)

        start_idx = self._find_node(nodes, start)
        target_idxs = []
        for i, p in enumerate(nodes):
            if i != start_idx and self._is_on_target_boundary(p, direction):
                target_idxs.append(i)

        if not target_idxs:
            return []

        n = len(nodes)
        dist = [INF] * n
        prev = [-1] * n
        used = [False] * n
        dist[start_idx] = 0.0
        end_idx = -1

        for _ in range(n):
            u = -1
            best = INF
            for i in range(n):
                if not used[i] and dist[i] < best:
                    best = dist[i]
                    u = i
            if u < 0:
                break
            if u in target_idxs:
                end_idx = u
                break
            used[u] = True

            for v in range(n):
                if used[v] or v == u:
                    continue
                if not self._move_allowed(nodes[u], nodes[v], direction, avoid_dir):
                    continue
                if not self._line_valid(nodes[u], nodes[v], rects):
                    continue
                w = self._distance(nodes[u], nodes[v])
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u

        if end_idx < 0:
            return []

        path = []
        i = end_idx
        while i >= 0:
            path.append(nodes[i])
            i = prev[i]
        path.reverse()
        return self._path_to_list(self._smooth_path(path, rects, direction, avoid_dir))

    def plan_one_turn(self, x, y, direction, avoid_dir, objects=None, circles=None):
        objects = objects or []
        circles = circles or []
        direction = self._normalize_dir(direction)
        avoid_dir = 1 if avoid_dir >= 0 else -1

        start = (float(x), float(y))
        rects = self._build_rects(objects, circles)
        start = self._nearest_valid(start, rects)

        direct_end = self._project_to_boundary(start, direction)
        if (self._move_allowed(start, direct_end, direction, avoid_dir)
                and self._line_valid(start, direct_end, rects)):
            return self._path_to_list([start, direct_end])

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
        candidates.extend(self._one_turn_intersections(start, aim_nodes,
                                                       ref_nodes, direction))
        candidates = self._unique_valid_nodes(candidates, rects)

        best_path = []
        best_cost = INF
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

            cost = self._distance(start, p) + self._distance(p, end)
            if cost < best_cost:
                best_cost = cost
                best_path = [start, p, end]

        return self._path_to_list(best_path)

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

    def _build_rects(self, objects, circles):
        rects = []
        for obj in objects:
            if len(obj) >= 4:
                cx, cy = float(obj[0]), float(obj[1])
                half_x = float(obj[2]) / 2.0 + self.safe_margin
                half_y = float(obj[3]) / 2.0 + self.safe_margin
                rects.append(self._make_rect(cx, cy, half_x, half_y))
        for c in circles:
            if len(c) >= 2:
                cx, cy = float(c[0]), float(c[1])
                r = self.circle_radius + self.safe_margin
                rects.append(self._make_rect(cx, cy, r, r))
        return rects

    def _make_rect(self, cx, cy, half_x, half_y):
        return [
            (cx - half_x, cy - half_y),
            (cx + half_x, cy - half_y),
            (cx + half_x, cy + half_y),
            (cx - half_x, cy + half_y),
        ]

    def _boundary_nodes(self, direction, start, rects):
        sx, sy = start
        points = []
        if direction in (0, 180):
            by = self.field_y if direction == 0 else 0.0
            points.append((sx, by))
            points.append((0.0, by))
            points.append((self.field_x, by))
            for rect in rects:
                for p in rect:
                    points.append((p[0], by))
                for p in self._rect_corner_nodes(rect):
                    points.append((p[0], by))
        else:
            bx = self.field_x if direction == 90 else 0.0
            points.append((bx, sy))
            points.append((bx, 0.0))
            points.append((bx, self.field_y))
            for rect in rects:
                for p in rect:
                    points.append((bx, p[1]))
                for p in self._rect_corner_nodes(rect):
                    points.append((bx, p[1]))
        return points

    def _add_rect_nodes(self, nodes, rects):
        for rect in rects:
            nodes.extend(self._rect_corner_nodes(rect))

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
        fwd, right = self._forward_right(direction)
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
                if self._inside_field(p):
                    out.append(p)
        return out

    def _project_to_boundary(self, p, direction):
        if direction == 0:
            return (p[0], self.field_y)
        if direction == 180:
            return (p[0], 0.0)
        if direction == 90:
            return (self.field_x, p[1])
        return (0.0, p[1])

    def _nearest_valid(self, p, rects):
        px = max(0.0, min(float(p[0]), self.field_x))
        py = max(0.0, min(float(p[1]), self.field_y))
        p = (px, py)
        if self._point_valid(p, rects):
            return p

        radius = 2.0
        max_r = max(self.field_x, self.field_y)
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

    def _find_node(self, nodes, target):
        for i, p in enumerate(nodes):
            if abs(p[0] - target[0]) < 0.001 and abs(p[1] - target[1]) < 0.001:
                return i
        return 0

    def _point_valid(self, p, rects):
        if not self._inside_field(p):
            return False
        for rect in rects:
            if self._point_in_poly(p, rect):
                return False
        return True

    def _inside_field(self, p):
        return 0.0 <= p[0] <= self.field_x and 0.0 <= p[1] <= self.field_y

    def _is_on_target_boundary(self, p, direction):
        eps = 0.001
        if direction == 0:
            return abs(p[1] - self.field_y) <= eps
        if direction == 180:
            return abs(p[1]) <= eps
        if direction == 90:
            return abs(p[0] - self.field_x) <= eps
        return abs(p[0]) <= eps

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
            if self._segment_hits_poly(a, b, rect):
                return False
        return True

    def _smooth_path(self, path, rects, direction, avoid_dir):
        if len(path) <= 2:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if (self._move_allowed(path[i], path[j], direction, avoid_dir)
                        and self._line_valid(path[i], path[j], rects)):
                    break
                j -= 1
            out.append(path[j])
            i = j
        return out

    def _distance(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _cross(self, a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def _on_segment(self, a, b, p):
        return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
                and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])
                and abs(self._cross(a, b, p)) < 0.000001)

    def _seg_intersect(self, a, b, c, d):
        c1, c2 = self._cross(a, b, c), self._cross(a, b, d)
        c3, c4 = self._cross(c, d, a), self._cross(c, d, b)
        if c1 * c2 < 0.0 and c3 * c4 < 0.0:
            return True
        if abs(c1) < 0.000001 and self._on_segment(a, b, c):
            return True
        if abs(c2) < 0.000001 and self._on_segment(a, b, d):
            return True
        if abs(c3) < 0.000001 and self._on_segment(c, d, a):
            return True
        if abs(c4) < 0.000001 and self._on_segment(c, d, b):
            return True
        return False

    def _point_in_poly(self, p, poly):
        inside = False
        j = len(poly) - 1
        for i in range(len(poly)):
            pi, pj = poly[i], poly[j]
            if self._on_segment(pi, pj, p):
                return True
            if (pi[1] > p[1]) != (pj[1] > p[1]):
                x = (pj[0] - pi[0]) * (p[1] - pi[1]) / (pj[1] - pi[1]) + pi[0]
                if p[0] < x:
                    inside = not inside
            j = i
        return inside

    def _segment_hits_poly(self, a, b, poly):
        if self._point_in_poly(a, poly) or self._point_in_poly(b, poly):
            return True
        j = len(poly) - 1
        for i in range(len(poly)):
            if self._seg_intersect(a, b, poly[j], poly[i]):
                return True
            j = i
        return False

    def _path_to_list(self, path):
        return [[p[0], p[1]] for p in path]
