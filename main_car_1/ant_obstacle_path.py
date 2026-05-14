import math


FIELD_W = 240.0
FIELD_H = 320.0
OBSTACLE_R = 10.0
SAFE_MARGIN = 2.0
INF = 1000000000.0


def plan_path(circle, retangle, x0, y0, x1, y1, r=OBSTACLE_R, margin=SAFE_MARGIN):
    """
    Return waypoint list from (x0, y0) to (x1, y1), avoiding circles and rectangles.

    circle:    [(cx, cy), ...]
    retangle:  [[(x, y), (x, y), (x, y), (x, y)], ...]
               Also accepts one rectangle as [(x, y), (x, y), (x, y), (x, y)].
    """
    circles = _normalize_points(circle)
    rects = _normalize_rectangles(retangle)
    start = (float(x0), float(y0))
    end = (float(x1), float(y1))
    block_r = float(r) + float(margin)

    if not _point_valid(start, circles, rects, block_r):
        return []
    if not _point_valid(end, circles, rects, block_r):
        return []
    if _line_valid(start, end, circles, rects, block_r):
        return [[x0, y0], [x1, y1]]

    nodes = [start, end]
    _add_circle_nodes(nodes, circles, rects, block_r)
    _add_rectangle_nodes(nodes, rects, margin)
    nodes = _unique_valid_nodes(nodes, circles, rects, block_r)

    n = len(nodes)
    dist = [INF] * n
    prev = [-1] * n
    used = [False] * n
    dist[0] = 0.0

    for _ in range(n):
        u = -1
        best = INF
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
            if _line_valid(nodes[u], nodes[v], circles, rects, block_r):
                w = _distance(nodes[u], nodes[v])
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u

    if prev[1] < 0:
        return []

    path = []
    i = 1
    while i >= 0:
        path.append(nodes[i])
        i = prev[i]
    path.reverse()
    return _path_to_list(_smooth_path(path, circles, rects, block_r))


def _normalize_points(points):
    out = []
    if not points:
        return out
    for p in points:
        if len(p) >= 2:
            out.append((float(p[0]), float(p[1])))
    return out


def _normalize_rectangles(rects):
    if not rects:
        return []
    if len(rects) == 4 and len(rects[0]) == 2 and isinstance(rects[0][0], (int, float)):
        return [_sort_polygon(_normalize_points(rects))]

    out = []
    for rect in rects:
        pts = _normalize_points(rect)
        if len(pts) >= 3:
            out.append(_sort_polygon(pts))
    return out


def _sort_polygon(poly):
    cx = 0.0
    cy = 0.0
    for p in poly:
        cx += p[0]
        cy += p[1]
    cx /= len(poly)
    cy /= len(poly)
    items = []
    for p in poly:
        items.append((math.atan2(p[1] - cy, p[0] - cx), p))
    items.sort()
    out = []
    for item in items:
        out.append(item[1])
    return out


def _add_circle_nodes(nodes, circles, rects, block_r):
    dirs = (
        (1.0, 0.0), (0.9239, 0.3827), (0.7071, 0.7071), (0.3827, 0.9239),
        (0.0, 1.0), (-0.3827, 0.9239), (-0.7071, 0.7071), (-0.9239, 0.3827),
        (-1.0, 0.0), (-0.9239, -0.3827), (-0.7071, -0.7071), (-0.3827, -0.9239),
        (0.0, -1.0), (0.3827, -0.9239), (0.7071, -0.7071), (0.9239, -0.3827),
    )
    d = block_r + 1.0
    for c in circles:
        for v in dirs:
            nodes.append((c[0] + v[0] * d, c[1] + v[1] * d))
        _add_circle_tangent_nodes(nodes, c, nodes[:], d, block_r)
        anchors = []
        for rect in rects:
            for p in rect:
                anchors.append(p)
        _add_circle_tangent_nodes(nodes, c, anchors, d, block_r)


def _add_circle_tangent_nodes(nodes, c, anchors, node_r, block_r):
    for a in anchors:
        dx = a[0] - c[0]
        dy = a[1] - c[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= block_r + 0.001:
            continue
        theta = math.atan2(dy, dx)
        alpha = math.acos(block_r / dist)
        nodes.append((c[0] + math.cos(theta + alpha) * node_r,
                      c[1] + math.sin(theta + alpha) * node_r))
        nodes.append((c[0] + math.cos(theta - alpha) * node_r,
                      c[1] + math.sin(theta - alpha) * node_r))


def _add_rectangle_nodes(nodes, rects, margin):
    d = float(margin) + 1.0
    for rect in rects:
        cx = 0.0
        cy = 0.0
        for p in rect:
            cx += p[0]
            cy += p[1]
        cx /= len(rect)
        cy /= len(rect)
        for p in rect:
            vx = p[0] - cx
            vy = p[1] - cy
            l = math.sqrt(vx * vx + vy * vy)
            if l == 0.0:
                nodes.append(p)
            else:
                nodes.append((p[0] + vx / l * d, p[1] + vy / l * d))


def _unique_valid_nodes(nodes, circles, rects, block_r):
    out = []
    for p in nodes:
        if not _inside_field(p):
            continue
        if not _point_valid(p, circles, rects, block_r):
            continue
        duplicate = False
        for q in out:
            if abs(p[0] - q[0]) < 0.001 and abs(p[1] - q[1]) < 0.001:
                duplicate = True
                break
        if not duplicate:
            out.append(p)
    return out


def _point_valid(p, circles, rects, block_r):
    if not _inside_field(p):
        return False
    for c in circles:
        if _distance(p, c) <= block_r:
            return False
    for rect in rects:
        if _point_in_poly(p, rect):
            return False
    return True


def _line_valid(a, b, circles, rects, block_r):
    for c in circles:
        if _dist_point_to_seg(c, a, b) <= block_r:
            return False
    for rect in rects:
        if _segment_hits_poly(a, b, rect):
            return False
    return True


def _inside_field(p):
    return p[0] >= 0.0 and p[0] <= FIELD_W and p[1] >= 0.0 and p[1] <= FIELD_H


def _distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def _dist_point_to_seg(p, a, b):
    ax = a[0]
    ay = a[1]
    bx = b[0]
    by = b[1]
    dx = bx - ax
    dy = by - ay
    den = dx * dx + dy * dy
    if den == 0.0:
        return _distance(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / den
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    q = (ax + t * dx, ay + t * dy)
    return _distance(p, q)


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a, b, p):
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and
            min(a[1], b[1]) <= p[1] <= max(a[1], b[1]) and
            abs(_cross(a, b, p)) < 0.000001)


def _seg_intersect(a, b, c, d):
    c1 = _cross(a, b, c)
    c2 = _cross(a, b, d)
    c3 = _cross(c, d, a)
    c4 = _cross(c, d, b)
    if c1 * c2 < 0.0 and c3 * c4 < 0.0:
        return True
    if abs(c1) < 0.000001 and _on_segment(a, b, c):
        return True
    if abs(c2) < 0.000001 and _on_segment(a, b, d):
        return True
    if abs(c3) < 0.000001 and _on_segment(c, d, a):
        return True
    if abs(c4) < 0.000001 and _on_segment(c, d, b):
        return True
    return False


def _point_in_poly(p, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        pi = poly[i]
        pj = poly[j]
        if _on_segment(pi, pj, p):
            return True
        if ((pi[1] > p[1]) != (pj[1] > p[1])):
            x = (pj[0] - pi[0]) * (p[1] - pi[1]) / (pj[1] - pi[1]) + pi[0]
            if p[0] < x:
                inside = not inside
        j = i
    return inside


def _segment_hits_poly(a, b, poly):
    if _point_in_poly(a, poly) or _point_in_poly(b, poly):
        return True
    j = len(poly) - 1
    for i in range(len(poly)):
        if _seg_intersect(a, b, poly[j], poly[i]):
            return True
        j = i
    return False


def _smooth_path(path, circles, rects, block_r):
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if _line_valid(path[i], path[j], circles, rects, block_r):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def _path_to_list(path):
    out = []
    for p in path:
        out.append([p[0], p[1]])
    return out


def build_li(circle, retangle, start, end, r=OBSTACLE_R, margin=SAFE_MARGIN):
    return plan_path(circle, retangle, start[0], start[1], end[0], end[1], r, margin)
