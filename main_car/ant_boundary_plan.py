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
        if swell_angle == 1 or swell_angle== -1:swell_size = 10.0
        else:swell_size = 20.0
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

        def swell_rect(rect,swell_angle):
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
                elif swell_angle == 1:
                    if y < rect[2][1] - 0.001:
                        y -= swell_size
                    elif y > rect[0][1] + 0.001:
                        y += swell_size
                elif swell_angle == -1:
                    if x < rect[0][0] + 0.001:
                        x -= swell_size
                    elif x > rect[0][0] + 0.001:
                        x += swell_size
                out.append((x, y))
            return out

        for obj in objects:
            if len(obj) >= 4:
                cx, cy = float(obj[0]), float(obj[1])
                half_w = float(obj[2]) / 2.0
                half_h = float(obj[3]) / 2.0
                rects.append(swell_rect(make_rect(cx, cy, half_w, half_h),swell_angle))

        for circle in circles:
            if len(circle) >= 2:
                cx, cy = float(circle[0]), float(circle[1])
                rects.append(swell_rect(make_rect(cx, cy, circle_r, circle_r),swell_angle))

        for rect in raw_rects:
            if len(rect) >= 4:
                rects.append(swell_rect(rect,swell_angle))
        return rects

    def plan_move(self, direction, swell_dir, objects, x=None, y=None):
        self.rects = self.special_swell_barriers(objects, swell_dir)
        self.ready_path = self.plan_one_turn(direction,x,y)
        return self.ready_path

    def plan_one_turn(self, direction,x=None,y=None):
        if x is None or y is None:x,y=self.my_car.x_current,self.my_car.y_current
        path_left = self._plan_one_turn_with_avoid(direction, -1,x,y)
        path_right = self._plan_one_turn_with_avoid(direction, 1,x,y)
        if not path_left:return path_right
        if not path_right:return path_left
        if self._path_cost(path_left) <= self._path_cost(path_right):return path_left
        return path_right

    def _plan_one_turn_with_avoid(self, direction, avoid_dir,x,y):
        direction = self._normalize_dir(direction)
        avoid_dir = 1 if avoid_dir >= 0 else -1
        start = (float(x), float(y))
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
    
class objects_planner:
    def __init__(self, plan_data, car, my_plan, my_BoundaryPath : BoundaryPathPlanner):
        self.Data = plan_data
        self.my_car = car
        self.my_plan = my_plan
        self.objects = []
        self.objects_information = []
        self.objects_characters = []
        self.barrier = []
        self.my_BoundaryPath = my_BoundaryPath
        self.near_therohold = 15
        self.objects_score = []
        gc.collect()
    def set_barriers(self):
        barriers=[]
        wideness={'T':4,'S':3,'E':3,'B':2,'W':2,}
        height={'T':4,'S':3,'E':3,'B':2,'W':2,}
        for keyi in self.objects:
            w,h=wideness[keyi],height[keyi]
            for i in self.objects[keyi]:
                    barriers.append([i[0],i[1],w,h])
        return barriers
    def calculate_score(self,_characters,car_side):
        danger,speed = 0,0
        side = {'D':[0,0],'L':[0,1],'U':[1,1],'R':[1,0]}
        if not _characters['has_push_path']:danger += 1
        danger *=10
        if _characters['push_angle']>60:danger += 1
        danger *=10
        for i in _characters['needed_area_barriers']:
            if len(_characters['needed_area_barriers'][i])>0:
                danger+=4
                if 'T' in _characters['needed_area_barriers'][i]:
                    danger+=2
        R_Bs=_characters['run_area_barriers']
        min_dis = 10
        min_dis_T = 10
        Plan_side,Plan_side_T,target_side= car_side,car_side,car_side
        for i in R_Bs:
            dis = abs(side[i][0]-side[car_side][0])+abs(side[i][1]-side[car_side][1])
            if not R_Bs[i]:
                if min_dis>dis:
                    min_dis = dis
                    Plan_side = i
            else:
                if not 'T' in R_Bs[i]:
                    if min_dis_T>dis:
                        min_dis_T = dis
                        Plan_side_T = i
        target_side=Plan_side
        if min_dis == 10:
            if min_dis_T==10:
                danger += 9
            else:
                danger += 5
                speed+=dis*50
            target_side=Plan_side_T
        else:speed+=dis*50
        speed += _characters['distance_from_car']*1+_characters['push_distance']*1
        return danger,speed,target_side#危险性，速度性，目标�?    
    def judge_object_character(self,objects,car_side):
        self.objects = objects
        self.barrier = self.set_barriers()
        idx=0
        self.objects_characters = []
        self.objects_score = []
        for keyi in self.objects:
            for i in self.objects[keyi]:
                now_barrier=self.barrier[:]
                now_barrier.pop(idx)
                needed_area_barrier = self.judge_need_area(keyi)
                run_area_barriers = {'U':[],'D':[],'R':[],'L':[]}
                dir,sdir=self.judge_push_direction(keyi)
                path = self.my_BoundaryPath.plan_move(dir,sdir,now_barrier,i[0],i[1])
                push_distance,push_angle,has_push_path= 1000,90,True
                if len(path) <= 2: has_push_path = False
                else:
                    p_,p__=path[0],path[1]
                    push_distance = self.calculate_distance(i,p_)+self.calculate_distance(p_,p__)
                    push_yaw = math.atan2(p_[0] - i[0], p_[1] - i[1]) * 180.0 / math.pi
                    push_angle = abs(push_yaw - dir)
                    if push_angle > 180:
                        push_angle = 360 - push_angle
                for keyj in self.objects:
                    for j in self.objects[keyj]:
                        if i!=j:
                            if self.calculate_distance(i,j)<=self.near_therohold:
                                judge_ = self.judge_UDRL_area(i,j)
                                if judge_ in needed_area_barrier:
                                    needed_area_barrier[judge_].append(keyj)
                            self.judge_running_area(i,j,run_area_barriers,keyj)
                distance_from_car = self.calculate_distance(i,[self.my_car.x_current,self.my_car.y_current])
                _characters={
                    'species':keyi,
                    'needed_area_barriers': needed_area_barrier,
                    'run_area_barriers': run_area_barriers,
                    'has_push_path':has_push_path,
                    'push_distance':push_distance,
                    'push_angle':push_angle,
                    'distance_from_car':distance_from_car,
                }
                _danger,_speed,_side=self.calculate_score(_characters,car_side)
                self.objects_score.append([idx,_danger,_speed,_side,keyi,i])
                self.objects_characters.append(_characters)
                idx+=1
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
        if dy!=0 and (dx<5 or abs(dx-5)/abs(dy)<=0.5):
            if dy>0:barriar['D'].append(sp)
            else:barriar['U'].append(sp)
        if dx!=0 and (dy<5 or abs(dy-5)/abs(dx)<=0.5):
            if dy>0:barriar['L'].append(sp)
            else:barriar['R'].append(sp)
