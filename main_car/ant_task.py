from micropython import const
import gc,math

PI = const(3.1415926)
READY_NAVIGATE = const(0)   # 准备导航状�?
NAVIGATE = const(1)       # 导航状�?
SCAN = const(2)           # 扫描状�?
SERVO = const(3)          # 视觉伺服状�?
ORBIT = const(4)          # 环绕状�?
MOVE = const(5)           # 搬运状�?
CALIBRATE = const(6)      # 校准状�?
ADJUST = const(7)           # 微调状�?
RETURN = const(8)		    # 返回状�?
STOP = const(9)           # 停止状�?
RETREAT = const(10)
object_to_line_dict = {
    'T': 'U',
    'S': 'L',
    'E': 'L',
    'W': 'R',
    'B': 'R'
}
counter = 0 
class TaskController:
    def __init__(self,flash_system,object_plan, beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager, art_protocal, main_protocol,uart_debug):
        # 注入对象
        self.my_beep = beep
        self.my_path = path
        self.my_uart = uart
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car
        self.my_moving = moving
        self.data = plan_data
        self.my_order_manager = order_manager
        self.my_art_protocol = art_protocal
        self.my_main_protocol = main_protocol
        self.object_plan = object_plan
        self.uart_debug = uart_debug
        self.my_flash_system = flash_system
        # 状态映射表：将状态常量映射到对应的处理函�?
        self.handlers = {
            READY_NAVIGATE: self.handle_ready_navigate,
            NAVIGATE: self.handle_navigate,
            SCAN:     self.handle_scan,
            SERVO:    self.handle_servo,
            MOVE:     self.handle_move,
            CALIBRATE: self.handle_calibrate,
            ADJUST:   self.handle_adjust,
            RETURN:    self.handle_return,
            STOP:      self.handle_stop,
            RETREAT: self.handle_retreat,
            # ... 其他状�?
        }
        self.clamp_distance = {'T':1.5,'S':2,'E':2,'W':1,'B':1}
        self.scan_empty_counter = 0
        self.if_rogue_plan=self.data.if_rogue_plan
        self.navigate_message = []  # 导航信息：目标点坐标和朝�?
        self.slave_navigate_message = []  # 从车导航信息：目标点坐标和朝�?
        self.current_object = ''  # 当前目标物体种类
        self.need_calibrate_score = 0
        self.now_objects = []
        # 标志�?
        self.if_transitioning = True  # 是否正在进行状态转�?
        self.if_send_path = False  # 是否已经发送路径规划信�?
        self.detected_num = 0
        self.if_send_detect_message = False
        self.if_model_detect = self.my_flash_system.find_value("if_model_detect")
        if self.if_model_detect:
            self.last_side = 'U'
        else:self.last_side = 'D'
        self.retreat_message= (0,0)
        self.scan_waiting_count = 0
        self.ap_slave_buffer = []
        self.april_tag_list = ['L','U']
        self.planned_scan_path = []
        self.if_plan_scan =False#是否规划出扫描路径
        self.if_end_first_scan = False#是否完成第一次扫描，全局只扫一次
        self.if_first_round = True#是否是第一轮用于判断是否需插入从边线返回途经点
        self.if_choose_object = False#用于判断readynavigate是否成功选择到物体并readymove
        self.need_calibrate_score = 0
        if self.if_model_detect:
            self.use_scan_point = 4
        else:self.use_scan_point = 2
        self.fixed_scan_point = [[[self.my_car.x_current,self.my_car.y_current],0],
                                 [[145,self.data.fixed_point[1][1]-5],0],
                                 [[190,self.data.fixed_point[1][1]-5],0],
                                 [[175,self.data.fixed_point[2][1]+5],180],
                                 [[130,self.data.fixed_point[2][1]+5],180]]
        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作
        
    # 不同模式下的执行函数
    def run(self):
        if self.if_transitioning:
            self.enter()  # 进入新状态执行一次性的进入函数

        # 获取当前状态对应的函数并执�?
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函�?
    def enter(self):
        state = self.my_state.state
        self.if_transitioning = False  # 进入新状态，重置状态转换标志位
        if state == READY_NAVIGATE:
            self.my_plan.reset_navigate_angle()
            self.object_plan.reset_judge()
            self.if_choose_object = False
            # 进入准备导航状态，做好路径规划准备和导航信息准�?
            pass
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟�?
            pass
        elif state == SCAN:
            # 进入扫描状态，开始寻找目标物�?
            self.detected_num = 0
            self.my_plan.reset_navigate()
            self.if_send_detect_message = False
            #self.my_order_manager.mode_detect()
            self.object_plan.reset_judge()
            if self.if_rogue_plan:
                self.my_art_protocol.send_object_kind(self.current_object)  # 发送目标物体种类信�?
            #self.scan_message.append([self.my_car.x_current, self.my_car.y_current])  # 记录扫描状态开始时小车的位置，作为后续判断是否迷路的参�?
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物�?
            pass
        elif state == MOVE:
            # 进入搬运状态，开始搬运物�?
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_moving.my_photo.reset_photo()
            pass
            # 测试
            # self.my_uart.write(f"state: {self.my_moving.current_state},moving_pt: {self.my_moving.moving_point},angle_buffer: {self.my_moving.angle_buffer}\n")
        elif state == CALIBRATE:
            self.my_plan.if_finish_plan = False
        elif state == ADJUST:
            # 进入调整状态，根据需要进行微�?
            pass
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务�?
            self.my_path.plan_path(self.data.fixed_point[3][0], self.data.fixed_point[3][1], ignore_center_rect=True)  # 规划回起始点的路�?
            p1 = [min(max(15,self.my_car.x_current),320-15),min(max(15,self.my_car.y_current),240-15)]
            self.my_path.ready_path[-1] = self.data.fixed_point[3]
            # 最后插入一个途径点便于计�?
            self.my_path.ready_path.insert(-1, [self.data.fixed_point[3][0], 10.0])
            self.my_path.ready_path.insert(0, p1)
            # self.my_uart.write(f"Path: {self.my_path.ready_path}")  # 测试：打印路径点
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()
        elif state == RETREAT:
            self.my_main_protocol.send_path('A',self.ap_slave_buffer[1],self.ap_slave_buffer[0])
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            pass

    def exit(self):
        state = self.my_state.state
        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            self.if_send_path = False  # 重置路径发送标志位
            self.my_plan.reset_navigate()  # 重置导航标志
            if not self.if_end_first_scan:
                if self.if_model_detect:
                    self.my_main_protocol.send_path('P',180.0,(0.0,0.0))
                else:
                    self.my_main_protocol.send_path('P',0.0,(0.0,0.0))
                self.my_state.state = SCAN
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                return
            if not self.if_choose_object:
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                return
            self.my_state.state = MOVE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0]) 
            # 退出导航状态，停止路径跟随
            self.if_send_path = False  # 重置路径发送标志位
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = SCAN  # 直接切换到扫描状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SCAN:
            self.planned_scan_path.clear()
            self.if_plan_scan = False
            # 退出扫描状态，停止寻找目标物体
            if not self.if_end_first_scan:
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，直接回家
                return
            if not self.my_plan.if_finish_navigate:
                self.my_plan.reset_navigate()
                self.my_vision.reset_servo_angle()
                self.my_art_protocol.send_object_kind(self.current_object)
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                # 如果小车并没有找到物体，直接return
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，直接回家
        elif state == SERVO:
            pass
        elif state == MOVE:
            if self.current_object == 'T':self.last_side = 'U'
            elif self.current_object == 'S' or self.current_object == 'E':self.last_side = 'L'
            elif self.current_object == 'W' or self.current_object == 'B':self.last_side = 'R'
            else:
                self.my_plan.reset_navigate_angle()
                # 如果从车丢失物体直接返回发车区避免浪费时�?
                self.my_state.state = RETURN 
            dis = math.sqrt((self.my_car.x_current - self.my_vision.calibrate_buffer[0][0][0])**2 +\
                            (self.my_car.y_current - self.my_vision.calibrate_buffer[0][0][1])**2 )
            score = self.need_calibrate_score - dis * 0.05 
            global counter
            if self.data.current_index >= self.data.total_objects_num or self.my_moving.current_state != NAVIGATE:
                if counter >=40:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                    self.my_moving.reset_move()  # 重置搬运标志
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                    return
                else:counter +=1
            elif self.last_side in self.april_tag_list and self.my_order_manager.if_calibrate and score >= 10:
                self.data.current_index += 1
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETREAT
                self.my_moving.reset_move()  # 重置搬运标志
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
            else:
                if counter >=40:
                    counter=0
                    self.data.current_index += 1
                    self.my_plan.reset_navigate()
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE
                    self.my_moving.reset_move()  # 重置搬运标志
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                else:counter +=1
        elif state == CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更�?
            if self.my_vision.if_lost_object:
                self.my_plan.if_finish_plan = False
                self.my_plan.reset_navigate()
                self.my_plan.reset_navigate_angle()
                self.my_vision.reset_calibrate()  # 重置校准标志
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
            else:
                counter += 1
                # 延时200ms
                if counter >= 20:
                    counter = 0     # 重置计数器
                    self.my_plan.if_finish_plan = False
                    self.my_vision.reset_calibrate()  # 重置校准标志
                    self.my_plan.reset_navigate()
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物?
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状?
        elif state == ADJUST:
            # 退出调整状态，完成微调后进行必要的状态更�?
            self.my_vision.reset_orbit()
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == RETURN:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4]) 
        
            # 退出返回状态，完成返回后进行必要的状态更�?
            self.if_send_path = True
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = STOP  # 直接切换到停止状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状�?
            self.my_beep.test()  # 任务完成，发出提示音
        elif state == RETREAT:
            # 重置导航标志�?
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = CALIBRATE
            if self.data.current_index >= self.data.total_objects_num:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
    def handle_ready_navigate(self):
        if not self.if_end_first_scan:
            self.exit()
            return
        if not self.if_choose_object:
            if self.now_objects:
                if self.object_plan.judge_object_character(self.now_objects,self.last_side):
                    target = self.object_plan.plan_target
                    self.if_end_first_scan = True
                    self.my_uart.write(f"{self.now_objects}\n")
                    self.my_uart.write(f"target{self.object_plan.target_objects}\n")
                    self.my_uart.write(f"path{self.object_plan.path}\n")
                    self.my_uart.write(f"score{self.object_plan.target_score}\n")
                    if not target:
                        #self.my_uart.write("False\n")
                        self.exit()
                    else:
                        self.object_plan.barrier.pop(target[0])
                        self.now_objects.pop(target[0])
                        self.my_moving.now_barriar=self.object_plan.barrier[:]
                        #self.my_uart.write(f"barriar{self.my_moving.now_barriar}\n")
                        self.current_object=target[1]
                        self.my_plan.current_object = self.current_object
                        self.my_vision.current_servo_object = self.current_object
                        rm = self.my_moving.ready_move([target[2],target[3]],now_side = self.last_side)
                        # self.my_uart.write(f"car_position:{self.my_moving.push_postion}\n")
                        #self.my_uart.write(f"rm:{rm},nav_n:{len(self.my_moving.navigate_buffer)}\n")
                        if rm:
                            self.my_moving.saved_best_path =self.object_plan.best_path
                            num_compensation = self.data.current_index * 0.2
                            self.my_moving.clamp_distance = self.clamp_distance[self.current_object]+num_compensation
                            self.if_choose_object = True
                            self.my_plan.reset_navigate()
                        else:self.exit()
            else:self.exit()
        else:
            if self.data.current_index >= self.data.total_objects_num:
                self.my_state.state = RETURN
                self.if_transitioning = True
                return
            # 进入准备导航状态，做好路径规划准备和导航信息准�?
            slave_stop_threshold = 25.0
            planned_path = self.my_moving.navigate_buffer['MAIN_P']
            insert_point = []
            if self.last_side == "L":
                target_angle = 90.0
                self.slave_navigate_message = [[planned_path[-2][0] - slave_stop_threshold, planned_path[-2][1]], target_angle]
                if self.if_first_round:self.if_first_round = False
                else:insert_point = [self.my_car.x_current+15,self.my_car.y_current]
            elif self.last_side == "R":
                target_angle = -90.0
                self.slave_navigate_message = [[planned_path[-2][0] + slave_stop_threshold,planned_path[-2][1]], target_angle]
                if self.if_first_round:self.if_first_round = False
                else:insert_point = [self.my_car.x_current-15,self.my_car.y_current]
            elif self.last_side == "U":
                target_angle = 180.0
                self.slave_navigate_message = [[planned_path[-2][0], planned_path[-2][1] + slave_stop_threshold], target_angle]
                if self.if_first_round:self.if_first_round = False
                else:insert_point = [self.my_car.x_current,self.my_car.y_current-15]
            else:
                target_angle = 0.0
                self.slave_navigate_message = [[planned_path[-2][0], planned_path[-2][1] - slave_stop_threshold], target_angle]
                if self.if_first_round:self.if_first_round = False
                else:insert_point = [self.my_car.x_current,self.my_car.y_current+15]
            # 进行路径规划
            self.my_moving.slave_massage['path'] = self.slave_navigate_message [0]
            self.my_moving.slave_massage['angle'] = self.slave_navigate_message [1]
            if insert_point:planned_path = [insert_point] + planned_path
            self.my_moving.navigate_buffer['MAIN_P'] = planned_path
            self.exit()  # 退出当前状态，进入导航状�?
    def handle_navigate(self):
        # if state == NAVIGATE
        self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])
        # 主车行驶多远后给从车发送路径信�?
        dist_threshold = 20.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信�?
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状�?
            
    # 处理物体信息（将像素坐标转换为世界坐标）
    def handle_object_info(self, ob_info,angle):
        """将单帧物体列表的像素坐标转换为世界坐标，返回新列表"""
        real_ob_info = []
        for ob in ob_info[1]:
            sp, x, y  = ob
            if x<10 or y<10 or x>145 or y >105:
                continue
            kind = chr(sp)
            # 更新当前物体种类，便于选择物体高度
            self.my_vision.current_servo_object = kind
            if self.if_model_detect:  
                if angle == 180:limit_y = 50
                else:limit_y = 75
            else: limit_y = None
            real_point = self.my_vision.predict_point(x, y,limit_y = limit_y)
            if not real_point: continue
            if not self.my_vision.if_in_rect(real_point[0],real_point[1]):continue
            real_ob_info.append((kind,real_point[0], real_point[1]))
        self.my_vision.current_servo_object = ''  # 重置当前物体种类
        return real_ob_info
    def merge_nearby_same_kind(self,objects, threshold_near=10.0):
        merged = []
        threshold_far = threshold_near+5
        for kind, x, y in objects:
            match_idx = -1
            for idx, (old_kind, old_x, old_y) in enumerate(merged):
                if old_kind != kind:
                    continue
                object_dist = max(abs(y - self.my_car.y_current),abs(old_y - self.my_car.y_current))
                if object_dist <= 30.0 or self.if_model_detect:
                    threshold = threshold_near
                elif object_dist >= 110.0:
                    threshold = threshold_far
                else:
                    ratio = (object_dist - 30.0) / 80.0
                    threshold = threshold_near + (threshold_far - threshold_near) * ratio
                dist2 = (x - old_x) ** 2 + (y - old_y) ** 2
                if dist2 <= threshold ** 2:
                    match_idx = idx
                    break
            if match_idx < 0:
                merged.append((kind, x, y))
            else:
                old_kind, old_x, old_y = merged[match_idx]
                merged[match_idx] = (
                    old_kind,
                    (old_x + x) / 2.0,
                    (old_y + y) / 2.0,
                )
        return merged
    # 合并物体信息（双目视觉融合）
    def integrate_object_info(self,world_1,world_2):
        # 同一物体在两个扫描中的最大世界坐标偏差（cm）
        # 包含：测量噪声 ~5cm + 60cm 基线的视差效应 ~30cm + 安全裕量
        MATCH_DIST_THRESHOLD = 10.0
        # ── 2. 边界情况：任一侧为空则直接返回另一侧 ──
        if not world_1 and not world_2:
            return []
        if not world_1:
            return world_2[:]
        if not world_2:
            return world_1[:]
        groups_1 = {}
        groups_2 = {}
        for i, (kind,x, y) in enumerate(world_1):
            groups_1.setdefault(kind, []).append((i, x, y))
        for i, (kind,x, y) in enumerate(world_2):
            groups_2.setdefault(kind, []).append((i, x, y))
        matched_pairs = []
        used_1 = set()
        used_2 = set()
        all_kinds = set(groups_1.keys()) | set(groups_2.keys())
        for kind in all_kinds:
            objs_1 = groups_1.get(kind, [])
            objs_2 = groups_2.get(kind, [])
            if not objs_1 or not objs_2:
                continue
            candidates = []
            for i1, x1, y1 in objs_1:
                for i2, x2, y2 in objs_2:
                    d = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                    if d <= MATCH_DIST_THRESHOLD:
                        candidates.append((d, i1, i2))
            candidates.sort(key=lambda t: t[0])
            for d, i1, i2 in candidates:
                if i1 not in used_1 and i2 not in used_2:
                    matched_pairs.append((i1, i2))
                    used_1.add(i1)
                    used_2.add(i2)
        ob_info = []
        for i1, i2 in matched_pairs:
            kind, x1, y1,  = world_1[i1]
            _, x2, y2 = world_2[i2]
            ob_info.append((kind,(x1 + x2) / 2.0, (y1 + y2) / 2.0))
        for i in range(len(world_1)):
            if i not in used_1:
                ob_info.append(world_1[i])
        for i in range(len(world_2)):
            if i not in used_2:
                ob_info.append(world_2[i])  
        return ob_info
    def first_scan(self):
        def analyse_package(num,angle):
            global counter
            object_package=self.my_art_protocol.detect_objects_on_the_court()#[物体种类(ord),x,y]
            if object_package:
                counter +=1
                self.scan_empty_counter=0
                new_world = self.handle_object_info(object_package,angle)
                #self.my_uart.write(f"{counter}{new_world}\n")
                if self.now_objects: self.now_objects = self.integrate_object_info(self.now_objects,new_world)#将新帧与上一帧融合
                else: self.now_objects = new_world
                self.my_vision.analysed_objects = self.now_objects
            else:
                self.scan_empty_counter+=1
                if self.scan_empty_counter>40:
                    self.my_plan.reset_navigate()
                    self.scan_waiting_count = 0 
                    self.scan_empty_counter = 0
                    self.my_order_manager.finish()
                    self.if_send_detect_message = False
                    self.if_plan_scan = False
                    counter = num#直接退出
            if counter == num:
                self.detected_num+=1#切换到下一个物体
                counter = 0
                self.scan_waiting_count = 0
                self.my_plan.reset_navigate()
                self.if_plan_scan = False
                self.my_order_manager.finish()
                self.if_send_detect_message = False
                #self.my_uart.write(f"{num}{self.my_vision.analysed_objects}\n")
                self.my_art_protocol.clear_uart_buffer()
        def scan_point(num):#输入帧数
            self.my_plan.navigate(path = self.planned_scan_path[self.detected_num][0],
                                  target_turn_angle = self.planned_scan_path[self.detected_num][1])
            if self.my_plan.if_finish_navigate:
                if self.scan_waiting_count < 10:
                    if not self.if_send_detect_message:
                        self.scan_empty_counter=0
                        counter = 0
                        self.if_send_detect_message = True
                        self.my_art_protocol.clear_uart_buffer()
                        self.my_order_manager.clear_knock()
                        self.my_order_manager.mode_detect()
                        if self.if_model_detect:
                            self.my_order_manager.trans_to_mode_detect()
                    self.scan_waiting_count +=1
                else:analyse_package(num,self.planned_scan_path[self.detected_num][1])
        if self.detected_num == self.use_scan_point:
            self.now_objects = self.merge_nearby_same_kind(self.now_objects)
            '''if len(self.now_objects) != self.data.total_objects_num:
                self.my_uart.write(f"{self.now_objects}\n")
                self.my_uart.write(f"target{self.object_plan.target_objects}\n")
                self.my_uart.write(f"path{self.object_plan.path}\n")
                self.my_uart.write(f"score{self.object_plan.target_score}\n")
                self.exit()
                return'''
            self.if_end_first_scan = True
        else:scan_point(1)
    def handle_scan(self):
        global counter
        if not self.if_end_first_scan:
            if not self.if_plan_scan:
                if counter >= self.use_scan_point:
                    self.planned_scan_path[0][0].insert(0,[self.my_car.x_current,self.my_car.y_current+30])
                    self.if_plan_scan = True
                    counter = 0
                    return
                self.my_path.plan_path(self.fixed_scan_point[counter+1][0][0],self.fixed_scan_point[counter+1][0][1],start_point = self.fixed_scan_point[counter][0]) 
                self.planned_scan_path.append([self.my_path.ready_path,self.fixed_scan_point[counter+1][1]])
                counter+=1
            else:self.first_scan()
        else:self.exit()
    def handle_servo(self):
        pass
    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()
        if self.my_moving.if_finish_move:
            current_object = self.current_object
            retreat_threhold = 10
            ap_threhold = 25
            self.my_vision.reset_calibrate()
            if current_object == 'T':
                self.need_calibrate_score += 3
                self.my_vision.car_position = 'U'
                if self.my_car.now_yaw<0:
                    self.retreat_message=[self.my_car.x_current+retreat_threhold, self.my_car.y_current]
                    set_angle = -90
                    ap_pos = self.my_vision.apriltage_postion[self.my_vision.car_position]
                    path = [[ap_pos[0]+ap_threhold,ap_pos[1]]]
                    self.ap_slave_buffer = [[ap_pos[0]-ap_threhold,ap_pos[1]],90]
                    if self.my_car.x_current+retreat_threhold >= path[0][0]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                else:
                    self.retreat_message=[self.my_car.x_current-retreat_threhold, self.my_car.y_current]
                    set_angle = 90
                    ap_pos = self.my_vision.apriltage_postion[self.my_vision.car_position]
                    path = [[ap_pos[0]-ap_threhold,ap_pos[1]]]
                    self.ap_slave_buffer = [[ap_pos[0]+ap_threhold,ap_pos[1]],-90]
                    if self.my_car.x_current-retreat_threhold <= path[0][0]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
            elif current_object in ['S', 'E']:
                self.need_calibrate_score += 4
                self.my_vision.car_position = 'L'
                if self.my_car.now_yaw<-PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
                    set_angle = 180
                    ap_pos = self.my_vision.apriltage_postion[self.my_vision.car_position]
                    path = [[ap_pos[0],ap_pos[1]+ap_threhold]]
                    self.ap_slave_buffer = [[ap_pos[0],ap_pos[1]-ap_threhold],0]
                    if self.my_car.y_current+retreat_threhold >= path[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
                    set_angle = 0
                    ap_pos = self.my_vision.apriltage_postion[self.my_vision.car_position]
                    path = [[ap_pos[0],ap_pos[1]-ap_threhold]]
                    self.ap_slave_buffer = [[ap_pos[0],ap_pos[1]+ap_threhold],180]
                    if self.my_car.y_current-retreat_threhold <= path[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
            else:
                self.need_calibrate_score += 3
                self.my_vision.car_position = 'R'
                if self.my_car.now_yaw<PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
                    set_angle = 0
                    ap_pos = self.my_vision.apriltage_postion[self.my_vision.car_position]
                    path = [[ap_pos[0],ap_pos[1]-ap_threhold]]
                    self.ap_slave_buffer = [[ap_pos[0],ap_pos[1]+ap_threhold],180]
                    if self.my_car.y_current-retreat_threhold <= path[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
                    set_angle = 180
                    ap_pos = self.my_vision.apriltage_postion[self.my_vision.car_position]
                    path = [[ap_pos[0],ap_pos[1]+ap_threhold]]
                    self.ap_slave_buffer = [[ap_pos[0],ap_pos[1]-ap_threhold],0]
                    if self.my_car.y_current+retreat_threhold >= path[0][1]:
                        self.my_vision.if_waiting = True
                    else:self.my_vision.if_waiting = False
            self.my_vision.calibrate_buffer = [path,set_angle]
            self.exit()  # 退出当前状态，进入下一个状�?
    def handle_retreat(self):
        self.my_plan.navigate(path = [self.retreat_message])
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状态
    def handle_calibrate(self):
        if self.my_vision.if_finish_calibrate:
            self.exit()
            return
        
        if self.my_vision.if_lost_object == False:
            self.my_vision.apriltag_calibrate_control()
        else:
            # 控制小车前后移动寻找apriltag码
            self.my_plan.navigate([ [self.my_car.x_current- 15.0, self.my_car.y_current],
                                    [self.my_car.x_current- 15.0, self.my_car.y_current-15.0], 
                                    [self.my_car.x_current+ 15.0, self.my_car.y_current-15.0], 
                                    [self.my_car.x_current+ 15.0, self.my_car.y_current+15.0]])

            target_point = self.my_art_protocol.apriltag_receive()
            if target_point:    
                self.my_plan.reset_navigate()
                self.my_vision.counter = 0
                self.my_vision.calibrate_times = 0
                self.my_vision.if_lost_object, self.my_vision.if_gain_calibrate_angle = False, False
            
            if self.my_plan.if_finish_navigate:
                self.exit()

    def handle_adjust(self):
        pass

    def handle_return(self):
        # if state == RETURN
        self.my_plan.navigate(path = self.my_path.ready_path)  # 返回起始�?
        # 主车行驶多远后给从车发送路径信�?
        dist_threshold = 50.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信�?
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状�?
    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()             
