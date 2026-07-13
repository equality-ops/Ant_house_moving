from micropython import const
import time
import math
import gc

PI = const(3.1415926)
READY_NAVIGATE = const(0) # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)         # 微调状态
RETURN = const(8)		  # 返回状态
STOP = const(9)           # 停止状态
PREDICT = const(10)       # 预测状态

InField = const(-1)
OnLine = const(0)
OutLine = const(1)

# 根据物体位置列举的三种情形
ALL_IN_BOTTOM = const(0)  # 物体完全在下区域内
ONE_IN_TOP = const(2)     # 物体有一个在上区域内
OVER_ONE_IN_TOP = const(4)  # 物体有两个或以上在上区域内


# 计数器
counter = 0 
##############################【蜂鸣器】##############################
BEEP_OFF = const(0)
BEEP_ON = const(1)

class beep:
    def __init__(self, beep):
        # 注入蜂鸣器对象
        self.beep = beep
        self.beep_state = BEEP_OFF

    # 蜂鸣器警告函数(响3声，每500ms响一声，每次持续50ms)
    def beep_warn(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            for i in range(3):
                time.sleep_ms(50)
                self.beep.high()
                time.sleep_ms(50)
                self.beep.low()
                time.sleep_ms(200)
                self.beep_state = BEEP_OFF
            return 
        elif self.beep_state == BEEP_ON:
            return 
        
    # 按键测试函数(响一声，持续80ms)
    def key_test(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            self.beep.high()
            time.sleep_ms(80)
            self.beep.low()
            self.beep_state = BEEP_OFF
            return
        elif self.beep_state == BEEP_ON:
            return

    # 蜂鸣器测试函数(响一声，持续50ms)
    def test(self) -> None:
        if self.beep_state == BEEP_OFF:
            self.beep_state = BEEP_ON
            self.beep.high()
            time.sleep_ms(50)
            self.beep.low()
            self.beep_state = BEEP_OFF
            return
        elif self.beep_state == BEEP_ON:
            return
        

##############################【uart串口解析数据】##############################
# 指令管理类
class order_manager:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
    
    # 切换到目标识别模式（模型）
    def mode_target(self):
        self.my_uart.write("M")

    # 切换到上下边界识别模式
    def mode_boundary_ud(self):
        self.my_uart.write("U")

    # 切换到左右边界识别模式
    def mode_boundary_lf(self):
        self.my_uart.write("L")

    # 切换到apriltag识别模式
    def mode_apriltag(self):
        self.my_uart.write("C")

    # 当前模式结束
    def finish(self):
        self.my_uart.write("F")    

        
# 状态机解析串口数据类
class UARTProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        self.state_coordinate = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待物体种类, 5:等待帧尾
        self.state_apriltag = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待角度, 5:等待帧尾
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.apriltag_buffer = [0, 0, 0, 0, 0, 0, 0]
        self.byte_count = 0

        gc.collect()
    
    # 清空缓冲区数据
    def clear_buffer(self):
        lenth = self.my_uart.any()
        if lenth > 0:
            self.my_uart.read(lenth) 
        
    # 发送物体种类
    def send_object_kind(self, object_kind = None):
        if object_kind:
            self.my_uart.write(object_kind.lower())
        else:
            self.my_uart.write('c') # 表示当前没有需要锁定的物体种类

    # 非阻塞接收并解析物体中心的像素点坐标  
    def coordinate_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数据
        while self.my_uart.any():	
            byte = self.my_uart.read(1)[0]
            
            if self.state_coordinate == 0:
                if byte == 0xA5:
                    self.state_coordinate = 1
            elif self.state_coordinate == 1:
                if byte == 0xA6:
                    self.state_coordinate = 2
                else:
                    self.state_coordinate = 0
            elif self.state_coordinate == 2:
                self.coordinate_buffer[2] = byte
                self.state_coordinate = 3
            elif self.state_coordinate == 3:
                self.coordinate_buffer[3] = byte
                self.state_coordinate = 4
            elif self.state_coordinate == 4:
                self.coordinate_buffer[4] = byte
                self.state_coordinate = 5
            elif self.state_coordinate == 5:
                if byte == 0x5B:
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返回
                    x, y = self.coordinate_buffer[2], self.coordinate_buffer[3]
                    last_valid_frame = (x, y, self.coordinate_buffer[4])
                    self.state_coordinate = 0 
                else:
                    self.state_coordinate = 0
                
        # 循环结束后，返回缓冲区里最新的一帧
        return last_valid_frame
    
    # 非阻塞接收并解析apriltag码的像素点坐标和角度  
    def apriltag_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数据
        while self.my_uart.any():	
            byte = self.my_uart.read(1)[0]
            
            if self.state_apriltag == 0:
                if byte == 0xA5:
                    self.state_apriltag = 1
            elif self.state_apriltag == 1:
                if byte == 0xA8:
                    self.state_apriltag = 2
                else:
                    self.state_apriltag = 0
            elif self.state_apriltag == 2:
                self.apriltag_buffer[2] = byte
                self.state_apriltag = 3
            elif self.state_apriltag == 3:
                self.apriltag_buffer[3] = byte
                self.state_apriltag = 4
            elif self.state_apriltag == 4:
                self.apriltag_buffer[4] = byte
                self.state_apriltag = 5
            elif self.state_apriltag == 5:
                self.apriltag_buffer[5] = byte
                self.state_apriltag = 6
            elif self.state_apriltag == 6:
                if byte == 0x5B:
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返回
                    last_valid_frame = [self.apriltag_buffer[2], self.apriltag_buffer[3], ((self.apriltag_buffer[5] << 8 | self.apriltag_buffer[4]) / 10 - 90)]
                    self.state_apriltag = 0 
                else:
                    self.state_apriltag = 0
                
        # 循环结束后，返回缓冲区里最新的一帧
        return last_valid_frame

# 主从机通信类
class LinkProtocol:
    def __init__(self, uart3):
        # 注入串口对象
        self.my_uart3 = uart3
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer）
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer）

        gc.collect()

    # 用于主车向从车发送目标物体种类及规划好的路径坐标点
    def send_path(self, target_object, target_turn, target_point):
        """
        发送路径点列表 (非阻塞)
        格式: #P/S/B/T/E/W/A,0.0,120.5,80.1!
        :param target_object: 目标物体种类
        :param target_turn: 目标转向角度
        :param target_point: (x, y) 目标点坐标
        """
        packet = "#" + target_object + "," + str(target_turn) + "," + "{:.1f},{:.1f}".format(*target_point) + "!"
        self.my_uart3.write(packet.encode('utf-8'))

    # 用于主车向从车发送当前姿态
    def send_pose(self, v, yaw, turn_angle):
        """
        发送数据包 (非阻塞)
        格式: #Z,120.0,0.0,0.0!
        :param v, yaw, turn_angle: 浮点数，分别表示当前速度、航向角和姿态角
        """
        # {:.1f} 保留1位小数足够精度且节省带宽，提高传输频率
        packet = "#Z,{:.1f},{:.1f},{:.1f}!".format(
            v, yaw, turn_angle
        )
        self.my_uart3.write(packet.encode('utf-8'))
        
    # 用于主车向从车发送环绕角度
    def send_orbit_angle(self, angle):
        """
        发送环绕角度 (非阻塞)
        格式: #O,45.0!
        :param angle: 浮点环绕角度
        """
        packet = "#O,{:.1f}!".format(angle)
        self.my_uart3.write(packet.encode('utf-8'))

    # 向从车发送开始信息
    def send_start(self):
        self.my_uart3.write('S'.encode('utf-8'))

    def get_slave_state(self):
        """
        解析从车状态包 (非阻塞)
        包格式: 'R' (ready), 'L' (lost), 'F' (finish), 'G' (get) 等单字节状态指令
        :return: 'ready', 'lost', 'finish', 'get' 或 None
        """
        if self.my_uart3.any():
            try:
                byte = self.my_uart3.read(1)[0]
                if byte == ord('R'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲区
                    return "ready"
                elif byte == ord('L'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲区
                    return "lost"
                elif byte == ord('F'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲区
                    return "finish"
                elif byte == ord('G'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲区
                    return "get"
                else:
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲区
                    return None
            except:
                return None
        else:
            return None


##############################【flash系统操作】##############################
class flash_system:
    def __init__(self, beep, file_path: str):
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        # 传入文件路径
        self.file_path = file_path  # type: str
        # 创建变量字典
        self.config = dict()

        gc.collect()

    # 将字符串解析为整数或浮点数，如果无法解析则返回原始字符串
    def phase_num_string(self, s: str):
        # 尝试解析为整数(只支持十进制)
        try:
            value = int(s, 10)
            return value
        except ValueError:
            pass

        # 尝试解析为浮点数
        try:
            value = float(s)
            return value
        except ValueError:
            pass

        # 如果无法解析为数字，则返回原始字符串
        return s
    
    # 打开参数文件并进行解析，传入一个文件路径，返回一个字典
    def phase_config(self) -> None:
        try:
            f = open(self.file_path, 'r')
        except FileNotFoundError as e:
            print(e)
            print(f"Error: File {self.file_path} not found.")
        content = f.readlines()
        for line in content:
            # 跳过空行和注释行
            if not line or line.startswith('#') or line.startswith('\r\n'):
                continue
            line = line.strip()
            line = line.split('=', 1)
            var_name = line[0].strip()
            var_value = line[1].strip()
            # 解析变量值
            # 如果值以双引号开头和结尾，认为它是一个列表的字符串表示，替换为方括号后使用eval解析为列表
            if var_value[0] == '"' and var_value[-1] == '"':  # 列表类型
                var_value = "[" + var_value[1:-1] + "]"
                try:
                    self.config[var_name] = eval(var_value)
                except Exception as e:
                    print(f"Error: Failed to evaluate {var_name} = {var_value}")
                    self.beep.beep_warn()
            else:
                self.config[var_name] = self.phase_num_string(var_value)
        f.close()


    def find_value(self, var_name: str):
        try:
            var_value = self.config[var_name.strip()]
            return var_value
        except KeyError as e:
            print(f"Failure to find {var_name.strip()} in {self.file_path}!")
            self.beep.beep_warn()
            return 0
            
    def check_list_format(self) -> None:
        """检查特定的列表与其内部变量格式是否正确"""
        error_flag = False

        # 检查 rogue_planning: [[(float, float), str, str, [(str, float), ...]]]
        if "rogue_planning" in self.config:
            rp = self.config["rogue_planning"]
            if type(rp) is not list:
                print("Error: 'rogue_planning' 必须是列表")
                error_flag = True
            else:
                for plan in rp:
                    if type(plan) is not list or len(plan) != 4:
                        print("Error: 'rogue_planning' 中的元素必须是长度为4的列表")
                        error_flag = True
                        break
                    p_coord, p_kind, p_dir, p_cond = plan
                    if type(p_coord) is not tuple or len(p_coord) != 2 or not all(type(x) in (int, float) for x in p_coord):
                        print("Error: 'rogue_planning' 的坐标格式错误，应为 (float, float)")
                        error_flag = True
                    if type(p_kind) is not str or p_kind not in ('E', 'S', 'B', 'W', 'T'):
                        print("Error: 'rogue_planning' 的物体种类格式错误，应为字符串 (如 'P', 'E', 'S', 'B', 'W', 'T')")
                        error_flag = True
                    if type(p_dir) is not str or p_dir not in ('U', 'D', 'L', 'R'):
                        print("Error: 'rogue_planning' 的方向格式错误，应为字符串 (如 'U', 'D', 'L', 'R')")
                        error_flag = True
                    if type(p_cond) is not list:
                        print("Error: 'rogue_planning' 的条件格式错误，应为列表")
                        error_flag = True
                    else:
                        for cond in p_cond:
                            if type(cond) is not tuple or len(cond) != 2 or type(cond[0]) is not str or type(cond[1]) not in (int, float):
                                print("Error: 'rogue_planning' 中条件元素格式错误，应为 (str, float)")
                                error_flag = True
                                break

        # 检查 cube_obstacles: [(float, float, float, float), ...]
        if "cube_obstacles" in self.config:
            co = self.config["cube_obstacles"]
            if type(co) is not list:
                print("Error: 'cube_obstacles' 必须是列表")
                error_flag = True
            else:
                for obs in co:
                    if type(obs) is not tuple or len(obs) != 4 or not all(type(x) in (int, float) for x in obs):
                        print("Error: 'cube_obstacles' 的元素格式错误，应为包干4个数字的元组 (float, float, float, float)")
                        error_flag = True
                        break

        # 检查 circle: [(float, float), ...]
        if "circle" in self.config:
            cr = self.config["circle"]
            if type(cr) is not list:
                print("Error: 'circle' 必须是列表")
                error_flag = True
            else:
                for cir in cr:
                    if type(cir) is not tuple or len(cir) != 2 or not all(type(x) in (int, float) for x in cir):
                        print("Error: 'circle' 的元素格式错误，应为包干2个数字的元组 (float, float)")
                        error_flag = True
                        break

        if error_flag:
            self.beep.beep_warn()

# 状态机类
class TaskController:
    def __init__(self, beep: beep, fan, photo, state, uart3, uart8, car, plan, vision, plan_data, order_manager: order_manager, art_protocal: UARTProtocol, main_protocol: LinkProtocol):
        # 注入对象
        self.my_beep = beep
        self.my_fan = fan
        self.my_photo = photo
        self.my_uart3 = uart3
        self.my_uart8 = uart8
        self.my_plan = plan
        self.my_vision = vision
        self.my_state = state
        self.my_car = car
        self.data = plan_data
        self.my_order_manager = order_manager
        self.my_art_protocol = art_protocal
        self.my_main_protocol = main_protocol

        # 状态映射表：将状态常量映射到对应的处理函数
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
            ORBIT:     self.handle_orbit,
            PREDICT:   self.handle_predict,
            # ... 其他状态
        }

        self.navigate_message = []  # 导航信息：目标点坐标和朝向
        self.scan_message = []  # 扫描路径信息
        self.slave_navigate_message = []  # 从车导航信息：目标点坐标和朝向
        self.move_message = []  # 搬运目标点
        self.adjust_message = []  # 微调目标点
        self.predict_message = []  # 预测目标点(根据第一帧图像预测的)
        self.orbit_angle_buf = 0.0  # 环绕角度缓冲区
        self.scan_angle_buf = 0.0  # 自转角度缓冲区
        self.current_object = ''  # 当前目标物体种类
        self.object_status = ALL_IN_BOTTOM  # 当前物体位置状态
        # 标志位
        self.if_start_off = False  # 是否出发车区
        self.if_transitioning = True  # 是否正在进行状态转换
        self.if_send_path = False  # 是否已经发送路径规划信息
        self.if_to_top = False  # 是否前往上区域
        self.the_last_one = False  # 是否是最后一个物体
        self.if_second_verify = False  # 是否进行第二次验证视觉
        self.if_change_status = False  # 是否完成任务状态切换
        self.if_skip_orbit = False  # 是否跳过环绕模式

        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作

    
    # 重置小车里程计
    def reset_car_pos(self):
        # 经验修正值
        correction = 2.0
        if self.object_status in [ALL_IN_BOTTOM, ONE_IN_TOP]:
            self.my_car.y_current = 0.0 + correction
        elif self.object_status == OVER_ONE_IN_TOP:
            if self.the_last_one:
                self.my_car.y_current = 0.0 + correction
            else:
                self.my_car.y_current = 240.0 - correction

    # 修改物体状态
    def change_object_status(self):
        def change():
            self.if_change_status = True
            if self.object_status == ALL_IN_BOTTOM:
                if self.data.finished_num == self.data.total_objects_num - 1:
                    self.object_status = ONE_IN_TOP  # 最后一个物体在上区域内
                else:
                    self.object_status = OVER_ONE_IN_TOP  # 还有一个或以上物体在上区域内
        
        if self.if_change_status:
            return

        if self.my_plan.aimed_point_index >= 1:
            change()


    # 不同模式下的执行函数
    def run(self):
        if self.if_transitioning:
            self.enter()  # 进入新状态执行一次性的进入函数

        # 获取当前状态对应的函数并执行
        handler = self.handlers.get(self.my_state.state)
        if handler:
            handler()

    # 模式之间的进入和退出函数
    def enter(self):
        state = self.my_state.state
        self.if_transitioning = False  # 进入新状态，重置状态转换标志位

        if state == READY_NAVIGATE:
            # 进入准备导航状态，做好路径规划准备和导航信息准备
            pass
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟随
            pass
        elif state == SCAN:
            # 进入扫描状态，开始寻找目标物体
            # 清空视觉串口缓冲区，准备接收新数据
            self.my_art_protocol.clear_buffer()
            # 此时扫描模式不锁定物体
            self.my_art_protocol.send_object_kind()
            # 打开目标识别模式
            self.my_order_manager.mode_target() 
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物体
            # 清空视觉串口缓冲区，准备接收新数据
            self.my_art_protocol.clear_buffer()
        elif state == ORBIT:
            # 进入环绕状态，开始环绕目标物体
            if self.object_status in [ALL_IN_BOTTOM, ONE_IN_TOP]:
                # 主车顺时针旋转
                self.orbit_angle_buf = self.my_vision.orbit_angle
            elif self.object_status == OVER_ONE_IN_TOP:
                if self.the_last_one or self.if_to_top == False:
                    self.if_skip_orbit = True
                    self.my_vision.skip_orbit()  # 跳过环绕模式
                    if self.if_to_top == False:
                        # 此时小车将过渡到上半区
                        self.if_to_top = True
                else:
                    # 主车逆时针旋转
                    self.orbit_angle_buf = -180 + self.my_vision.orbit_angle

        elif state == MOVE: 
            if self.object_status in [ALL_IN_BOTTOM, ONE_IN_TOP]:
                # 搬运到-20的点保证光电管能检测到边界
                self.move_message = [self.my_car.x_current + self.my_plan.error_x, -20.0]
            elif self.object_status == OVER_ONE_IN_TOP:
                if self.the_last_one:
                    # 搬运到-20的点保证光电管能检测到边界
                    self.move_message = [self.my_car.x_current + self.my_plan.error_x, -20.0]
                else:
                    # 搬运到260的点保证光电管能检测到边界
                    self.move_message = [self.my_car.x_current - self.my_plan.error_x, 260.0]

            # 测试
            # self.my_uart.write(f"{self.move_message}\n")
        elif state == CALIBRATE:
            # 进入校准状态，进行位置或传感器校准
            # 记录小车在哪个边线
            # self.my_vision.car_position = object_to_line_dict.get(self.current_object)
            pass
        elif state == ADJUST:
            # 进入调整状态，根据需要进行微调
            if self.my_car.y_current < 120.0:
                self.adjust_message = [self.my_car.x_current - 5.0, self.my_car.y_current]
            else:
                self.adjust_message = [self.my_car.x_current + 5.0, self.my_car.y_current]
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务点
            pass
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()
        elif state == PREDICT:
            # 进入预测状态，进行目标位置预测
            pass

    def exit(self):
        global counter
        state = self.my_state.state

        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            self.my_state.state = NAVIGATE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0]) 
                self.if_send_path = True  # 设置路径发送标志位，避免重复发送
            
            if not self.if_start_off:
                self.if_start_off = True  # 已经出发车区

            self.my_plan.reset_navigate()  # 重置导航状态
            self.my_plan.reset_navigate_angle()  # 重置导航角度
            # 退出导航状态，停止路径跟随
            self.if_send_path = False  # 重置路径发送标志位
            self.my_state.state = SCAN  # 直接切换到扫描状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SCAN:
            # 退出扫描状态，停止寻找目标物体
            if not self.my_plan.if_finish_navigate:
                # 更新扫描起始点横坐标（缩短用时）
                if self.object_status == ALL_IN_BOTTOM:
                    self.data.scan_point[0][0] = self.my_car.x_current 
                elif self.object_status == OVER_ONE_IN_TOP and self.if_to_top:
                    self.data.scan_point[0][0] = self.my_car.x_current 

                self.if_change_status = False  # 重置物体状态切换标志位
                self.my_plan.reset_navigate()

                if self.object_status == OVER_ONE_IN_TOP and (not self.if_to_top or self.the_last_one):
                    # 重置二次验证标志位
                    self.if_second_verify = False
                    self.my_plan.if_second_verify = False
                    
                    x_threshold = 20.0
                    y_threshold = 20.0
                    rich_angle = 8.0  # 角度裕量
                    if self.if_to_top == False:
                        self.spin_angle_buf = -180 + self.my_vision.orbit_angle + rich_angle
                        slave_angle = 180 - self.my_vision.orbit_angle - rich_angle
                        
                        # 让从车停靠在主车左侧伺服后不环绕直接往上边界搬运
                        self.my_main_protocol.send_path(self.current_object, slave_angle, [self.predict_message[0] - x_threshold, self.predict_message[1] - y_threshold])   
                        
                        self.predict_message[0] += x_threshold
                        self.predict_message[1] -= y_threshold
                    elif self.the_last_one:
                        self.spin_angle_buf = self.my_vision.orbit_angle + rich_angle
                        slave_angle = -self.my_vision.orbit_angle - rich_angle

                        # 让从车停靠在主车左侧伺服后不环绕直接往下边界搬运    
                        self.my_main_protocol.send_path(self.current_object, slave_angle, [self.predict_message[0] + x_threshold, self.predict_message[1] + y_threshold])
                        
                        self.predict_message[0] -= x_threshold
                        self.predict_message[1] += y_threshold

                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = PREDICT
                else:
                    self.my_vision.reset_servo_angle()
                    self.my_state.state = SERVO

                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                self.my_plan.reset_navigate()
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 直接返回
        elif state == SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                counter += 1
                # 延时100ms
                if counter <= 10:
                    return

                stop_threshold = 25.0
                if self.object_status in [ALL_IN_BOTTOM, ONE_IN_TOP]:
                    # 发送路径信息给从车
                    self.my_main_protocol.send_path(self.current_object, self.slave_navigate_message[1], [self.my_car.x_current, self.my_car.y_current - stop_threshold])   
                else:   # OVER_ONE_IN_TOP的情况
                    if self.if_to_top and not self.the_last_one:
                        self.my_main_protocol.send_path(self.current_object, self.slave_navigate_message[1], [self.my_car.x_current, self.my_car.y_current + stop_threshold])

                # 重置计数器
                counter = 0
                self.if_send_path = False  # 重置路径发送标志位
                self.my_vision.if_finish_servo = False  # 重置伺服完成标志
                self.my_vision.reset_orbit_angle()
                self.my_state.state = ORBIT  # 直接切换到环绕状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == PREDICT:
            self.my_plan.reset_navigate()
            self.my_vision.reset_servo_angle()
            # 更新小车的上一帧位置便于预测
            self.my_vision.reset_last_car_pos()
            self.my_state.state = SERVO  # 直接切换到伺服状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == ORBIT:
            # 此时关闭负压风扇
            # self.my_fan.fan_off()
            order = self.my_main_protocol.get_slave_state()
            if order == "finish":
                if self.if_skip_orbit:
                    self.my_main_protocol.send_start()  # 让从车开始搬运
                    self.if_skip_orbit = False
                self.my_vision.reset_orbit()  # 重置环绕标志
                self.my_plan.reset_navigate_angle()
                self.my_state.state = MOVE  # 直接切换到搬运状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif order == "lost":
                # 回城时重新打开负压
                # self.my_fan.set_fan_signal()
                self.my_vision.reset_orbit()  # 重置环绕标志
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 直接切换到返回状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == MOVE:
            # 退出搬运状态，停止搬运动作 
            # 若从车丢失物体，则跳过当前物体   
            counter += 1
            # 延时400ms
            if counter >= 40:   
                # 重置计数器
                counter = 0
                self.data.finished_num += 1
                if self.data.finished_num >= self.data.total_objects_num:
                    self.my_plan.reset_navigate_angle()
                    # 若搬运完成物体直接返回，则进入返回状态
                    self.my_state.state = RETURN 
                else:
                    if self.data.finished_num == self.data.total_objects_num - 1:
                        # 现在是最后一个物体
                        self.the_last_one = True
                    
                    self.my_plan.reset_navigate_angle()
                    # 若搬运完成物体继续处理下一个物体，则进入准备导航状态
                    self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态
                    # 测试
                    # self.my_state.state = STOP
                    # self.my_uart3.write(f"{self.my_car.x_current},{self.my_car.y_current}\n")

                # 重置导航标志位
                self.my_plan.reset_navigate()
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更新
            pass
        elif state == ADJUST:
            # 重置导航标志位
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == RETURN:
            if not self.if_send_path:
                # 发送路径信息给从车
                self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4]) 
        
            # 退出返回状态，完成返回后进行必要的状态更新
            self.if_send_path = True
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = STOP  # 直接切换到停止状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状态
            self.my_beep.test()  # 任务完成，发出提示音
    
    def handle_ready_navigate(self):
        target_point, target_angle = [], 0.0
        stop_threshold = 25.0  # 从车停止点与主车的距离阈值
        if self.object_status in [ALL_IN_BOTTOM, ONE_IN_TOP] or (self.object_status == OVER_ONE_IN_TOP and not self.if_to_top):
            target_angle = 0.0
            target_point = self.data.scan_point[0]
            self.scan_message = self.data.scan_path_1
            self.slave_navigate_message = [[160.0, target_point[1] - stop_threshold], target_angle]
        else:
            target_angle = 180.0
            target_point = self.data.scan_point[1]
            self.scan_message = self.data.scan_path_2
            self.slave_navigate_message = [[160.0, target_point[1] + stop_threshold], target_angle]

        self.navigate_message = [[target_point], target_angle]  # 准备导航信息

        if self.if_start_off == False:
            # 先让小车走到y=10的位置保证顺利发车
            self.navigate_message[0].insert(0, [self.my_car.x_current, self.my_car.y_current + 30.0]) 

        self.exit()  # 退出当前状态，进入导航状态

    def handle_navigate(self):
        # if state == NAVIGATE
        self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])
        
        # 主车行驶多远后给从车发送路径信息
        dist_threshold = 15.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('P', self.slave_navigate_message[1], self.slave_navigate_message[0])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信息

        # 当小车经过第一个目标点后切换到扫描模式开始扫描
        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状态
   
    def handle_scan(self):
        global counter
        # if state == SCAN
        self.my_plan.navigate(path = self.scan_message, target_turn_angle = self.navigate_message[1])  # 导航到扫描终点
        
        # 根据当前位于的扫描途径点修改任务状态
        self.change_object_status()

        if_pos_rational = ((self.my_car.x_current <= 140.0 or self.my_car.x_current >= 180.0) and self.my_plan.aimed_point_index == 0)\
        or (self.my_plan.aimed_point_index != 0)
        
        target_point = self.my_art_protocol.coordinate_receive()
        if target_point and if_pos_rational:
            # 更新当前伺服物体种类
            self.my_vision.current_servo_object = chr(target_point[2])  
            # 判断目标点是否合理，若不合理则清空当前伺服物体种类    
            if self.my_vision.judge_if_object_rational(target_point[0], target_point[1]):
                if self.object_status == OVER_ONE_IN_TOP and (not self.if_to_top or self.the_last_one):
                    if not self.if_second_verify:
                        self.my_plan.if_second_verify = True
                        counter += 1
                        # 只有连续扫到3帧目标体才认为是有效目标
                        if counter >= 3:
                            counter = 0
                            # 清空串口缓冲区
                            self.my_art_protocol.clear_buffer()
                            self.if_second_verify = True
                    else:
                        self.current_object = chr(target_point[2])
                        self.my_art_protocol.send_object_kind(self.current_object)  # 发送目标物体种类openart
                        self.predict_message = self.my_vision.predict_point(target_point[0], target_point[1])
                        # 清空串口缓冲区
                        self.my_art_protocol.clear_buffer()
                        self.my_vision.ready_servo_and_orbit(target_point, 'adjust')
                        self.exit()  # 退出当前状态
                else:
                    self.current_object = chr(target_point[2])
                    self.my_art_protocol.send_object_kind(self.current_object)  # 发送目标物体种类openart
                    self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                    self.exit()  # 退出当前状态
                return
            else:
                self.my_vision.current_servo_object = ''  # 清空当前伺服物体种类

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入伺服状态
            return

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            x = self.my_car.x_current
            y = self.my_car.y_current
            now_yaw = self.my_car.now_yaw  # 弧度，0=北(+Y)，90°=东(+X)
            # 车身右方(+X): (cos(now_yaw), -sin(now_yaw))
            # 车身左方(-X): (-cos(now_yaw), sin(now_yaw))
            right_x = x + 15.0 * math.cos(now_yaw)
            right_y = y - 15.0 * math.sin(now_yaw)
            left_x = x - 15.0 * math.cos(now_yaw)
            left_y = y + 15.0 * math.sin(now_yaw)
            self.my_plan.navigate(path = [[right_x, right_y], [left_x, left_y]])
            
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.my_vision.current_servo_object:
                if self.object_status == OVER_ONE_IN_TOP and (not self.if_to_top or self.the_last_one):
                    self.my_vision.ready_servo_and_orbit(target_point, 'adjust')
                else:
                    self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False

        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_orbit(self):
        # if state == ORBIT
        self.my_vision.orbit_control(self.orbit_angle_buf)

        if self.my_vision.if_finish_orbit:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_predict(self):
        # if state == PREDICT
        self.my_plan.navigate(path = [self.predict_message], target_turn_angle = self.spin_angle_buf)  # 导航到预测的目标点

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_move(self):
        # if state == MOVE
        # 更新光电管状态
        self.my_photo.update_photo_state()
        if self.my_photo.current_state == OutLine:
            self.reset_car_pos()
            self.my_photo.reset_photo()
            # 测试
            self.my_beep.test()
            self.my_plan.if_finish_navigate = True

        self.my_plan.navigate(path = [self.move_message])  # 导航到搬运目标点

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_calibrate(self):
        # if state == CALIBRATE
        pass

    def handle_adjust(self):
        # if state == ADJUST
        self.my_plan.navigate(path = [self.adjust_message])

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_return(self):
        # if state == RETURN
        path = [[self.data.fixed_point[3][0], 20.0], self.data.fixed_point[3]]  # 先回到y=20.0的安全位置再返回起始点
        # 若小车y坐标小于20.0则先平移到y=20.0处再返回防止打到光电门
        if self.my_car.y_current < 20.0:
            path.insert(0, [self.my_car.x_current, 20.0])

        self.my_plan.navigate(path = path)  # 返回起始点
                
        # 主车行驶多远后给从车发送路径信息
        dist_threshold = 15.0
        if self.my_plan.finished_dist >= dist_threshold and not self.if_send_path:
            self.my_main_protocol.send_path('R', 999, self.data.fixed_point[4])  # 发送路径信息给从车
            self.if_send_path = True  # 设置标志位，避免重复发送路径信息

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状态

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()             