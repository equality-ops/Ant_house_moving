from micropython import const
import time
import gc

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

# 计数�?
counter = 0 
##############################【蜂鸣器�?#############################
BEEP_OFF = const(0)
BEEP_ON = const(1)

class beep:
    def __init__(self, beep):
        # 注入蜂鸣器对�?
        self.beep = beep
        self.beep_state = BEEP_OFF

    # 蜂鸣器警告函�?�?声，�?00ms响一声，每次持续50ms)
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

    # 蜂鸣器测试函�?响一声，持续50ms)
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
        

##############################【uart串口解析数据�?#############################
# 指令管理�?
class order_manager:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
    
    # 切换到目标识别模式（模型�?
    def mode_target(self):
        self.my_uart.write("M")

    # 切换到上下边界识别模�?
    def mode_boundary_ud(self):
        self.my_uart.write("U")

    # 切换到左右边界识别模�?
    def mode_boundary_lf(self):
        self.my_uart.write("L")

    # 切换到apriltag识别模式
    def mode_apriltag(self):
        self.my_uart.write("C")

    def mode_detect(self):
        self.my_uart.write("A")
    # 当前模式结束
    def finish(self):
        self.my_uart.write("F")    

        
# 状态机解析串口数据�?
class UARTProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        self.state_coordinate = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待物体种类, 5:等待帧尾
        self.state_apriltag = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待角度, 5:等待帧尾
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.apriltag_buffer = [0, 0, 0, 0, 0, 0, 0]
        self.byte_count = 0

        self.object_species = [ord('T'),ord('E'),ord('S'),ord('B'),ord('W')]
        self.state_detect_all_objects = 0 # 0:等待帧头1, 1:等待物体数量, 2:等待发送物体讯�? 5:等待帧尾
        self.detect_buffer = [0,[]]
        self.object_buffer = ['',0,0]
        self.state_object = 0 # 0:等待x, 1:等待y, 2:等待物体种类
        gc.collect()
    def clear_uart_buffer(self):
        self.state_coordinate = 0
        self.state_apriltag = 0
        self.state_detect_all_objects = 0
        self.state_object = 0
        self.detect_buffer = [0,[]]
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.object_buffer = ['',0,0]
        self.my_uart.read(self.my_uart.any())#清空缓冲�?
    # 非阻塞接收并解析物体中心的像素点坐标  
    def coordinate_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数�?
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
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返�?
                    x, y = self.coordinate_buffer[2], self.coordinate_buffer[3]
                    last_valid_frame = (x, y, self.coordinate_buffer[4])
                    self.state_coordinate = 0 
                else:
                    self.state_coordinate = 0
                
        # 循环结束后，返回缓冲区里最新的一�?
        return last_valid_frame

    def reset_detect_objects(self):
        self.state_detect_all_objects = 0
        self.state_object = 0
        self.detect_buffer = [0,[]]
        self.object_buffer = ['',0,0]

    def detect_objects_on_the_court(self):
        objects_package = None
        while self.my_uart.any():
            byte = self.my_uart.read(1)[0]
            if self.state_detect_all_objects == 0:
                self.reset_detect_objects()
                if byte == 0x77:
                    self.state_detect_all_objects = 1
            elif self.state_detect_all_objects == 1:#等待物体数量
                if byte>0x00 and byte<=0x10:#物体数量大于0小于等于16
                    self.detect_buffer[0]=byte
                    self.state_detect_all_objects = 2
                else:
                    self.reset_detect_objects()
                    continue
            elif self.state_detect_all_objects == 2:
                if self.state_object == 0:
                    self.object_buffer[1] = byte
                    self.state_object = 1
                elif self.state_object == 1:
                    self.object_buffer[2] = byte
                    self.state_object = 2
                elif self.state_object == 2:
                    if byte in self.object_species:
                        self.object_buffer[0] = byte
                        self.state_object = 0
                        self.detect_buffer[1].append(self.object_buffer[:])
                        if len(self.detect_buffer[1])>=self.detect_buffer[0]:
                            self.state_detect_all_objects = 3
                    else:
                        self.reset_detect_objects()
                        continue
            elif self.state_detect_all_objects == 3:
                if byte == 0x78:
                    objects_package = self.detect_buffer
                self.reset_detect_objects()
                continue
        return objects_package
    # 发送物体种�?
    def send_object_kind(self, object_kind):
        self.my_uart.write(object_kind.lower())
    
    # 非阻塞接收并解析apriltag码的像素点坐标和角度  
    def apriltag_receive(self):
        last_valid_frame = None
        # 持续读取直到处理完当前缓冲区的所有数�?
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
                    # 解析成功，保存当前帧，但【不要】清空缓冲区，【不要】立即返�?
                    last_valid_frame = [self.apriltag_buffer[2], self.apriltag_buffer[3], ((self.apriltag_buffer[5] << 8 | self.apriltag_buffer[4]) / 10 - 90)]
                    self.state_apriltag = 0 
                else:
                    self.state_apriltag = 0
                
        # 循环结束后，返回缓冲区里最新的一�?
        return last_valid_frame

# 主从机通信�?
class LinkProtocol:
    def __init__(self, uart3):
        # 注入串口对象
        self.my_uart3 = uart3
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer�?
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer�?

        gc.collect()

    # 用于主车向从车发送目标物体种类及规划好的路径坐标�?
    def send_path(self, target_object, target_turn, target_point):
        """
        发送路径点列表 (非阻�?
        格式: #P/S/B/T/E/W/A,0.0,120.5,80.1!
        :param target_object: 目标物体种类
        :param target_turn: 目标转向角度
        :param target_point: (x, y) 目标点坐�?
        """
        if isinstance(target_object, int):
            target_object = chr(target_object)
        packet = "#" + target_object + "," + str(target_turn) + "," + "{:.1f},{:.1f}".format(*target_point) + "!"
        self.my_uart3.write(packet.encode('utf-8'))

    def send_orbit_path(self, target_object, target_turn, target_point):
        """
        发送路径点列表 (非阻�?
        格式: #P/S/B/T/E/W/A,0.0,120.5,80.1!
        :param target_object: 目标物体种类
        :param target_turn: 目标转向角度
        :param target_point: (x, y) 目标点坐�?
        """
        packet = "#" + target_object + "," + str(target_turn) + "," + "{:.1f},{:.1f}".format(*target_point) + "!"
        self.my_uart3.write(packet.encode('utf-8'))

    def send_detected_object(self, object_kind, target_point):
        if isinstance(object_kind, int):
            object_kind = chr(object_kind)
        packet = "#D,{},{:.1f},{:.1f}!".format(object_kind, target_point[0], target_point[1])
        self.my_uart3.write(packet.encode('utf-8'))
    # 用于主车向从车发送当前姿�?
    def send_pose(self, v, yaw, turn_angle):
        """
        发送数据包 (非阻�?
        格式: #Z,120.0,0.0,0.0!
        :param v, yaw, turn_angle: 浮点数，分别表示当前速度、航向角和姿态角
        """
        # {:.1f} 保留1位小数足够精度且节省带宽，提高传输频�?
        packet = "#Z,{:.1f},{:.1f},{:.1f}!".format(
            v, yaw, turn_angle
        )
        self.my_uart3.write(packet.encode('utf-8'))
        
    # 用于主车向从车发送环绕角�?
    def send_orbit_angle(self, angle):
        """
        发送环绕角�?(非阻�?
        格式: #O,45.0!
        :param angle: 浮点环绕角度
        """
        packet = "#O,{:.1f}!".format(angle)
        self.my_uart3.write(packet.encode('utf-8'))

    # 向从车发送开始信�?
    def send_start(self):
        self.my_uart3.write('S'.encode('utf-8'))

    def get_slave_state(self):
        """
        解析从车状态包 (非阻�?
        包格�? 'R' (ready), 'L' (lost), 'F' (finish), 'G' (get) 等单字节状态指�?
        :return: 'ready', 'lost', 'finish', 'get' �?None
        """
        if self.my_uart3.any():
            try:
                byte = self.my_uart3.read(1)[0]
                if byte == ord('R'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲�?
                    return "ready"
                elif byte == ord('L'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲�?
                    return "lost"
                elif byte == ord('F'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲�?
                    return "finish"
                elif byte == ord('G'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲�?
                    return "get"
                else:
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲�?
                    return None
            except:
                return None
        else:
            return None


# 主辅助车通信�?
class AssistLinkProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer�?
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer�?

        gc.collect()

    # 用于主车向辅助车发送即将到达的边界信息
    def send_advanced_line(self, line):
        """
        发送状�?(非阻�?
        格式: *line666!
        :param line: 字符串，表示小车即将到达的边界，�?A','B','C','D'�?
        """
        if line == 'U':
            line_temp = 'B'
        elif line == 'L':
            line_temp = 'A'
        elif line == 'R':
            line_temp = 'C'
        else:
            line_temp = 'D'
        packet = "*{}666!".format(line_temp)
        self.my_uart.write(packet.encode('utf-8'))

    # 用于主车向辅助车发送回到线上的消息
    def send_back_message(self):
        """
        发送状�?(非阻�?
        格式: *任意字母888!
        """
        packet = "*A888!"
        self.my_uart.write(packet.encode('utf-8'))

    # 用于主车向辅助车发送搬运到的具体位�?
    def send_target_pos(self, line: str, pos: float):
        """
        发送目标位�?(非阻�?
        格式: *linepos!
        :param line: 字符串，表示边界，如'A','B','C','D'�?
        :param pos: 整数，表示目标位置x/y坐标
        """
        pos_int = int(pos)
        if pos_int < 0:
            pos_int = 0
        
        # 对于 B �?D（如上下边界），限幅�?320
        if line in ('B', 'D') and pos_int > 320:
            pos_int = 320
        # 对于 A �?C（如左右边界），限幅�?240
        elif line in ('A', 'C') and pos_int > 240:
            pos_int = 240

        pos_str = "{:03d}".format(pos_int)
        packet = "*{}{}!".format(line, pos_str)
        self.my_uart.write(packet.encode('utf-8'))

##############################【flash系统操作�?#############################
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
        # 尝试解析为整�?只支持十进制)
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
    
    # 打开参数文件并进行解析，传入一个文件路径，返回一个字�?
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
            # 解析变量�?
            # 如果值以双引号开头和结尾，认为它是一个列表的字符串表示，替换为方括号后使用eval解析为列�?
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

        # 检�?rogue_planning: [[(float, float), str, str, [(str, float), ...]]]
        if "rogue_planning" in self.config:
            rp = self.config["rogue_planning"]
            if type(rp) is not list:
                print("Error: 'rogue_planning' 必须是列?")
                error_flag = True
            else:
                for plan in rp:
                    if type(plan) is not list or len(plan) != 4:
                        print("Error: 'rogue_planning' 中的元素必须是长度为4的列�?")
                        error_flag = True
                        break
                    p_coord, p_kind, p_dir, p_cond = plan
                    if type(p_coord) is not tuple or len(p_coord) != 2 or not all(type(x) in (int, float) for x in p_coord):
                        print("Error: 'rogue_planning' 的坐标格式错误，应为 (float, float)")
                        error_flag = True
                    if type(p_kind) is not str or p_kind not in ('E', 'S', 'B', 'W', 'T'):
                        print("Error: 'rogue_planning' 的物体种类格式错误，应为字符�?(�?'P', 'E', 'S', 'B', 'W', 'T')")
                        error_flag = True
                    if type(p_dir) is not str or p_dir not in ('U', 'D', 'L', 'R'):
                        print("Error: 'rogue_planning' 的方向格式错误，应为字符�?(�?'U', 'D', 'L', 'R')")
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

        # 检�?cube_obstacles: [(float, float, float, float), ...]
        if "cube_obstacles" in self.config:
            co = self.config["cube_obstacles"]
            if type(co) is not list:
                print("Error: 'cube_obstacles' 必须是列�?")
                error_flag = True
            else:
                for obs in co:
                    if type(obs) is not tuple or len(obs) != 4 or not all(type(x) in (int, float) for x in obs):
                        print("Error: 'cube_obstacles' 的元素格式错误，应为包干4个数字的元组 (float, float, float, float)")
                        error_flag = True
                        break

        # 检�?circle: [(float, float), ...]
        if "circle" in self.config:
            cr = self.config["circle"]
            if type(cr) is not list:
                print("Error: circle must be list")
                error_flag = True
            else:
                for cir in cr:
                    if type(cir) is not tuple or len(cir) != 2 or not all(type(x) in (int, float) for x in cir):
                        print("Error: 'circle' 的元素格式错误，应为包干2个数字的元组 (float, float)")
                        error_flag = True
                        break

        if error_flag:
            self.beep.beep_warn()

# 状态机�?
class TaskController:
    def __init__(self,object_plan, beep: beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager: order_manager, art_protocal: UARTProtocol, main_protocol: LinkProtocol, assist_protocol: AssistLinkProtocol):
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
        self.my_assist_protocol = assist_protocol
        self.object_plan = object_plan
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
        self.if_rogue_plan=self.data.if_rogue_plan
        self.navigate_message = []  # 导航信息：目标点坐标和朝�?
        self.slave_navigate_message = []  # 从车导航信息：目标点坐标和朝�?
        self.scan_message = []  # 扫描信息：目标物体位�?
        self.current_object = ''  # 当前目标物体种类
        # 标志�?
        self.if_transitioning = True  # 是否正在进行状态转�?
        self.if_send_path = False  # 是否已经发送路径规划信�?
        self.detected_num = 0
        self.if_send_detect_message = False
        self.last_side = 'D'
        self.retreat_message= (0,0)
        
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
            # 进入准备导航状态，做好路径规划准备和导航信息准�?
            pass
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟�?
            pass
        elif state == SCAN:
            # 进入扫描状态，开始寻找目标物�?
            '''
            if self.my_vision.if_send_order == False:
                # 打开摄像�?
                self.my_order_manager.mode_target()
                self.my_vision.if_send_order = True
            '''
            self.my_vision.reset_analysed_objects()
            self.detected_num = 0
            self.my_order_manager.mode_detect()
            if self.if_rogue_plan:
                self.my_art_protocol.send_object_kind(self.current_object)  # 发送目标物体种类信�?
            self.my_vision.reset_analysed_objects()
            #self.scan_message.append([self.my_car.x_current, self.my_car.y_current])  # 记录扫描状态开始时小车的位置，作为后续判断是否迷路的参�?
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物�?
            pass
        elif state == MOVE:
            # 进入搬运状态，开始搬运物�?
            self.my_plan.reset_navigate()
            self.my_moving.my_photo.reset_photo()
            pass
            # 测试
            # self.my_uart.write(f"state: {self.my_moving.current_state},moving_pt: {self.my_moving.moving_point},angle_buffer: {self.my_moving.angle_buffer}\n")
        elif state == CALIBRATE:
            # 进入校准状态，进行位置或传感器校准
            # 记录小车在哪个边�?
            self.my_vision.car_position = object_to_line_dict.get(self.current_object)
        elif state == ADJUST:
            # 进入调整状态，根据需要进行微�?
            pass
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务�?
            self.my_path.plan_path(self.data.fixed_point[3][0], self.data.fixed_point[3][1], ignore_center_rect=True)  # 规划回起始点的路�?
            self.my_path.ready_path[-1] = self.data.fixed_point[3]
            # 最后插入一个途径点便于计�?
            self.my_path.ready_path.insert(-1, [self.data.fixed_point[3][0], 10.0])
            # self.my_uart.write(f"Path: {self.my_path.ready_path}")  # 测试：打印路径点
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()
        elif state == RETREAT:
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            pass

    def exit(self):
        state = self.my_state.state
        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            self.my_state.state = NAVIGATE  # 直接切换到导航状态
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
            # 退出扫描状态，停止寻找目标物体
            if not self.my_plan.if_finish_navigate:
                self.my_plan.reset_navigate()
                self.my_vision.reset_servo_angle()
                self.my_art_protocol.send_object_kind(self.current_object)
                self.my_state.state = MOVE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                # 如果小车并没有找到物体，直接return
                self.my_plan.reset_navigate()
                self.my_state.state = RETURN
                self.if_transitioning = True  # 退出当前状态，直接回家
                '''
                self.data.current_index += 1  # 跳过当前物体，进入下一个物体的准备导航状�?
                if self.data.current_index >= self.data.total_objects_num:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                else:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                '''
        elif state == SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                self.my_vision.if_finish_servo = False
                self.my_state.state = MOVE
                self.if_transitioning = True
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                # 将openart置为等待模式
                # self.my_order_manager.finish()
                self.my_plan.reset_navigate()
                self.data.current_index += 1  # 跳过当前物体，进入下一个物体的准备导航状�?
                if self.data.current_index >= self.data.total_objects_num:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
                else:
                    self.my_plan.reset_navigate_angle()
                    self.my_state.state = READY_NAVIGATE
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == MOVE:
            # 退出搬运状态，停止搬运动作 
            if self.current_object == 'T':
                self.last_side = 'U'
            elif self.current_object == 'S' or self.current_object == 'E':
                self.last_side = 'L'
            elif self.current_object == 'W' or self.current_object == 'B':
                self.last_side = 'R'
            else:
                self.my_plan.reset_navigate_angle()
                # 如果从车丢失物体直接返回发车区避免浪费时�?
                self.my_state.state = RETURN 
            # 若从车丢失物体，则跳过当前物�?     
            self.data.current_index += 1
            self.my_plan.reset_navigate()
            self.my_plan.reset_navigate_angle()
            self.my_state.state = RETREAT  # 直接切换到校准状�?
            # 此时从车丢失物体
            if self.my_moving.current_state == ADJUST:
                self.my_plan.reset_navigate_angle()
                # 如果从车丢失物体直接返回发车区避免浪费时�?
                self.my_state.state = RETURN 
            # 跳过当前物体
            self.my_moving.reset_move()  # 重置搬运标志
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
        elif state == CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更�?
            self.my_vision.reset_apriltag_calibrate()  # 重置校准标志
            if self.data.current_index >= self.data.total_objects_num:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
            else:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
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
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状�?
            if self.data.current_index >= self.data.total_objects_num:
                self.my_plan.reset_navigate_angle()
                self.my_state.state = RETURN  # 如果所有物体都处理完了，进入返回状�?
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状�?
    
    def handle_ready_navigate(self):

        # 进入准备导航状态，做好路径规划准备和导航信息准�?
        target_x = 160
        target_y = 120
        slave_stop_threshold = 25.0
        scan_threshold = 20
        if self.data.current_index >= self.data.total_objects_num:
            self.my_state.state = RETURN
            self.if_transitioning = True
            return
        # 主车最终目标点
        main_final_pt = []#惯导先到的位�?
        if self.if_rogue_plan:
            target_x = self.data.rogue_planning[self.data.current_index][0][0]
            target_y = self.data.rogue_planning[self.data.current_index][0][1]
            self.current_object = self.data.rogue_planning[self.data.current_index][1]  # 提取当前物体种类信息
            # 便于边线处减�?
            self.my_plan.current_object = self.current_object  
            self.last_side = self.data.rogue_planning[self.data.current_index][2]
            scan_threshold=0
            # 小车导航到物体前的距�?
            stop_threshold = 20.0
            # 根据小车进入的边界信息选择合适的角度和扫描点信息
        
        if self.last_side == "L":
            target_angle = 90.0
            self.slave_navigate_message = [[self.data.fixed_point[1][0] - slave_stop_threshold, target_y+scan_threshold], target_angle]
            main_final_pt = [self.data.fixed_point[1][0], target_y+scan_threshold]
            if self.if_rogue_plan:
                self.scan_message = [[target_x - stop_threshold, target_y]]
            else:
                self.scan_message = [[self.data.fixed_point[1][0], target_y-scan_threshold]]
        elif self.last_side == "R":
            target_angle = -90.0
            self.slave_navigate_message = [[self.data.fixed_point[2][0] + slave_stop_threshold, target_y-scan_threshold], target_angle]
            main_final_pt = [self.data.fixed_point[2][0], target_y-scan_threshold]
            if self.if_rogue_plan:
                self.scan_message = [[target_x + stop_threshold, target_y]]
            else:
                self.scan_message = [[self.data.fixed_point[2][0], target_y+scan_threshold]]
        elif self.last_side == "U":
            target_angle = 180.0
            self.slave_navigate_message = [[target_x+scan_threshold, self.data.fixed_point[2][1] + slave_stop_threshold], target_angle]
            main_final_pt = [target_x+scan_threshold, self.data.fixed_point[2][1]]
            if self.if_rogue_plan:
                self.scan_message = [[target_x, target_y + stop_threshold]]
            else:
                self.scan_message = [[target_x-scan_threshold,self.data.fixed_point[2][1]]]
        else:
            target_angle = 0.0
            self.slave_navigate_message = [[target_x-scan_threshold, self.data.fixed_point[1][1] - slave_stop_threshold], target_angle]
            main_final_pt = [target_x-scan_threshold, self.data.fixed_point[1][1]-5]
            #main_final_pt = [target_x, self.data.fixed_point[1][1]-5]
            if self.if_rogue_plan:
                self.scan_message = [[target_x, target_y - stop_threshold]]
            else:
                self.scan_message = [[target_x+scan_threshold,self.data.fixed_point[1][1]-5]]
         # 进行路径规划
        self.my_path.plan_path(main_final_pt[0], main_final_pt[1])  
        self.navigate_message = [self.my_path.ready_path, target_angle]  # 准备导航信息
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

    def handle_scan(self):
        def analyse_package(num):
            if not self.if_send_detect_message:
                self.if_send_detect_message = True
                self.my_order_manager.mode_detect()
            object_package=self.my_art_protocol.detect_objects_on_the_court()
            if object_package:
                self.my_vision.analyse_object_coordinate(object_package,if_cover = True)
                self.detected_num+=1
                if self.detected_num==num:
                    self.my_order_manager.finish()
                    self.if_send_detect_message = False
                    self.my_uart.write(f"1{self.my_vision.analysed_objects}\n")
        if self.detected_num < 2:
            analyse_package(2)
            self.my_plan.if_finish_navigate = False
        if self.detected_num < 4:
            self.my_plan.navigate(path = self.scan_message)
            if self.my_plan.if_finish_navigate:
                analyse_package(4)
        if self.detected_num == 4:
            self.object_plan.judge_object_character(self.my_vision.analysed_objects,self.last_side)
            target = self.object_plan.find_target()
            self.my_uart.write(f"score{self.object_plan.objects_score}\n")
            self.my_uart.write(f"char{self.object_plan.objects_characters}\n")
            self.my_uart.write(f"pt:{target[5]}\n")
            if not target:
                self.my_uart.write("False\n")
                self.exit()
            else:
                #self.my_uart.write("rm ok\n")
                self.object_plan.barrier.pop(target[0])
                self.my_moving.now_barriar=self.object_plan.barrier[:]
                self.current_object=target[4]
                self.my_plan.current_object = self.current_object
                self.my_vision.current_servo_object = self.current_object
                if target[3]!=self.last_side:rm = self.my_moving.ready_move(target[5],new_side = target[3])
                else:rm = self.my_moving.ready_move(target[5])
                self.my_uart.write(f"nav:{self.my_moving.navigate_buffer}\n")
                if rm:self.my_plan.if_finish_navigate = False
                self.exit()

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            x = self.my_car.x_current
            y = self.my_car.y_current
            self.my_plan.navigate(path = [[x+10.0, y], [x-10.0, y], self.navigate_message[0][-1]])
            
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.my_vision.current_servo_object:
                self.my_vision.ready_servo_and_orbit(target_point, 'servo')
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False

        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状�?

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()
        if self.my_moving.if_finish_move:
            current_object = self.current_object
            retreat_threhold = 5
            if current_object == 'T':
                if self.my_car.now_yaw<0:
                    self.retreat_message=[self.my_car.x_current+retreat_threhold, self.my_car.y_current]
                else:
                    self.retreat_message=[self.my_car.x_current-retreat_threhold, self.my_car.y_current]
            elif current_object in ['S', 'E']:
                if self.my_car.now_yaw<-PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
            elif current_object in ['B', 'W']:
                if self.my_car.now_yaw<PI/2:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current-retreat_threhold]
                else:
                    self.retreat_message=[self.my_car.x_current, self.my_car.y_current+retreat_threhold]
            self.exit()  # 退出当前状态，进入下一个状�?

    def handle_retreat(self):
        # if state == ADJUST
        self.my_plan.navigate(path = [self.retreat_message])

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入下一个状�?

    def handle_calibrate(self):
        # if state == CALIBRATE
        global counter
        self.my_vision.apriltag_calibrate_control()

        if self.my_vision.if_finish_calibrate:
            counter += 1
            # 延时100ms
            if counter >= 10:
                counter = 0
                self.exit()  # 退出当前状态，进入下一个状�?

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
