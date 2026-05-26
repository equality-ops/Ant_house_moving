from micropython import const
import time
import gc


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
    
    # 发送物体种类
    def send_object_kind(self, object_kind):
        self.my_uart.write(object_kind.lower())

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
        格式: #P/S/B/T/E/W,L,120.5,80.1!
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
        格式: #A,120.0,0.0,0.0!
        :param v, yaw, turn_angle: 浮点数，分别表示当前速度、航向角和姿态角
        """
        # {:.1f} 保留1位小数足够精度且节省带宽，提高传输频率
        packet = "#A,{:.1f},{:.1f},{:.1f}!".format(
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


# 主辅助车通信类
class AssistLinkProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer）
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer）

        gc.collect()

    # 用于主车向辅助车发送即将到达的边界信息
    def send_advanced_line(self, line: str):
        """
        发送状态 (非阻塞)
        格式: *line666!
        :param line: 字符串，表示小车即将到达的边界，如'U','D','L','R'等
        """
        packet = "*{}666!".format(line)
        self.my_uart.write(packet.encode('utf-8'))

    # 用于主车向辅助车发送回到线上的消息
    def send_back_message(self):
        """
        发送状态 (非阻塞)
        格式: *任意字母888!
        """
        packet = "*U888!"
        self.my_uart.write(packet.encode('utf-8'))

    # 用于主车向辅助车发送搬运到的具体位置
    def send_target_pos(self, line: str, pos: float):
        """
        发送目标位置 (非阻塞)
        格式: *linepos!
        :param line: 字符串，表示边界，如'U','D','L','R'等
        :param pos: 整数，表示目标位置x/y坐标
        """
        pos_int = int(pos)
        packet = "*{}{}!".format(line, pos_int)
        self.my_uart.write(packet.encode('utf-8'))

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
                self.config[var_name] = eval(var_value) 
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