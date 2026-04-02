import time

##############################【蜂鸣器】##############################
class beep:
    def __init__(self, beep):
        # 注入蜂鸣器对象
        self.beep = beep
        self.BEEP_OFF = 0
        self.BEEP_ON = 1
        self.beep_state = self.BEEP_OFF

    # 蜂鸣器警告函数(响3声，每500ms响一声，每次持续50ms)
    def beep_warn(self) -> None:
        if self.beep_state == self.BEEP_OFF:
            self.beep_state = self.BEEP_ON
            for i in range(3):
                time.sleep_ms(50)
                self.beep.high()
                time.sleep_ms(50)
                self.beep.low()
                time.sleep_ms(400)
                self.beep_state = self.BEEP_OFF
            return 
        elif self.beep_state == self.BEEP_ON:
            return 
        
    def key_test(self) -> None:
        if self.beep_state == self.BEEP_OFF:
            self.beep_state = self.BEEP_ON
            self.beep.high()
            time.sleep_ms(100)
            self.beep.low()
            self.beep_state = self.BEEP_OFF
            return
        elif self.beep_state == self.BEEP_ON:
            return

    # 蜂鸣器测试函数(响一声，持续50ms)
    def test(self) -> None:
        if self.beep_state == self.BEEP_OFF:
            self.beep_state = self.BEEP_ON
            self.beep.high()
            time.sleep_ms(50)
            self.beep.low()
            self.beep_state = self.BEEP_OFF
            return
        elif self.beep_state == self.BEEP_ON:
            return


##############################【uart串口解析数据】##############################
# 指令管理类
class order_manager:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
    
    # 切换到目标识别模式
    def mode_target(self):
        self.my_uart.write("T")

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

    # 向openart发送物体种类信息
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
    
    # 用于主车向从车发送目标物体种类及规划好的路径坐标点
    def send_path(self, target_object, path_points):
        """
        发送路径点列表 (非阻塞)
        格式: #P/S/B/T/E/W,120.5,80.1;130.2,90.3;140.0,100.0!  #P,160.0,50.0!
        :param target_object: 目标物体种类
        :param path_points: [(x1, y1), (x2, y2), ...]
        """
        point_strs = ["{:.1f},{:.1f}".format(x, y) for x, y in path_points]
        packet = "#" + chr(target_object) + "," + ";".join(point_strs) + "!"
        self.my_uart3.write(packet.encode('utf-8'))

    # 用于主车向从车发送坐标和当前状态数据的接口
    def send_pose(self, role_prefix, x, y, yaw, turn_angle, state):
        """
        发送数据包 (非阻塞)
        格式: #M,120.5,80.1,90.5,20.0,1!
        :param role_prefix: 'M' (主车) 或 'S' (从车)
        :param x, y, yaw: 浮点坐标
        :param state: 整数状态
        """
        # {:.1f} 保留1位小数足够精度且节省带宽，提高传输频率
        packet = "#{:s},{:.1f},{:.1f},{:.1f},{:.1f},{:d}!".format(
            role_prefix, x, y, yaw, turn_angle, state
        )
        self.my_uart3.write(packet.encode('utf-8'))
        
    # 向从车发送开始信息
    def send_start(self):
        self.my_uart3.write('S'.encode('utf-8'))

    # 像从车发送跳过扫描apriltag码的信息
    def send_pass_message(self):
        self.my_uart3.write('P'.encode('utf-8'))

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

# 数学常量类
class Math:
    def __init__(self):
        self.PI = 3.1415926      # type: float
        self.SIN30 = 0.5000000   # type: float
        self.SIN60 = 0.8660254   # type: float
        self.COS30 = 0.8660254   # type: float
        self.COS60 = 0.5000000   # type: float
        self.TwoThirdS = 0.6666667 # type: float
        self.OneThird = 0.3333333  # type: float
        self.SQRT3 = 1.7320508   # type: float


##############################【flash系统操作】##############################
class flash_system:
    def __init__(self, beep, file_path: str):
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        # 传入文件路径
        self.file_path = file_path  # type: str
        # 创建变量字典
        self.config = dict()

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
            if var_value[0] == '"' and var_value[-1] == '"':  # 列表类型
                lst = list(var_value)
                lst[0] = '['
                lst[-1] = ']'
                var_value = ''.join(lst)
                self.config[var_name] = var_value
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
    
    def gain_rogue_planning(self):
        try:
            self.config['rogue_planning'] = eval(self.config['rogue_planning'])
            return self.config['rogue_planning']
        except Exception as e:
            print(f"Failure to find rogue_planning in {self.file_path}!")
            self.beep.beep_warn()
            return 0