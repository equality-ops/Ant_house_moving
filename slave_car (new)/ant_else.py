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
        self.state_apriltag = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待距离低8位, 5:等待距离高8位, 6:等待帧尾
        self.coordinate_buffer = [0, 0, 0, 0, '', 0]
        self.apriltag_buffer = [0, 0, 0, 0, 0, 0, 0]
        self.byte_count = 0

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

    # 用于从车向主车发送当前状态数据的接口
    def send_slave_state(self, state):
        if state == "ready":
            self.my_uart3.write('R'.encode('utf-8'))
        elif state == "lost":
            self.my_uart3.write('L'.encode('utf-8'))
        elif state == "finish":
            self.my_uart3.write('F'.encode('utf-8'))
        elif state == "get":
            self.my_uart3.write('G'.encode('utf-8'))

    # 用于从车解析主车发送的开始信号
    def get_start_signal(self):
        if self.my_uart3.any():
            try:
                byte = self.my_uart3.read(1)[0]
                if byte == ord('S'):
                    byte = self.my_uart3.read(self.my_uart3.any()) # 清空缓冲区
                    return True
            except:
                pass
        return False
    
    # 用于从车解析主车发送的坐标和当前状态数据的接口
    def get_latest_valid_data(self, target_prefix):
        """
        贪婪读取：只返回缓冲区中【最后一个】完整的有效包
        :param target_prefix: 期望读取的包头 ('M' 或 'S')
        :return: [x, y, yaw, turn_angle, state] 或 None
        """
        # 1. 将硬件缓冲区的所有数据读入软件缓冲区
        if self.my_uart3.any():
            try:
                chunk = self.my_uart3.read()
                if chunk:
                    self.raw_buffer += chunk
            except:
                pass # 忽略读取错误
        
        # 如果缓冲区为空，直接返回
        if not self.raw_buffer:
            return None

        # 2. 内存保护：如果堆积太多（比如卡顿了），强制丢弃旧数据，保留最后一部分
        if len(self.raw_buffer) > self.max_buf:
            self.raw_buffer = self.raw_buffer[-self.max_buf:]

        # 3. 寻找包尾 '!' (寻找最后一个，保证最新)
        # rfind 从右边（最新）开始找
        self.end_idx = self.raw_buffer.rfind(b'!')
        
        if self.end_idx == -1:
            return None # 没有完整的包尾，等待下次数据

        # 4. 寻找匹配的包头 '#' (在包尾之前找)
        # 构造目标头，例如 b'#M'
        start_tag = ("#" + target_prefix).encode('utf-8')
        self.start_idx = self.raw_buffer.rfind(start_tag, 0, self.end_idx)

        if self.start_idx == -1:
            # 有尾无头，说明数据错位或头部还在传输中
            # 策略：保留 end_idx 之后的数据（可能是下一个包的开头），前面的全是垃圾
            self.raw_buffer = self.raw_buffer[self.end_idx+1:]
            return None

        # 5. 提取核心负载
        # payload_bytes 如: b',120.5,80.1,90.5,1'
        # start_idx + len(start_tag) 跳过 "#M"
        payload_bytes = self.raw_buffer[self.start_idx + len(start_tag) : self.end_idx]
        
        # 6. 关键一步：消费缓冲区
        # 我们已经拿到了最新的包，end_idx 之前的所有数据（包括旧包）都可以扔掉了
        self.raw_buffer = self.raw_buffer[self.end_idx+1:]
        # 7. 解析数据
        try:
            # 解码并按逗号分割并过滤掉可能的空字符串
            parts = (payload_bytes.decode('utf-8')).strip(',').split(',')
            
            if len(parts) == 5:
                return [float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4])]
            else:
                return None # 字段数量不对（可能是粘包严重导致的残损）
        except:
            return None # 浮点转换失败或解码失败
        

    # 用于从车解析主车发送的路径坐标点
    def get_path_list(self):
        """
        尝试解析主车发送的路径包
        发送格式: #P,120.5,80.1;130.2,90.3!
        :return: 解析成功返回 list [(x1, y1), (x2, y2)...]; 未收到或解析失败返回 None
        """
        # 1. 将硬件缓冲区的所有数据读入软件缓冲区
        if self.my_uart3.any():
            try:
                chunk = self.my_uart3.read()
                if chunk:
                    self.raw_buffer += chunk
            except:
                pass # 忽略读取错误
        
        # 如果缓冲区为空，直接返回
        if not self.raw_buffer:
            return None
        
        # 2. 内存保护：如果堆积太多（比如卡顿了），强制丢弃旧数据，保留最后一部分
        if len(self.raw_buffer) > self.max_buf:
            self.raw_buffer = self.raw_buffer[-self.max_buf:]

        # 3. 寻找包尾 '!'
        self.end_idx = self.raw_buffer.find(b'!')
        if self.end_idx == -1:
            return None # 数据还没传完，继续等待

        # 4. 寻找包头 '#P,'
        start_tag = b'#P,'
        # 只在 end_idx 之前找，防止跨包
        self.start_idx = self.raw_buffer.find(start_tag, 0, self.end_idx)

        if self.start_idx == -1:
            # 有尾巴没头，说明头已经被挤出缓冲区或者数据错乱
            # 清理掉这个废弃的尾巴
            self.raw_buffer = self.raw_buffer[self.end_idx+1:]
            return None

        # 5. 提取中间的纯数据段
        # payload_bytes 如: b'120.5,80.1;130.2,90.3'
        payload_bytes = self.raw_buffer[self.start_idx + len(start_tag) : self.end_idx]
        
        # 6. 消费缓冲区 (清理掉已读取的这个包)
        self.raw_buffer = self.raw_buffer[self.end_idx+1:]

        # 7. 纯字符串解析逻辑
        try:
            payload_str = payload_bytes.decode('utf-8')
            # 此时 payload_str = "120.5,80.1;130.2,90.3"
            
            final_path = []
            
            # 第一刀：按分号切开各个点
            points_str_list = payload_str.split(';')
            
            for p_str in points_str_list:
                # 过滤空字符串 (防止结尾多写了一个分号导致最后是空的)
                if not p_str: continue 
                
                # 第二刀：按逗号切开 x 和 y
                coords = p_str.split(',')
                
                if len(coords) == 2:
                    x = float(coords[0])
                    y = float(coords[1])
                    final_path.append((x, y))
            
            if len(final_path) > 0:
                return final_path
            else:
                return None
                
        except Exception as e:
            # print("解析路径异常:", e)
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
    

 
# 调试程序
"""
if __name__ == "__main__":
    test_strings = ["123", "45.67", "hello", "-89", "3.14159", "world123"]

    # 检测phase_num_string函数
    for s in test_strings:
        result = phase_num_string(s)
        print(f"Input: {s} => Output: {result} (Type: {type(result).__name__})")

    # 检测find_aimed_value函数
    config = phase_config("config.txt")

    print("Parsed Successfully:")
    for key, value in config.items():
        print(f"{key} = {value} (Type: {type(value).__name__})")

    # 检测phase_config函数
    print(f"I want to find 'encouder_l_normal_kp' value: {find_aimed_value(config, 'encouder_l_normal_kp')}")
    print(f"I want to find 'encouder_l_normal_ks' value: {find_aimed_value(config, 'encouder_l_normal_ks')}")
"""