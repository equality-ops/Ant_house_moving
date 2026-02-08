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

    # 当前模式结束
    def finish(self):
        self.my_uart.write("F")    
        
# 状态机解析串口数据类
class UARTProtocol:
    def __init__(self, uart):
        # 注入串口对象
        self.my_uart = uart
        self.state_coordinate = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待帧尾
        self.state_angle = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待angle, 3:等待帧尾
        self.angle_list = []  # 用于缓存矫正角度信息
        self.coordinate_buffer = [0, 0, 0, 0, 0]
        self.angle_buffer = [0, 0, 0, 0]
        self.byte_count = 0

    # 非阻塞接收并解析物体中心的像素点坐标  
    def coordinate_receive(self):
        while self.my_uart.any():	
            byte = self.my_uart.read(1)[0]
            
            if self.state_coordinate == 0:  # 等待帧头1
                if byte == 0xA5:
                    self.coordinate_buffer[0] = byte
                    self.state_coordinate = 1
                # 如果不是0xA5，继续等待（保持状态0）
                
            elif self.state_coordinate == 1:  # 等待帧头2
                if byte == 0xA6:
                    self.coordinate_buffer[1] = byte
                    self.state_coordinate = 2
                else:
                    self.state_coordinate = 0  # 状态重置
                    
            elif self.state_coordinate == 2:  # 接收x
                self.coordinate_buffer[2] = byte
                self.state_coordinate = 3
                
            elif self.state_coordinate == 3:  # 接收y
                self.coordinate_buffer[3] = byte
                self.state_coordinate = 4
                
            elif self.state_coordinate == 4:  # 等待帧尾
                if byte == 0x5B:
                    self.coordinate_buffer[4] = byte
                    # 完整帧接收完成
                    x, y = self.coordinate_buffer[2], self.coordinate_buffer[3]
                    self.state_coordinate = 0  # 重置状态
                    # 若解析成功清空缓冲区
                    byte = self.my_uart.read(self.my_uart.any()) 
                    return (x, y)
                else:
                    self.state_coordinate = 0  # 帧尾错误，重新同步
        
        return None  # 没有完整帧
    
    # 非阻塞接收并解析边界的斜率
    def angle_receive(self):
        while self.my_uart.any():
            byte = self.my_uart.read(1)[0]

            if self.state_angle == 0:  # 等待帧头1
                if byte == 0xA5:
                    self.angle_buffer[0] = byte
                    self.state_angle = 1
                # 如果不是0xA5，继续等待（保持状态0）
            
            elif self.state_angle == 1:  # 等待帧头2
                if byte == 0xA7:
                    self.angle_buffer[1] = byte
                    self.state_angle = 2
                else:
                    self.state_angle = 0  # 状态重置

            elif self.state_angle == 2:
                self.angle_buffer[2] = byte
                self.state_angle = 3

            elif self.state_angle == 3:
                if byte == 0x5B:
                    self.angle_buffer[3] = byte
                    # 记录接收到的角度值进行解算后到列表中
                    self.angle_list.append(self.angle_buffer[2] - 90)
                    self.state_angle = 0   
                    # 若解析成功清空缓冲区
                    byte = self.my_uart.read(self.my_uart.any())
                else:
                    self.state_angle = 0

        return None       

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
    
    # 用于主车向从车发送坐标和当前状态数据的接口
    def send_pose(self, role_prefix, x, y, yaw, state):
        """
        发送数据包 (非阻塞)
        格式: #M,120.5,80.1,90.5,1!
        :param role_prefix: 'M' (主车) 或 'S' (从车)
        :param x, y, yaw: 浮点坐标
        :param state: 整数状态
        """
        # {:.1f} 保留1位小数足够精度且节省带宽，提高传输频率
        packet = "#{:s},{:.1f},{:.1f},{:.1f},{:d}!".format(
            role_prefix, x, y, yaw, state
        )
        self.my_uart3.write(packet.encode('utf-8'))

    # 用于从车向主车发送当前状态数据的接口
    def send_slave_state(self, state):
        if state == "ready":
            self.my_uart3.write('R'.encode('utf-8'))
        elif state == "lost":
            self.my_uart3.write('L'.encode('utf-8'))
        elif state == "finish":
            self.my_uart3.write('F'.encode('utf-8'))

    # 用于从车解析主车发送的坐标和当前状态数据的接口
    def get_latest_valid_data(self, target_prefix):
        """
        贪婪读取：只返回缓冲区中【最后一个】完整的有效包
        :param target_prefix: 期望读取的包头 ('M' 或 'S')
        :return: [x, y, yaw, state] 或 None
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
            
            if len(parts) == 4:
                return [float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])]
            else:
                return None # 字段数量不对（可能是粘包严重导致的残损）
        except:
            return None # 浮点转换失败或解码失败
        
    def get_slave_state(self):
        """
        解析从车状态包 (非阻塞)
        包格式: 'R' (ready), 'L' (lost), 'F' (finish)
        :return: 'ready', 'lost', 'finish' 或 None
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
    print(f"I want to find 'encouder_l_normal_kp' value: {find_aimed_value(config, "encouder_l_normal_kp")}")
    print(f"I want to find 'encouder_l_normal_ks' value: {find_aimed_value(config, "encouder_l_normal_ks")}")
"""