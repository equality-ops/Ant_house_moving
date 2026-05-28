from micropython import const
import time
import gc

READY_NAVIGATE = const(0)   # 准备导航状态
NAVIGATE = const(1)       # 导航状态
SCAN = const(2)           # 扫描状态
SERVO = const(3)          # 视觉伺服状态
ORBIT = const(4)          # 环绕状态
MOVE = const(5)           # 搬运状态
CALIBRATE = const(6)      # 校准状态
ADJUST = const(7)           # 微调状态
RETURN = const(8)		    # 返回状态
STOP = const(9)           # 停止状态

object_to_line_dict = {
    'T': 'U',
    'S': 'L',
    'E': 'L',
    'W': 'R',
    'B': 'R'
}
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

        gc.collect()

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

    # 切换到apriltag识别模式
    def mode_apriltag(self):
        self.my_uart.write("C")

    # 切换到搬运检查模式
    def mode_pickup_check(self):
        self.my_uart.write("D")

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

        gc.collect()

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
    
    # 接收来自openart的搬运过程中是否丢失物体的信息
    def get_object_state(self):
        if self.my_uart.any():
            try:
                byte = self.my_uart.read(1)[0]
                if byte == ord('N'):
                    return "No"
                else:
                    return None
            except:
                return None
        else:
            return None

# 主从机通信类
class LinkProtocol:
    def __init__(self, uart3):
        # 注入串口对象
        self.my_uart3 = uart3
        # 当前要伺服的物体
        self.aimed_object = ''
        # 创建字节流缓冲区
        self.raw_buffer = b''           
        self.max_buf = 128         # 缓冲区最大长度，防止内存泄漏
        self.start_idx = 0          # 上次成功解析后剩余数据的起始索引（相对于raw_buffer）
        self.end_idx = 0            # 上次成功解析后剩余数据的结束索引（相对于raw_buffer）

        gc.collect()

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
        
    # 解析主车发送的路径信息
    def get_path_list(self):
            """
            解析主车发送的任务路径包
            发送格式: #T,90.0,120.5,80.1!  (或 #S, #B, #P, #E, #W，#M，#A，#R，中间包含转向角度等)
            :return: 成功返回 [task_type, target_turn, (x, y)], 如 ['T', 90.0, (120.5, 80.1)]
                    失败返回 None
            """
            # 1. 填充缓冲区 (保持原样)
            if self.my_uart3.any():
                try:
                    chunk = self.my_uart3.read()
                    if chunk:
                        self.raw_buffer += chunk
                except:
                    pass 
            
            if not self.raw_buffer:
                return None
            
            # 2. 内存保护 (保持原样)
            if len(self.raw_buffer) > self.max_buf:
                self.raw_buffer = self.raw_buffer[-self.max_buf:]

            # 3. 寻找包尾 '!'
            self.end_idx = self.raw_buffer.find(b'!')
            if self.end_idx == -1:
                return None 

            # 4. 寻找包头 (核心修改点：支持多种包头)
            # 逻辑：先找 '#'，再判断后面是不是符合 T, S, B 格式
            self.start_idx = self.raw_buffer.find(b'#', 0, self.end_idx)

            if self.start_idx == -1:
                # 没找到包头，清理掉无效的尾部之前的数据
                self.raw_buffer = self.raw_buffer[self.end_idx+1:]
                return None

            # 检查包头格式是否完整（# 后面至少要有 "X," 三个字节）
            if self.start_idx + 2 >= self.end_idx:
                # 数据不完整，继续等待
                return None

            # 提取标识符 (T, S, B 或 P)
            try:
                # 这里的 tag_type 是 bytes 类型，转成 string 方便后续判断
                tag_type = self.raw_buffer[self.start_idx + 1 : self.start_idx + 2].decode('utf-8')
                
                # 如果不是我们预期的指令，说明可能是脏数据
                if tag_type not in ['T', 'S', 'B', 'P', 'E', 'W', 'M', 'A', 'R']:
                    # 这种情况下，丢弃这个错误的开头，继续找下一个
                    self.raw_buffer = self.raw_buffer[self.start_idx + 1:]
                    return None
                    
            except:
                return None

            # 5. 提取中间的纯数据段
            # 跳过 "#X," 这三个字节，直到 "!" 之前
            payload_bytes = self.raw_buffer[self.start_idx + 3 : self.end_idx]
            
            # 6. 消费缓冲区
            self.raw_buffer = self.raw_buffer[self.end_idx+1:]

            # 7. 纯字符串解析逻辑
            try:
                payload_str = payload_bytes.decode('utf-8')
                
                # 按照逗号分割，应该得到比如 ['L', '120.5', '80.1'] 这样的3个元素
                parts = payload_str.split(',')
                if len(parts) == 3:
                    target_turn = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    
                    # 【关键点】返回类型：物体种类，转向，目标坐标
                    return [tag_type, target_turn, (x, y)]
                else:
                    return None
                    
            except Exception as e:
                return None

    # 解析主车发送的当前姿态信息
    def get_main_pose(self):
        # 1. 填充缓冲区
        if self.my_uart3.any():
            try:
                chunk = self.my_uart3.read()
                if chunk:
                    self.raw_buffer += chunk
            except:
                pass

        if not self.raw_buffer:
            return None

        # 内存保护
        if len(self.raw_buffer) > self.max_buf:
            self.raw_buffer = self.raw_buffer[-self.max_buf:]

        # 2. 寻找包头 '#A,' 和包尾 '!'
        start_idx = self.raw_buffer.find(b'#A,')
        if start_idx == -1:
            # 没找到需要的包头，清理掉无关的数据，防止内存越界
            # 只保留最后的边界，以免刚好读了一半的包头
            if len(self.raw_buffer) > 3:
                 self.raw_buffer = self.raw_buffer[-3:]
            return None
            
        end_idx = self.raw_buffer.find(b'!', start_idx)
        if end_idx == -1:
            # 包尾还没收到，说明数据还没传完，等下次再解析
            return None

        # 3. 提取有效数据段并清空已经处理的缓冲
        payload_bytes = self.raw_buffer[start_idx + 3 : end_idx]
        self.raw_buffer = self.raw_buffer[end_idx + 1:]

        # 4. 解析数据
        try:
            data_str = payload_bytes.decode('utf-8')
            data_parts = data_str.split(',')
            if len(data_parts) >= 3:
                v = float(data_parts[0])
                yaw = float(data_parts[1])
                turn_angle = float(data_parts[2])
                return (v, yaw, turn_angle)
        except Exception as e:
            # 解析失败（如数据转换乱码等）
            pass

        return None

    # 解析主车发送的环绕角度
    def get_orbit_angle(self):
        # 1. 填充缓冲区
        if self.my_uart3.any():
            try:
                chunk = self.my_uart3.read()
                if chunk:
                    self.raw_buffer += chunk
            except:
                pass

        if not self.raw_buffer:
            return None

        # 内存保护
        if len(self.raw_buffer) > self.max_buf:
            self.raw_buffer = self.raw_buffer[-self.max_buf:]

        # 2. 寻找包头 '#O,' 和包尾 '!'
        start_idx = self.raw_buffer.find(b'#O,')
        if start_idx == -1:
            # 没找到需要的包头，清理掉无关的数据，防止内存越界
            if len(self.raw_buffer) > 3:
                 self.raw_buffer = self.raw_buffer[-3:]
            return None
            
        end_idx = self.raw_buffer.find(b'!', start_idx)
        if end_idx == -1:
            # 包尾还没收到，说明数据还没传完，等下次再解析
            return None

        # 3. 提取有效数据段并清空已经处理的缓冲
        payload_bytes = self.raw_buffer[start_idx + 3 : end_idx]
        self.raw_buffer = self.raw_buffer[end_idx + 1:]

        # 4. 解析数据
        try:
            angle = float(payload_bytes.decode('utf-8'))
            return angle
        except Exception as e:
            # 解析失败（如数据转换乱码等）
            pass

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
    def __init__(self, beep: beep, state, uart, car, path, plan, vision, moving, plan_data, order_manager: order_manager, art_protocal: UARTProtocol, slave_protocol: LinkProtocol):
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
        self.my_slave_protocol = slave_protocol

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
            # ... 其他状态
        }

        self.navigate_message = []  # 导航信息：目标点坐标和朝向
        self.pt_buffer = []  # 目标点坐标缓冲区
        self.current_object = ''  # 当前目标物体种类
        # 标志位
        self.if_transitioning = True  # 是否正在进行状态转换

        gc.collect()  # 进行垃圾回收，确保有足够内存用于状态机操作
        
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
            self.my_plan.reset_navigate_angle()
        elif state == NAVIGATE:
            # 进入导航状态，开始执行路径跟随
            pass
        elif state == SCAN:
            pass
        elif state == SERVO:
            # 进入伺服状态，开始精确对准目标物体
            pass
        elif state == MOVE:
            pass
        elif state == CALIBRATE:
            # 进入校准状态，进行位置或传感器校准
            # 记录小车在哪个边线
            self.my_vision.car_position = object_to_line_dict.get(self.current_object)
        elif state == ADJUST:
            # 进入调整状态，根据需要进行微调
            pass
        elif state == RETURN:
            # 进入返回状态，返回起始点或下一任务点
            pass
        elif state == STOP:
            # 进入停止状态，停止所有动作等待下一指令
            self.my_plan.reset_navigate_angle()

    def exit(self):
        state = self.my_state.state

        if state == READY_NAVIGATE:
            # 退出准备导航状态，清理路径规划相关资源
            if self.current_object == 'R':
                # 若当前物体信息为回程信息
                self.my_state.state = RETURN  # 直接切换到返回状态
            else:
                self.my_state.state = NAVIGATE  # 直接切换到导航状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == NAVIGATE:
            # 退出导航状态，停止路径跟随
            if self.current_object == 'P':
                self.my_plan.reset_navigate_angle()  # 重置导航角度
                self.my_plan.reset_navigate()  # 重置导航标志
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            else:
                if self.my_vision.if_send_order == False:
                    # 发送指令给openart，切换到目标识别模式
                    self.my_order_manager.mode_target()  
                    # 设置标志位，避免重复发送指令
                    self.my_vision.if_send_order = True  

                target_point = self.my_art_protocol.coordinate_receive()
                if target_point and target_point[2] == self.current_object:
                    self.my_plan.current_object = target_point[2]
                    self.my_vision.ready_servo_and_orbit(target_point)
                    self.my_vision.reset_servo_angle()
                    self.my_plan.reset_navigate()  # 重置导航相关变量
                    self.current_state = SERVO
                    self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == SCAN:
            pass
        elif state == SERVO:
            # 退出伺服状态，停止精确对准动作
            if self.my_vision.if_finish_servo:
                # 重置环绕角度
                self.my_vision.reset_orbit_angle()
                self.my_vision.if_finish_servo = False  # 重置伺服完成标志
                self.my_state.state = MOVE  # 直接切换到搬运状态
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
            elif self.my_plan.if_finish_navigate:
                self.my_vision.if_lost_object = False
                # 将openart置为等待模式
                self.my_order_manager.finish()
                self.my_plan.reset_navigate()
                self.my_slave_protocol.send_slave_state("lost")  # 通知主车丢失物体
                self.my_plan.reset_navigate_angle()
                self.my_state.state = READY_NAVIGATE
                self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == MOVE:
            # 退出搬运状态，停止搬运动作
            self.my_moving.if_finish_move = False  # 重置搬运完成标志
            self.my_state.state = CALIBRATE  # 直接切换到校准状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == CALIBRATE:
            # 退出校准状态，完成校准后进行必要的状态更新
            self.my_vision.reset_apriltag_calibrate()  # 重置校准标志
            self.my_plan.reset_navigate_angle()
            self.my_state.state = READY_NAVIGATE  # 直接切换到准备导航状态，准备处理下一个物体
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == ADJUST:
            # 退出调整状态，完成微调后进行必要的状态更新
            pass
        elif state == RETURN:
            # 退出返回状态，完成返回后进行必要的状态更新
            self.my_plan.reset_navigate()  # 重置导航标志
            self.my_state.state = STOP  # 直接切换到停止状态
            self.if_transitioning = True  # 退出当前状态，准备进入下一个状态
        elif state == STOP:
            # 退出停止状态，准备进入下一任务或待命状态
            self.my_beep.test()  # 任务完成，发出提示音
    
    def handle_ready_navigate(self):
        # 进入准备导航状态，做好路径规划准备和导航信息准备
        path = self.my_slave_protocol.get_path_list()  # 从从车协议中获取路径信息
        if path:
            # 只有当路径信息为过渡或者回城时才记录目标点坐标
            if path[0] in ['P', 'R']:
                self.pt_buffer = [path[2], path[1]]  # 储存目标坐标
            # 进行路径规划
            self.my_path.plan_path(path[2])
            self.navigate_message = [self.my_path.ready_path, path[1]]  # 目标坐标和转向角度
            self.current_object = path[0]  # 当前物体种类
            self.my_uart.write(f"Ready to navigate to {self.current_object} at {self.navigate_message[0]} with turn {self.navigate_message[1]}\r\n")  # 调试信息
            self.exit()  # 退出当前状态，进入导航状态

    def handle_navigate(self):
        # if state == NAVIGATE
        self.my_plan.navigate(path = self.navigate_message[0], target_turn_angle = self.navigate_message[1])

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入扫描状态
   
    def handle_scan(self):
        # if state == SCAN
        pass

    def handle_servo(self):
        # if state == SERVO
        if self.my_vision.if_lost_object == False:
            self.my_vision.visual_servo_control()
        else:
            # 若丢失物体则四处移动寻找物体
            x = self.my_car.x_current
            y = self.my_car.y_current
            self.my_plan.navigate(path = [[x+10.0, y], [x-10.0, y], self.pt_buffer[0]], target_turn_angle = self.pt_buffer[1])
            
            target_point = self.my_art_protocol.coordinate_receive()
            if target_point and chr(target_point[2]) == self.my_vision.current_servo_object:
                self.my_vision.ready_servo_and_orbit(target_point)
                self.my_plan.reset_navigate()
                self.my_vision.if_lost_object = False

        if self.my_vision.if_finish_servo or self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入搬运状态

    def handle_move(self):
        # if state == MOVE
        self.my_moving.moving()

        if self.my_moving.if_finish_move:
            self.exit()  # 退出当前状态，进入下一个状态

    def handle_calibrate(self):
        # if state == CALIBRATE
        global counter
        self.my_vision.apriltag_calibrate_control()

        if self.my_vision.if_finish_calibrate:
            counter += 1
            # 延时100ms
            if counter >= 10:
                counter = 0
                self.exit()  # 退出当前状态，进入下一个状态

    def handle_adjust(self):
        # if state == ADJUST
        pass

    def handle_return(self):
        # if state == RETURN
        self.my_plan.navigate(path = [self.pt_buffer[0]])  # 返回起始点

        if self.my_plan.if_finish_navigate:
            self.exit()  # 退出当前状态，进入停止状态

    def handle_stop(self):
        # if state == STOP
        self.my_plan.stop()