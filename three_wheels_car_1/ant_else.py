from machine import *
import time
from seekfree import WIRELESS_UART
import ustruct

# 蜂鸣器初始化
beep = Pin('D24', Pin.OUT, value = False)

# 异步串口通信初始化
my_uart6 = UART(5)
my_uart6.init(115200)
my_uart6.write("Motor test begins!\r\n")
my_uart6.write("hello\r\n")
my_uart6.write("hello\r\n")
# 无线串口初始化
wireless = WIRELESS_UART(115200)


class BeepState:
    beep_state = 0
    BEEP_OFF = 0
    BEEP_ON = 1

# 蜂鸣器警告函数(响3声，每500ms响一声，每次持续50ms)
def beep_warn() -> None:
    if BeepState.beep_state == BeepState.BEEP_OFF:
        BeepState.beep_state = BeepState.BEEP_ON
        for i in range(3):
            time.sleep_ms(50)
            beep.high()
            time.sleep_ms(50)
            beep.low()
            time.sleep_ms(400)
        BeepState.beep_state = BeepState.BEEP_OFF
        return 
    elif BeepState.beep_state == BeepState.BEEP_ON:
        return 

def key_test() -> None:
    if BeepState.beep_state == BeepState.BEEP_OFF:
        BeepState.beep_state = BeepState.BEEP_ON
        beep.high()
        time.sleep_ms(100)
        beep.low()
        BeepState.beep_state = BeepState.BEEP_OFF
        return
    elif BeepState.beep_state == BeepState.BEEP_ON:
        return

# 检测是否完成视觉伺服的蜂鸣器提示函数  
def finish_servo() -> None:
    if BeepState.beep_state == BeepState.BEEP_OFF:
        BeepState.beep_state = BeepState.BEEP_ON
        beep.high()
        time.sleep_ms(100)
        beep.low()
        time.sleep_ms(300)
        beep.high()
        time.sleep_ms(100)
        beep.low()
        BeepState.beep_state = BeepState.BEEP_OFF
        return
    elif BeepState.beep_state == BeepState.BEEP_ON:
        return

# 状态机解析串口数据类
class UARTProtocol:
    def __init__(self):
        self.state = 0  # 0:等待帧头1, 1:等待帧头2, 2:等待x, 3:等待y, 4:等待帧尾
        self.coordinate_buffer = [0, 0, 0, 0, 0]
        self.byte_count = 0

    # 非阻塞接收并解析物体中心的像素点坐标  
    def coordinate_receive(self):
        while my_uart6.any():
            byte = my_uart6.read(1)[0]
            
            if self.state == 0:  # 等待帧头1
                if byte == 0xA5:
                    self.coordinate_buffer[0] = byte
                    self.state = 1
                # 如果不是0xA5，继续等待（保持状态0）
                
            elif self.state == 1:  # 等待帧头2
                if byte == 0xA6:
                    self.coordinate_buffer[1] = byte
                    self.state = 2
                else:
                    self.state = 0  # 状态重置
                    
            elif self.state == 2:  # 接收x
                self.coordinate_buffer[2] = byte
                self.state = 3
                
            elif self.state == 3:  # 接收y
                self.coordinate_buffer[3] = byte
                self.state = 4
                
            elif self.state == 4:  # 等待帧尾
                if byte == 0x5B:
                    self.coordinate_buffer[4] = byte
                    # 完整帧接收完成
                    x, y = self.coordinate_buffer[2], self.coordinate_buffer[3]
                    self.state = 0  # 重置状态
                    return (x, y)
                else:
                    self.state = 0  # 帧尾错误，重新同步
        
        return None  # 没有完整帧

# 使用
protocol = UARTProtocol()