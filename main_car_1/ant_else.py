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

def uart_receive():
    """读取一个完整的数据帧（5字节）"""
    if my_uart6.any() >= 5:
        raw = my_uart6.read(5)  # 只读取5个字节
        # 测试
        #wireless.send_str(f"{raw}\n")
        try:
            header1, header2, x, y, tail = ustruct.unpack("<BBBBB", raw)
            if header1 == 0xA5 and header2 == 0xA6 and tail == 0x5B:
                return (x, y)
        except Exception:
            pass
    
    return None