from machine import *
from seekfree import WIRELESS_UART
import time
import ustruct

# 异步串口通信初始化
my_uart6 = UART(5)
my_uart6.init(115200)
my_uart6.write("Motor test begins!\r\n")

# 无线串口初始化
wireless = WIRELESS_UART(115200)

def uart_receive(timeout_ms = 100):
    start = time.ticks_ms()
    while my_uart6.any() < 5:
        if time.ticks_diff(time.ticks_ms(), start) >= timeout_ms:
            return None
        time.sleep_ms(1)
    
    raw = my_uart6.read(5)
    if len(raw) != 5:
        return None
    
    try:
        header1, header2, x, y, tail = ustruct.unpack("<BBBBB", raw)
        if header1 == 0xA5 and header2 == 0xA6 and tail == 0x5B:
            return (x, y)
        else:
            return None
    except Exception:
        return None


