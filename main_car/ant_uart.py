from machine import *
from seekfree import WIRELESS_UART

# 异步串口通信初始化
my_uart6 = UART(5)
my_uart6.init(115200)
my_uart6.write("Motor test begins!\r\n")

# 无线串口初始化
wireless = WIRELESS_UART(115200)

def uart_receive():
    rx_data = my_uart6.readline()

    if rx_data:
        rx_str = rx_data.decode('utf-8', errors='ignore')
        return rx_str
    else:
        return None


