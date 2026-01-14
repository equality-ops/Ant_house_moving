from machine import *
from seekfree import WIRELESS_UART

# 异步串口通信初始化
my_uart6 = UART(5)
my_uart6.init(460800)
my_uart6.write("Motor test begins!\r\n")

# 无线串口初始化
wireless = WIRELESS_UART(115200)


