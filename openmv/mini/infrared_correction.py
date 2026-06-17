import sensor
import image
import time
import math
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct

led = LED(4)  
# 点亮 LED4 以指示系统正在启动
led.on()
time.sleep_ms(500)
led.off()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.set_vflip(True)
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定
sensor.set_hmirror(True)
sensor.skip_frames(time=200)  # 跳过初始帧，让摄像头稳定

clock = time.clock()
uart = UART(2, baudrate=115200)

H_matrix = [[-9.65659491e-01, -6.04356653e-01, 1.02806428e+02],
            [-7.06519421e-02, 1.19662988e+00, -9.45029305e+01],
            [-5.28740196e-03, -5.25801317e-02, 1.00000000e+00]]
last_angle = 0.0
last_x, last_y = 0.0, 0.0
alpha = 0.7

def pixel_to_real_world(u, v):
        """
        将像素坐标转换为实际物理坐标
        :param u: 像素点的 x 坐标 (列)
        :param v: 像素点的 y 坐标 (行)
        :return: 真实的物理坐标 (X_w, Y_w)
        """

        # 消除红外灯高度带来的误差
        light_H = 5.1
        K = (9.5 - light_H) / 9.5

        # 计算缩放因子
        w_prime = H_matrix[2][0] * u + H_matrix[2][1] * v + H_matrix[2][2]
        # 计算真实的物理坐标
        X_w = (H_matrix[0][0] * u + H_matrix[0][1] * v + H_matrix[0][2]) / w_prime * K
        Y_w = (H_matrix[1][0] * u + H_matrix[1][1] * v + H_matrix[1][2]) / w_prime * K

        return X_w, Y_w

def send_coordinate(x1, y1, x2, y2):
        """发送目标坐标（带防抖和范围限制）"""
        # is_first_send = (self.last_sent_x == SCREEN_CENTER_X) and (self.last_sent_y == SCREEN_CENTER_Y)

        # 保证 x1,y1 为左侧点，x2,y2 为右侧点
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # 坐标换算（保留一位小数，乘以 10 转为整数发送）
        x1 = int(round(x1, 1) * 10)
        y1 = int(round(y1, 1) * 10)
        x2 = int(round(x2, 1) * 10)
        y2 = int(round(y2, 1) * 10)

        # 打包并发送数据
        data = ustruct.pack(
            "<BhhhhB",
            0xA1,
            x1,
            y1,
            x2,
            y2,
            0xA2
        )
        uart.write(data)

while(True):
    clock.tick()
    img = sensor.snapshot()
    img.binary([(230, 255)])
    real_center = []
    for blob in img.find_blobs([(255, 255)], pixels_threshold=6, area_threshold=6, merge=True):
        #print(blob.cx(), blob.cy())
        #img.draw_rectangle(blob.rect())
        real_cx, real_cy = pixel_to_real_world(blob.cx(), blob.cy())
        real_center.append((real_cx, real_cy))
        # img.draw_cross(int(blob.cx()), int(blob.cy()), color=127, size=5)

    if len(real_center) == 2:
        # 按横坐标排序：x小的为(x1,y1)，x大的为(x2,y2)
        if real_center[0][0] <= real_center[1][0]:
            x1, y1 = real_center[0][0], real_center[0][1]
            x2, y2 = real_center[1][0], real_center[1][1]
        else:
            x1, y1 = real_center[1][0], real_center[1][1]
            x2, y2 = real_center[0][0], real_center[0][1]

        # 计算当前现实世界原始的中点和角度
        # curr_x = (x1 + x2) / 2
        # curr_y = (y1 + y2) / 2
        # curr_angle = math.degrees(math.atan2(y2 - y1, x2 - x1))

        # 一阶滞后滤波，防止数据跳变
        # target_coordinate_x = alpha * curr_x + (1 - alpha) * last_x
        # target_coordinate_y = alpha * curr_y + (1 - alpha) * last_y
        # target_angle = alpha * curr_angle + (1 - alpha) * last_angle

        # 更新历史数据
        # last_x, last_y = target_coordinate_x, target_coordinate_y
        # last_angle = target_angle

        send_coordinate(x1, y1, x2, y2)
        # print(f"1:{(x1, y1)}, 2:{(x2, y2)}")
    elif len(real_center) == 1:
        send_coordinate(real_center[0][0], real_center[0][1], real_center[0][0], real_center[0][1])
    else:
        pass
    #print(clock.fps())