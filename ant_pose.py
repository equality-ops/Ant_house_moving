from machine import *
from seekfree import IMU660RX
from smartcar import ticker, encoder
import ant_motor
from ant_math import MATH as MATH
from ant_flash import find_aimed_value as find_value

# 编码器初始化
encoder_ul = encoder("C0" , "C1" , True)
encoder_ur = encoder("C2" , "C3" , True)
encoder_md = encoder("D15", "D16", True)

# IMU初始化
imu = IMU660RX()

class PoseData:
    def __init__(self, diff_filter: ant_motor.SlipAveragingFilter):
        self.encoder_data_ul = 0    # type: int
        self.encoder_data_ur = 0    # type: int
        self.encoder_data_md = 0    # type: int
        self.acc_x = 0              # type: int
        self.acc_y = 0              # type: int
        self.acc_z = 0              # type: int
        self.gyro_x = 0             # type: int
        self.gyro_y = 0             # type: int
        self.gyro_z = 0             # type: float
        self.gyro_z_bias = find_value(ant_motor.config, "gyro_z_bias")        # type: int
        self.diff_filter = diff_filter

    def update_data(self):
        self.encoder_data_ul = encoder_ul.get()
        self.encoder_data_ur = encoder_ur.get()
        self.encoder_data_md = encoder_md.get()

        self.acc_x = imu[0]
        self.acc_y = imu[1]
        self.acc_z = imu[2]
        self.gyro_x = imu[3]
        self.gyro_y = imu[4]
        # 去零漂后滑动平均滤波（单位：角度每秒）
        self.gyro_z = self.diff_filter.filtering((imu[5] - self.gyro_z_bias) / 16.4) 

    