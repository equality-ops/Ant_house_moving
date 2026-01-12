from machine import *
from seekfree import IMU660RX
from smartcar import ticker, encoder
import ant_motor
from ant_math import MATH as MATH
from ant_flash import find_aimed_value as find_value

# 编码器初始化
encoder_ul = encoder("C2" , "C3" , True)
encoder_ur = encoder("C0" , "C1" , True)
encoder_md = encoder("D15", "D16", False)

# IMU初始化
imu = IMU660RX()
# 定时器1采集已经与imu_data相连
imu_data = []   # type: list

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
        self.acc_x_bias = 0.0        # type: float
        self.acc_y_bias = 0.0        # type: float
        self.acc_z_bias = 0.0        # type: float
        self.gyro_x_bias = 0.0       # type: float
        self.gyro_y_bias = 0.0       # type: float
        self.gyro_z_bias = 0.0       # type: float
        self.diff_filter = diff_filter

    # 初始零偏计算函数
    def init_bias(self):
        acc_x_sum = 0
        acc_y_sum = 0
        acc_z_sum = 0
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0

        sample_count = 200
        for i in range(sample_count):
            imu_data = imu.read()
            acc_x_sum += imu_data[0]
            acc_y_sum += imu_data[1]
            acc_z_sum += imu_data[2]
            gyro_x_sum += imu_data[3]
            gyro_y_sum += imu_data[4]
            gyro_z_sum += imu_data[5]

        self.acc_x_bias = acc_x_sum / sample_count
        self.acc_y_bias = acc_y_sum / sample_count
        self.acc_z_bias = acc_z_sum / sample_count
        self.gyro_x_bias = gyro_x_sum / sample_count
        self.gyro_y_bias = gyro_y_sum / sample_count
        self.gyro_z_bias = gyro_z_sum / sample_count

    # 传感器数据更新函数
    def update_data(self):
        self.encoder_data_ul = encoder_ul.get()
        self.encoder_data_ur = encoder_ur.get()
        self.encoder_data_md = encoder_md.get()

        self.acc_x = imu_data[0] - self.acc_x_bias
        self.acc_y = imu_data[1] - self.acc_y_bias
        self.acc_z = imu_data[2] - self.acc_z_bias
        self.gyro_x = imu_data[3] - self.gyro_x_bias
        self.gyro_y = imu_data[4] - self.gyro_y_bias
        # 去零漂后滑动平均滤波（单位：角度每秒）
        self.gyro_z = self.diff_filter.filtering(imu_data[5] - self.gyro_z_bias) / 16.4

    

