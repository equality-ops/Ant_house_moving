import sensor, image, time, math, mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct
from typing import Optional
###########################通信模块########################
class Communicator:
    def __init__(self, uart):
        self.uart = uart
        self.last_sent_x = 80
        self.last_sent_y = 60

    def send_coordinate(self, x, y, obj_type: Optional[str] = ''):
        x = int(round(x))
        y = int(round(y))

        if abs(x - self.last_sent_x) < 3 and abs(y - self.last_sent_y) < 3 and y <= 40:
            return #增加最小变化阈值（防抖）

        dx_coord = min(30, max(-30, x - self.last_sent_x))
        dy_coord = min(30, max(-30, y - self.last_sent_y))
        x_limited = self.last_sent_x + dx_coord
        y_limited = self.last_sent_y + dy_coord

        if x_limited < 0:
            x_limited = 0
        elif x_limited > 160:
            x_limited = 160

        if y_limited < 0:
            y_limited = 0
        elif y_limited > 120:
            y_limited = 120

        self.last_sent_x = x_limited
        self.last_sent_y = y_limited

        type_char = 0x00  # 初始值
        if obj_type == 'red' or obj_type == 'blue':
            type_char = ord('S')  # S的ASCII码
        elif obj_type == 'green':
            type_char = ord('T')  # T的ASCII码
        elif obj_type == 'brown':
            type_char = ord('B')  # B的ASCII码
        elif obj_type == 'white':
            type_char = ord('W')  # W的ASCII码

        data = ustruct.pack("<BBBBBB", 0xA5, 0xA6, x_limited, y_limited, type_char, 0x5B)
        self.uart.write(data)

    def send_angle(self, angle):
        if angle is None:
            return
        angle_mapped = angle + 90  # 映射到 0～180
        data = ustruct.pack("<BBBB", 0xA5, 0xA7, angle_mapped, 0x5B)
        self.uart.write(data)

#######################颜色检测模块########################
class ColorDetector:
    # 颜色阈值（类变量，共享）
    RED_THRESHOLD = [(5, 24, 12, 41, -5, 37), (30, 58, 39, 83, 10, 51)]
    GREEN_THRESHOLD = [(17, 67, -33, -15, -15, 68), (53, 100, -51, -15, -20, 95)]
    BLUE_THRESHOLD = [(13, 35, -24, -9, -18, -7), (37, 77, -31, -4, -54, -26)]
    BROWN_THRESHOLD = [(12, 43, -14, 14, 8, 46), (51, 92, -23, 20, -16, 70)]
    WHITE_THRESHOLD = []

    # 定义中心采样区 (x, y, w, h)
    # 针对 160x120 图像，取中心 40x30 区域
    CENTER_ROI = (60, 45, 40, 30)

    # 距离阈值
    DISTANCE_THRESHOLD = 30

    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def detect_colors(self, img):
        """
        adjusted_brown = [self.auto_adjust_threshold(img, th) for th in self.BROWN_THRESHOLD]
        adjusted_red = [self.auto_adjust_threshold(img, th) for th in self.RED_THRESHOLD]
        adjusted_green = [self.auto_adjust_threshold(img, th) for th in self.GREEN_THRESHOLD]
        adjusted_blue = [self.auto_adjust_threshold(img, th) for th in self.BLUE_THRESHOLD]

        """
        brown_blobs = img.find_blobs(self.BROWN_THRESHOLD, pixels_threshold=400, area_threshold=400, merge=True)
        white_blobs = img.find_blobs(self.WHITE_THRESHOLD, pixels_threshold=400, area_threshold=400, merge=True)
        red_blobs   = img.find_blobs(self.RED_THRESHOLD,   pixels_threshold=30,  area_threshold=30,  merge=False)
        green_blobs = img.find_blobs(self.GREEN_THRESHOLD, pixels_threshold=30,  area_threshold=30,  merge=False)
        blue_blobs  = img.find_blobs(self.BLUE_THRESHOLD,  pixels_threshold=30,  area_threshold=30,  merge=False)
        """
        brown_blobs = img.find_blobs(adjusted_brown, pixels_threshold=200, area_threshold=200, merge=True)
        red_blobs   = img.find_blobs(adjusted_red,   pixels_threshold=30,  area_threshold=30,  merge=False)
        green_blobs = img.find_blobs(adjusted_green, pixels_threshold=30,  area_threshold=30,  merge=False)
        blue_blobs  = img.find_blobs(adjusted_blue,  pixels_threshold=30,  area_threshold=30,  merge=False)
        """
        all_blobs = []
        for blob in brown_blobs: all_blobs.append((blob, 'brown'))
        for blob in white_blobs: all_blobs.append((blob, 'white'))
        for blob in red_blobs:   all_blobs.append((blob, 'red'))
        for blob in green_blobs: all_blobs.append((blob, 'green'))
        for blob in blue_blobs:  all_blobs.append((blob, 'blue'))
        return all_blobs

    def filter_all_blobs(self, blobs):
        filtered = []
        for blob, color in blobs:
            if blob.density() < 0.3:
                continue
            min_pixels = 50 * (blob.density() + 0.5)
            if blob.pixels() < min_pixels:
                continue
            """
            if (blob.w() > 140 or blob.h() > 110):
                continue
            """
            if color == 'brown' and (blob.w() > 3 * blob.h() or blob.h() > 3 * blob.w()):
                continue
            if color == 'white' and (blob.w() > 3 * blob.h() or blob.h() > 3 * blob.w()):
                continue
            if color in ('green', 'blue') and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w() or abs(blob.w() - blob.h()) > 10):
                continue

            cx, cy = blob.cx(), blob.cy()
            keep = True
            for saved_blob, _ in filtered:
                d = self.calculate_distance(cx, cy, saved_blob.cx(), saved_blob.cy())
                if d < self.DISTANCE_THRESHOLD:
                    keep = False
                    break
            if keep:
                filtered.append((blob, color))
        return filtered

    def auto_adjust_threshold(self, img, base_threshold):
        stats = img.get_statistics(roi = self.CENTER_ROI)
        l_mean = stats.l_mean()

        target_brightness = 50  # 目标亮度
        brightness_diff = l_mean - target_brightness
        adjust_factor = 0.3  # 0.1-0.5之间调整，越小越平滑
        diff = brightness_diff * adjust_factor
        dead_zone = 2  # 亮度在48-52之间时，不调整阈值
        if abs(brightness_diff) < dead_zone:
            diff = 0

        l_low = base_threshold[0] + diff
        l_high = base_threshold[1] + diff
        l_low = max(0, min(100, l_low))
        l_high = max(0, min(100, l_high))
        # 保证l_low < l_high（避免阈值交叉导致曝光异常）
        if l_low >= l_high:
            l_low = max(0, l_high - 5)  # 至少保留5的阈值差

        threshold_part = base_threshold[2:]
        return (round(l_low), round(l_high)) + threshold_part  # 取整适配OpenART

########################边界检测模块######################
class BoundaryDetector:
    YELLOW_THRESHOLD = (70, 100, -128, 127, 10, 127)
    ROI_MID = (40, 0, 80, 120)
    SCREEN_WIDTH = 160
    MIDDLE_X = SCREEN_WIDTH // 2  # 80
    X_TOLERANCE = 5

    # 边界识别
    def boundary_correction(self, mode, img):
        angle = None
        blobs = []
        center = 0

        if mode == 'row': #行
            num = [0, 26, 52, 80, 106, 132]
        elif mode == 'column': #列
            num = [0, 20, 40, 60, 80, 100]
        for x in num:
            if mode == 'row': # 从左到右找色块
                result = img.find_blobs([self.YELLOW_THRESHOLD], roi = [x,0,26,120] ,pixels_threshold=100, area_threshold=100, margin=1, merge=True, invert=0)
            elif mode == 'column':# 从上到下找色块
                result = img.find_blobs([self.YELLOW_THRESHOLD], roi = [0,x,160,20] ,pixels_threshold=100, area_threshold=100, margin=1, merge=True, invert=0)
            if result:
                best_blob = min(result, key= lambda b: abs(b.area() - 600)) # 面积还要调整
                blobs.append(best_blob)
                center += 1
                img.draw_rectangle(best_blob.rect(), color = (255, 0, 0), scale = 1, thickness = 1)

        if center >= 3:
            l = img.get_regression([self.YELLOW_THRESHOLD], roi = self.ROI_MID, robust = True)
            if l:
                img.draw_line(l.line(), color = (255, 0, 0), thickness = 2)
                x1, y1, x2, y2 = l.line()
                if y1 > y2:
                    bottom_x = x1
                else:
                    bottom_x = x2

                if abs(bottom_x - self.MIDDLE_X) <= self.X_TOLERANCE:
                    theta = l.theta()
                    if theta > 90:
                        angle = theta - 180
                    else:
                        angle = theta
                return angle
            else:
                return None
        else:
            return None

######################棕色目标跟踪模块######################
class KalmanTracker:
    MAX_LOST_FRAMES = 20

    def __init__(self):
# 1. 状态量定义: [x, y, vx, vy]
        self.x_hat = np.array([80, 60, 0, 0], dtype=np.float)

        # 2. 预分配矩阵 (内存优化：避免循环内创建对象)
        self.A = np.eye(4)      # 状态转移矩阵
        self.P = np.eye(4) * 10 # 后验协方差
        self.Q = np.eye(4) * 0.5# 过程噪声
        self.R = np.eye(2) * 5  # 测量噪声 (x, y)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float) # 观测矩阵

        self.first_detected = False
        self.lost_count = 0
        self.last_w = 0
        self.last_h = 0

    def reset(self):
        self.first_detected = False
        self.lost_count = 0
        self.x_hat = np.array([80, 60, 0, 0], dtype=np.float)
        self.P = np.eye(4) * 10
        self.last_w = 0
        self.last_h = 0
    """
    def is_valid(self, blob):
        current_area = blob.area()
        if self.last_brown_area > 0:
            rate = abs(current_area - self.last_brown_area) / self.last_brown_area
            if rate > 0.7:
                return False
        self.last_brown_area = current_area
        self.brown_visible_frames += 1
        return True
    """
    def kalman_filter(self, pos, Ts):
            # 更新状态转移矩阵中的时间步长 Ts
            self.A[0, 2] = Ts
            self.A[1, 3] = Ts

            # 丢包阻尼处理
            damping = 0.94 if pos else 0.82
            self.A[2, 2] = damping
            self.A[3, 3] = damping

            # --- 预测阶段 (Predict) ---
            x_pre = np.dot(self.A, self.x_hat)
            # P_pre = A*P*A.T + Q
            P_pre = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

            # --- 更新阶段 (Update) ---
            if pos:
                self.lost_count = 0
                self.first_detected = True
                z = np.array(pos, dtype=np.float) # [cx, cy]

                # 计算增益 K = P_pre*H.T / (H*P_pre*H.T + R)
                S = np.dot(self.H, np.dot(P_pre, self.H.T)) + self.R
                try:
                    K = np.dot(P_pre, np.dot(self.H.T, np.linalg.inv(S)))
                except np.linalg.LinAlgError:
                    self.x_hat = x_pre
                    self.P = P_pre
                    return self.x_hat

                # 更新状态 x_hat = x_pre + K*(z - H*x_pre)
                self.x_hat = x_pre + np.dot(K, (z - np.dot(self.H, x_pre)))
                # 更新协方差 P = (I - K*H)*P_pre
                self.P = np.dot((np.eye(4) - np.dot(K, self.H)), P_pre)
            else:
                self.lost_count += 1
                self.x_hat = x_pre # 丢失时仅依靠预测
                self.P = P_pre

            return self.x_hat

########################初始化#####################

# 串口
uart = UART(2, baudrate=460800)
uart.write("uart test\r\n")

# 摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_framerate(60)
sensor.set_auto_gain(False) # 自动增益
sensor.set_auto_whitebal(False)
sensor.set_brightness(1000)
# sensor.set_contrast(2) # 对比度
sensor.skip_frames(time = 30)
clock = time.clock()
# LED(4).on

# LCD
lcd = seekfree.IPS200(3)
lcd.full()

# 创建模块实例
color_detector = ColorDetector()
boundary_detector = BoundaryDetector()
brown_tracker = KalmanTracker()
white_tracker = KalmanTracker()
communicator = Communicator(uart)

# 模式定义
MODE_TARGET = 0
MODE_BOUNDARY_UD = 1
MODE_BOUNDARY_LR = 2
MODE_WAITING = 3
current_mode = MODE_WAITING

# 时间戳
last_time = time.ticks_ms()

# ====================== 锁定逻辑变量 ======================
LOCK_JUMP_THRESHOLD = 20  # 坐标跳变超过20像素视为同色干扰
LOCK_MAX_LOST_FRAMES = 5  # 丢失5帧解除锁定
# 锁定状态
is_target_locked = False  # 是否锁定目标
locked_target_color = ''  # 锁定的目标颜色
locked_target_cx = 80  # 锁定目标的初始x坐标
locked_target_cy = 60  # 锁定目标的初始y坐标
locked_last_cx = 80  # 上一帧锁定目标的x坐标
locked_last_cy = 60  # 上一帧锁定目标的y坐标
locked_lost_count = 0  # 锁定目标丢失帧数

def reset_lock_state():
    """重置锁定状态"""
    global is_target_locked, locked_target_color, locked_target_cx, locked_target_cy
    global locked_last_cx, locked_last_cy, locked_lost_count
    is_target_locked = False
    locked_target_color = ''
    locked_target_cx = 80
    locked_target_cy = 60
    locked_last_cx = 80
    locked_last_cy = 60
    locked_lost_count = 0

def is_coordinate_jump_too_large(cx, cy):
    """判断坐标跳变是否过大（同色干扰）"""
    global locked_last_cx, locked_last_cy
    distance = math.sqrt((cx - locked_last_cx)**2 + (cy - locked_last_cy)**2)
    return distance > LOCK_JUMP_THRESHOLD

######################命令处理###################
def handle_uart_commands():
    global current_mode
    if uart.any():
        cmd = uart.read(1)
        if cmd == b'T': current_mode = MODE_TARGET
        elif cmd == b'U': current_mode = MODE_BOUNDARY_UD
        elif cmd == b'L': current_mode = MODE_BOUNDARY_LR
        elif cmd == b'F': current_mode = MODE_WAITING

#######################主循环####################
while True:
    clock.tick()
    img = sensor.snapshot()

    # 时间戳更新
    current_time = time.ticks_ms()
    delta_time = time.ticks_diff(current_time, last_time)
    Ts = max(delta_time / 1000.0, 0.01)
    last_time = current_time

    handle_uart_commands()

    if current_mode == MODE_WAITING:
        """
        LED(1).on()
        LED(1).off()
        """
        continue

    elif current_mode == MODE_TARGET:
        """
        LED(2).on()
        LED(2).off()
        """
        # LED(4).off()
        # LED(4).on()
        # 色块检测与筛选
        all_blobs_with_color = color_detector.detect_colors(img)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        # 绘制颜色映射
        draw_colors = {
            'red': (255, 0, 0), # 红色沙包
            'green': (0, 255, 0), # 网球
            'blue': (0, 0, 255), # 蓝色沙包
            'white': (255, 255, 255), # 白色玩具熊
            'grey': (100, 100, 100), # 卡尔曼框
            'black': (0, 0, 0), # 锁定标识
            'brown': (150, 75, 0) # 棕色玩具熊
        }

        center = []  # 所有有效色块的中心坐标
        target_pos = None  # 最终要发送的目标坐标
        locked_blob = None  # 锁定的目标色块
        target_color = ''

        # 分离棕色白色与其它色块
        brown_blobs = []
        white_blobs = []
        other_blobs = []
        for item in filtered_blobs_with_color:
            blob = item[0]
            color = item[1]
            if color == 'brown':
                brown_blobs.append(blob)
            elif color == 'white':
                white_blobs.append(blob)
            else:
                other_blobs.append((blob, color))

        # 处理棕色色块
        if brown_blobs:
            target_brown = max(brown_blobs, key=lambda b: b.area())
            brown_pos = (target_brown.cx(), target_brown.cy())
            brown_tracker.last_w = target_brown.w()
            brown_tracker.last_h = target_brown.h()
            img.draw_rectangle(target_brown.rect(), color=draw_colors['brown'])
            img.draw_cross(target_brown.cx(), target_brown.cy(), color=draw_colors['brown'])
            # 棕色目标卡尔曼滤波
            brown_state = brown_tracker.kalman_filter(brown_pos, Ts)
            if brown_tracker.first_detected and brown_tracker.lost_count < brown_tracker.MAX_LOST_FRAMES:
                kcx, kcy = int(brown_state[0]), int(brown_state[1])
                center.append((kcx, kcy, 'brown'))
                img.draw_rectangle(kcx - brown_tracker.last_w//2, kcy - brown_tracker.last_h//2,
                                   brown_tracker.last_w, brown_tracker.last_h, color=draw_colors['grey'])
                img.draw_cross(kcx, kcy, color=draw_colors['grey'])
        else:
            brown_tracker.kalman_filter(None, Ts)
            if brown_tracker.lost_count >= brown_tracker.MAX_LOST_FRAMES:
                brown_tracker.reset()

        # 处理白色色块
        if white_blobs:
            target_white = max(white_blobs, key=lambda b: b.area())
            white_pos = (target_white.cx(), target_white.cy())
            white_tracker.last_w = target_white.w()
            white_tracker.last_h = target_white.h()
            img.draw_rectangle(target_white.rect(), color=draw_colors['white'])
            img.draw_cross(target_white.cx(), target_white.cy(), color=draw_colors['white'])
            # 白色目标卡尔曼滤波
            white_state = white_tracker.kalman_filter(white_pos, Ts)
            if white_tracker.first_detected and white_tracker.lost_count < white_tracker.MAX_LOST_FRAMES:
                kcx, kcy = int(white_state[0]), int(white_state[1])
                center.append((kcx, kcy, 'white'))
                img.draw_rectangle(kcx - white_tracker.last_w//2, kcy - white_tracker.last_h//2,
                                   white_tracker.last_w, white_tracker.last_h, color=draw_colors['grey'])
                img.draw_cross(kcx, kcy, color=draw_colors['grey'])
        else:
            white_tracker.kalman_filter(None, Ts)
            if white_tracker.lost_count >= white_tracker.MAX_LOST_FRAMES:
                white_tracker.reset()

        # 处理其他颜色色块
        for item in other_blobs:
            blob = item[0]
            color_name = item[1]
            img.draw_rectangle(blob.rect(), color=draw_colors[color_name])
            img.draw_cross(blob.cx(), blob.cy(), color=draw_colors[color_name])
            center.append((blob.cx(), blob.cy(), color_name))

        # 锁定处理
        if filtered_blobs_with_color:
            # 未锁定：选择y最大的目标作为锁定对象
            if not is_target_locked:
                max_y_blob, max_y_color = max(filtered_blobs_with_color, key=lambda item: item[0].cy())
                locked_target_cx = max_y_blob.cx()
                locked_target_cy = max_y_blob.cy()
                locked_last_cx = locked_target_cx
                locked_last_cy = locked_target_cy
                locked_target_color = max_y_color
                is_target_locked = True
                locked_lost_count = 0
                target_pos = (locked_target_cx, locked_target_cy)
                target_color = max_y_color
                locked_blob = max_y_blob
            # 已锁定：仅筛选同色目标，且坐标跳变不超过阈值
            else:
                # 筛选同色目标
                same_color_blobs = [item for item in filtered_blobs_with_color if item[1] == locked_target_color]
                valid_blobs = []
                for blob, color in same_color_blobs:
                    cx, cy = blob.cx(), blob.cy()
                    # 跳变不超过阈值才视为有效目标
                    if not is_coordinate_jump_too_large(cx, cy):
                        valid_blobs.append((blob, cx, cy))
                if valid_blobs:
                    # 选同色目标中最接近初始锁定位置的
                    best_blob, best_cx, best_cy = min(valid_blobs,
                        key=lambda item: math.sqrt((item[1]-locked_last_cx)**2 + (item[2]-locked_last_cy)**2))
                    target_pos = (best_cx, best_cy)
                    locked_last_cx = best_cx
                    locked_last_cy = best_cy
                    locked_lost_count = 0
                    target_color = locked_target_color
                    locked_blob = best_blob
                else:
                    # 无有效同色目标，计数+1
                    locked_lost_count += 1
                    target_pos = None
        else:
            # 无任何色块，锁定计数+1
            if is_target_locked:
                locked_lost_count += 1
            target_pos = None

        # 超过5帧解除锁定
        if is_target_locked and locked_lost_count >= LOCK_MAX_LOST_FRAMES:
            reset_lock_state()
            target_color = ''

        if is_target_locked and locked_blob is not None:
            lock_cx = locked_blob.cx()
            lock_cy = locked_blob.cy()
            # 在锁定目标中心绘制黑色圆
            img.draw_circle(lock_cx, lock_cy, 5, color=draw_colors['black'], thickness=2)

        # 锁定状态下仅发送锁定目标坐标，未锁定时按原有逻辑选y最大
        if is_target_locked and target_pos is not None:
            # 锁定状态：发送锁定目标坐标
            communicator.send_coordinate(target_pos[0], target_pos[1], target_color)
        elif center:
            # 未锁定：按原有逻辑选y最大的坐标
            target = max(center, key=lambda coordinate: coordinate[1])
            target_x = target[0]
            target_y = target[1]
            target_color = target[2]
            communicator.send_coordinate(target_x, target_y, target_color)

    elif current_mode == MODE_BOUNDARY_UD:
        """
        LED(3).on()
        LED(3).off()
        """
        angle = boundary_detector.boundary_correction('row', img)
        if angle is not None:
            communicator.send_angle(angle)

    elif current_mode == MODE_BOUNDARY_LR:
        """
        LED(4).on()
        LED(4).off()
        """
        angle = boundary_detector.boundary_correction('column', img)
        if angle is not None:
            communicator.send_angle(angle)

    lcd.show_image(img, 160, 120, zoom=0)
    # print(clock.fps())
