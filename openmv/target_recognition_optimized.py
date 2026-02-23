import sensor, image, time, math, mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np
import seekfree
import ustruct
###########################通信模块########################
class Communicator:
    def __init__(self, uart):
        self.uart = uart
        self.last_sent_x = 80
        self.last_sent_y = 60

    def send_coordinate(self, x, y):
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
        data = ustruct.pack("<BBBBB", 0xA5, 0xA6, x_limited, y_limited, 0x5B)
        self.uart.write(data)

    def send_angle(self, angle):
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

    # 距离阈值
    DISTANCE_THRESHOLD = 30

    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def detect_colors(self, img):
        brown_blobs = img.find_blobs(self.BROWN_THRESHOLD, pixels_threshold=200, area_threshold=200, merge=True)
        red_blobs   = img.find_blobs(self.RED_THRESHOLD,   pixels_threshold=30,  area_threshold=30,  merge=False)
        green_blobs = img.find_blobs(self.GREEN_THRESHOLD, pixels_threshold=30,  area_threshold=30,  merge=False)
        blue_blobs  = img.find_blobs(self.BLUE_THRESHOLD,  pixels_threshold=30,  area_threshold=30,  merge=False)

        all_blobs = []
        for blob in brown_blobs: all_blobs.append((blob, 'brown'))
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

########################边界检测模块######################
class BoundaryDetector:
    YELLOW_THRESHOLD = (70, 100, -128, 127, 10, 127)
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
                best_blob = min(result, key= lambda b: abs(b.area() - 600))
                blobs.append(best_blob)
                center += 1
                img.draw_rectangle(best_blob.rect(), color = (255, 0, 0), scale = 1, thickness = 1)

        if center >= 3:
            l = img.get_regression([self.YELLOW_THRESHOLD], robust = True)
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
class BrownTracker:
    MAX_LOST_FRAMES = 20

    def __init__(self):
        self.C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],
                           [0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.R = np.diag([5, 5, 2, 2, 50, 50])

        # 状态
        self.reset()

    def reset(self):
        self.first_detected = False
        self.lost_count = 0
        # self.brown_visible_frames = 0
        self.last_brown_area = 0
        self.last_cx, self.last_cy = 80, 60
        self.x_hat = np.array([80, 60, 30, 30, 2, 2], dtype=np.float)
        self.p = np.diag([100.0, 100.0, 50.0, 50.0, 300.0, 300.0])
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
    def kalman_filter(self, Z, Ts, is_detected):
        if is_detected:
            Q_value = [2.0, 5.0, 2.0, 2.0, 50.0, 50.0]
            damping = 0.92
            self.lost_count = 0
        else:
            Q_value = [20.0, 30.0, 10.0, 10.0, 40.0, 40.0]
            damping = 0.8
            self.lost_count += 1
        Q = np.diag(Q_value)

        A = np.array([
            [1, 0, 0, 0, Ts, 0],
            [0, 1, 0, 0, 0, Ts],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, damping, 0],
            [0, 0, 0, 0, 0, damping]
        ])

        x_hat_minus = np.dot(A, self.x_hat)
        p_minus = np.dot(A, np.dot(self.p, A.T)) + Q

        if is_detected:
            S = np.dot(np.dot(self.C, p_minus), self.C.T) + self.R
            S_reg = S + 1e-4 * np.eye(S.shape[0])
            S_inv = np.linalg.inv(S_reg)
            K = np.dot(np.dot(p_minus, self.C.T), S_inv)
            self.x_hat = x_hat_minus + np.dot(K, (Z - np.dot(self.C, x_hat_minus)))
            self.p = np.dot((np.eye(6) - np.dot(K, self.C)), p_minus)
        else:
            self.x_hat = x_hat_minus
            self.p = p_minus

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
brown_tracker = BrownTracker()
communicator = Communicator(uart)

# 模式定义
MODE_TARGET = 0
MODE_BOUNDARY_UD = 1
MODE_BOUNDARY_LR = 2
MODE_WAITING = 3
current_mode = MODE_WAITING

# 时间
clock = time.clock()

# 时间戳
last_time = time.ticks_ms()

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
        LED(1).on()
        LED(1).off()
        continue

    elif current_mode == MODE_TARGET:
        LED(2).on()
        LED(2).off()
        # print("1")
        # 获取图像并进行预处理
        all_blobs_with_color = color_detector.detect_colors(img)
        filtered_blobs_with_color = color_detector.filter_all_blobs(all_blobs_with_color)

        # 分离棕色与其它
        brown_blobs = []
        other_blobs = []

        for item in filtered_blobs_with_color:
            blob = item[0]
            color = item[1]
            if color == 'brown':
                brown_blobs.append(blob)
            else:
                other_blobs.append((blob, color))

        # 绘制筛选后的色块（不同颜色用不同框区分）
        draw_colors = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'brown': (255, 255, 255),
            'grey': (100, 100, 100)
        }

        center = []

        # 处理棕色
        brown_detected = False
        if brown_blobs:
            target_brown = max(brown_blobs, key = lambda b:b.area())
            # if not brown_tracker.first_detected: # or brown_tracker.is_valid(target_brown)
            cx, cy = target_brown.cx(), target_brown.cy()
            w, h = target_brown.w(), target_brown.h()
            max_speed = 80
            dx_raw = (cx - brown_tracker.last_cx) / Ts
            dx = max(-max_speed, min(max_speed, dx_raw))
            dy_raw = (cy - brown_tracker.last_cy) / Ts
            dy = max(-max_speed, min(max_speed, dy_raw))
            Z = np.array([cx, cy, w, h, dx, dy], dtype = np.float)
                # 如果是第一次检测到小熊，则初始化卡尔曼滤波器
            if not brown_tracker.first_detected:
                # 初始化卡尔曼滤波器的状态和协方差矩阵
                brown_tracker.x_hat = np.array([cx, cy, w, h, 0, 0], dtype=np.float)
                brown_tracker.p = np.diag([10.0, 10.0, 5.0, 5.0, 100.0, 100.0])
                brown_tracker.first_detected = True  # 标记为已首次检测过
            brown_tracker.kalman_filter(Z, Ts, True)
            brown_tracker.last_cx = cx
            brown_tracker.last_cy = cy
            brown_detected = True
            img.draw_rectangle(target_brown.rect(), color=draw_colors['brown'])  # 画矩形框
            img.draw_cross(cx, cy, color=draw_colors['brown'])  # 画中心点
        # else:
            # brown_tracker.brown_visible_frames = 0
        # 无检测时预测
        if not brown_detected and brown_tracker.first_detected:
            if brown_tracker.lost_count < brown_tracker.MAX_LOST_FRAMES:
                brown_tracker.kalman_filter(None, Ts, is_detected=False)
            else:
                brown_tracker.reset()

        if brown_tracker.first_detected:
            kcx, kcy = int(brown_tracker.x_hat[0]), int(brown_tracker.x_hat[1])
            kw, kh = max(1, int(brown_tracker.x_hat[2])), max(1, int(brown_tracker.x_hat[3]))
            img.draw_rectangle(kcx - kw//2, kcy - kh//2, kw, kh, color=draw_colors['grey'])
            img.draw_cross(kcx, kcy, color=draw_colors['grey'])
            center.append((kcx, kcy))

        for item in other_blobs:
            # 绘制该颜色的所有筛选后色块
            blob = item[0]
            color_name = item[1]
            img.draw_rectangle(blob.rect(), color=draw_colors[color_name])  # 画矩形框
            img.draw_cross(blob.cx(), blob.cy(), color=draw_colors[color_name])  # 画中心点
            center_x = blob.cx()
            center_y = blob.cy()
            center.append((center_x, center_y))
        if center:
            target = max(center, key = lambda coordinate : coordinate[1]) # 选择最靠近小车的坐标（判断依据为y最大的坐标）
            target_x, target_y = target
            communicator.send_coordinate(target_x, target_y)

    elif current_mode == MODE_BOUNDARY_UD:
        LED(3).on()
        LED(3).off()
        angle = boundary_detector.boundary_correction('row', img)
        if angle is not None:
            communicator.send_angle(angle)

    elif current_mode == MODE_BOUNDARY_LR:
        LED(4).on()
        LED(4).off()
        angle = boundary_detector.boundary_correction('column', img)
        if angle is not None:
            communicator.send_angle(angle)

    lcd.show_image(img, 160, 120, zoom=0)
    print(clock.fps())
