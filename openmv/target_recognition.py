import sensor, image, time, math, mjpeg
from pyb import LED
from machine import UART
from ulab import numpy as np

##########################串口初始化#########################
uart = UART(2, baudrate=115200)
uart.write("uart test\r\n")

##########################摄像头初始化########################
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_framerate(60)
sensor.set_auto_gain(False) # 自动增益
sensor.set_auto_whitebal(True)
sensor.set_brightness(1500)
# sensor.set_contrast(2) # 对比度
sensor.skip_frames(time = 200)
clock = time.clock()

######################最小变化阈值滤波#######################
position_threshold = 4 # 位置变化的最小阈值 值越小位置识别更新的越频繁，值越大小球的细微运动越不会更新识别
MAX_CHANGE_THRESHOLD = 80 # 最大变化阈值（位置和半径超过此值时不更新）
# 上一帧的矩形中心坐标
prev_x, prev_y = None, None

#######################卡尔曼滤波##########################
last_time = time.ticks_ms()
# 观测矩阵 C，描述从状态到观测值的映射关系 C 是观测矩阵，它将状态向量（位置、速度）与观测量（图像中的矩形框信息）联系起来。这里假设观测量是位置和速度。
C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],
[0,0,0,0,1,0],[0,0,0,0,0,1]])
# 过程噪声协方差矩阵 Q，用于描述过程的随机噪声
Q_value = [1e-6 for _ in range(6)]
Q = np.diag(Q_value) # 更新过程噪声协方差矩阵
# 观测噪声协方差矩阵 R 是观测噪声协方差矩阵，表示观测过程中测量误差的大小。
R_value = [1e-6 for _ in range(6)]
R = np.diag(R_value)
# 定义观测量Z
x = 0 # 左顶点x坐标
y = 0 # 左顶点y坐标
last_frame_x = x # 上一帧左顶点x坐标
last_frame_y = y # 上一帧左顶点y坐标
w = 0 # 矩形框宽度w
h = 0 # 矩形框高度h
dx = 0 # 左顶点x坐标移动速度
dy = 0 # 左顶点y坐标移动速度
Z = np.array([x, y, w, h, dx, dy])
# 初始状态估计
x_hat = np.array([80, 60, 30, 30, 2, 2]) #初始估计的状态值（位置，速度）
x_hat_minus = np.array([0,0,0,0,0,0]) # 初始预测的状态值
p_value = [10 for _ in range(6)] # 状态误差的初始值 p 是状态误差的初始协方差矩阵。

# 卡尔曼滤波函数
#预测阶段：利用状态转移矩阵和上一状态估计预测当前状态。
#校正阶段：通过卡尔曼增益对预测状态进行校正，使得估计值接近真实值。
#输入 Z：观测值（或测量值），通常是来自外部传感器（例如相机、雷达等）的数据。在这个代码中，Z 是
    #一个包含目标的位置信息（如矩形框的四个角坐标）的向量，格式为 [x, y, w, h, dx, dy]，其中 x 和
    #y 是目标的中心位置，w 和 h 是目标的宽度和高度，dx 和 dy 是目标的速度。
#输出 x_hat：更新后的状态估计，包括位置（x, y）、宽度（w, h）、速度（dx, dy）。该值是通过卡尔
    #曼滤波器的预测和校正步骤计算得到的最优估计。
def Kalman_Filter(Z,Ts):
    global C,Q,R,x_hat,x_hat_minus,p
    A = np.array([
        [1, 0, 0, 0, Ts, 0],
        [0, 1, 0, 0, 0, Ts],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ])
    # 预测部分
    x_hat_minus = np.dot(A,x_hat)
    p_minus = np.dot(A,np.dot(p,A.T)) + Q
    # 校正部分
    S = np.dot(np.dot(C,p_minus),C.T) + R
    # 选择一个小的正则化项
    regularization_term = 1e-4
    # 正则化S矩阵
    S_regularized = S + regularization_term * np.eye(S.shape[0])
    # 计算正则化后的S矩阵的逆
    S_inv = np.linalg.inv(S_regularized)
    # 计算卡尔曼增益
    K = np.dot(np.dot(p_minus,C.T),S_inv)
    x_hat = x_hat_minus + np.dot(K,(Z - np.dot(C,x_hat_minus)))
    p = np.dot((np.eye(6) - np.dot(K,C)),p_minus)
    return x_hat

last_frame_location = [0 for _ in range(4)] #用于存储上一帧的目标位置，这通常用于目标跟踪和计算目标移动等任务。一个长度为4的列表 last_frame_location，其中每个元素的初始值为 0
last_frame_rect = [0 for _ in range(4)] #存储上一帧检测到的矩形框坐标 成了一个长度为4的列表 last_frame_rect，并且每个元素的初始值为 0。
box = [0 for _ in range(4)]

########################变量定义##########################

# 四种主要颜色的阈值
RED_THRESHOLD   = [(0, 57, 27, 127, 7, 127),
                   (0, 56, 9, 85, -2, 53)]# 红
GREEN_THRESHOLD = [(32, 100, -128, -12, -128, 127),
                   (42, 100, -128, -19, -128, 127)]# 绿
BLUE_THRESHOLD  = [(34, 64, -18, 10, -128, -39),
                   (30, 100, -30, -5, -48, -9)]# 蓝
BROWN_THRESHOLD = [#(32, 100, -11, 12, -16, 127),
                   (0, 100, -128, 23, -7, 127)]# 棕


# 感兴趣的区域
roi = (0, 0, 160, 120)

# 录像
"""
red = LED(1)
green = LED(2)
#视频文件地址
m = mjpeg.Mjpeg("/sd/example.mjpeg")
#记录视频有多少帧
fps_count = 0;
"""

# 坐标距离阈值（两个框的中心距离小于此值，就认为是“过近”，只保留一个）
DISTANCE_THRESHOLD = 30  # 可根据实际调整

##########################函数定义##########################
# 计算两个坐标的距离
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# 分别查找各颜色色块
def detect_colors(img):
    brown_blobs   = img.find_blobs(BROWN_THRESHOLD,
    pixels_threshold=50, area_threshold=50, merge=True)
    red_blobs   = img.find_blobs(RED_THRESHOLD,
    pixels_threshold=30, area_threshold=30, merge=False)
    green_blobs   = img.find_blobs(GREEN_THRESHOLD,
    pixels_threshold=30, area_threshold=30, merge=False)
    blue_blobs   = img.find_blobs(BLUE_THRESHOLD,
    pixels_threshold=30, area_threshold=30, merge=False)

    all_blobs_with_color = []
    for blob in brown_blobs:
        all_blobs_with_color.append((blob, 'brown'))
    for blob in red_blobs:
        all_blobs_with_color.append((blob, 'red'))
    for blob in green_blobs:
        all_blobs_with_color.append((blob, 'green'))
    for blob in blue_blobs:
        all_blobs_with_color.append((blob, 'blue'))
    return all_blobs_with_color

# 对所有色块去重
def filter_all_blobs(blobs):
    filtered = []
    for item in blobs:
        blob = item[0]
        color = item[1]
        # 过滤宽度过大的色块
        if (
            # 长大于120，宽大于100，直接舍弃
            blob.w() > 120
            or blob.h() > 100

            # 棕色规则：边长比超过1.2:1
            or (
                color == 'brown'
                and (blob.w() > 1.2 * blob.h() or blob.h() > 1.2 * blob.w())
            )

            # 绿/蓝规则：边长比超过1.5:1
            or (
                color in ('green', 'blue')
                and (blob.w() > 1.5 * blob.h() or blob.h() > 1.5 * blob.w())
            )
        ):
            continue
        cx, cy = blob.cx(), blob.cy()
        keep = True
        # 对比已保留的色块，判断距离是否过近
        for saved_item in filtered:
            saved_blob = saved_item[0]
            distance = calculate_distance(cx, cy, saved_blob.cx(), saved_blob.cy())
            if distance < DISTANCE_THRESHOLD:
                keep = False
                break
        if keep:
            filtered.append(item)
    return filtered

############################主部分###########################
while(True):
    clock.tick()
    img = sensor.snapshot()
    current_time = time.ticks_ms()
    delta_time = time.ticks_diff(current_time,last_time)
    Ts = max(delta_time / 1000.0, 1 / 100)
    last_time = current_time

    # 录像
    # red.on()
    """
    #如果帧数没达到1000
    if fps_count < 1000:
        #保存当前图片为1帧
        m.add_frame(img)
        print(clock.fps())
        fps_count += 1
    else:
        #关闭文件才保存成功，需要传入保存视频的帧率，可以自己设定，参数填24表示保存的视频就是1秒钟播放24帧
        m.close(60)
        red.off()
        green.on()
        time.sleep_ms(500)
        break
    """
    # 获取图像并进行预处理
    all_blobs_with_color = detect_colors(img)
    filtered_blobs_with_color = filter_all_blobs(all_blobs_with_color)

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
        x, y, w, h = target_brown.rect()
        dx = (x - last_frame_x) / Ts
        dy = (y - last_frame_y) / Ts
        Z = np.array([x, y, w, h, dx, dy], dtype = np.float)
        x_hat = Kalman_Filter(Z, Ts)
        last_frame_x, last_frame_y = x, y
        brown_detected = True
        img.draw_rectangle(blob.rect(), color=draw_colors[color_name])  # 画矩形框
        img.draw_cross(blob.cx(), blob.cy(), color=draw_colors[color_name])  # 画中心点

    if not brown_detected:
        A_pred = np.array([
            [1, 0, 0, 0, Ts, 0],
            [0, 1, 0, 0, 0, Ts],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        x_hat = np.dot(A_pred, x_hat)

    prev_cx = int(x_hat[0] + x_hat[2] / 2)
    prev_cy = int(x_hat[1] + x_hat[3] / 2)
    center.append((prev_cx, prev_cy))
    img.draw_rectangle(blob.rect(), color=draw_colors["grey"])  # 画矩形框
    img.draw_cross(blob.cx(), blob.cy(), color=draw_colors['grey'])  # 画中心点  

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
        uart_data = f"X : {target_x} Y : {target_y}\n"
        uart.write(uart_data)
    print(f"FPS: {clock.fps()}")
