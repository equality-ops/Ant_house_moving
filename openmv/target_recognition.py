import sensor, image, time, math, mjpeg
from pyb import LED
from machine import UART

##########################串口初始化#########################
uart = UART(2, baudrate=115200)
uart.write("uart test\r\n")

##########################摄像头初始化########################
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_framerate(60)
# sensor.set_auto_gain(False) # 自动增益
sensor.set_auto_whitebal(True)
sensor.set_brightness(1500)
# sensor.set_contrast(2) # 对比度
sensor.skip_frames(time = 200)
clock = time.clock()

################# #######变量定义##########################

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
    #获取图像并进行预处理
    all_blobs_with_color = detect_colors(img)
    filtered_blobs_with_color = filter_all_blobs(all_blobs_with_color)

    # 绘制筛选后的色块（不同颜色用不同框区分）
    draw_colors = {
        'red': (255, 0, 0),
        'green': (0, 255, 0),
        'blue': (0, 0, 255),
        'brown': (255, 255, 255)
    }

    center = []
    for item in filtered_blobs_with_color:
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
