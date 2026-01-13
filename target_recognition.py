import sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_framerate(60)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_brightness(1500)
sensor.skip_frames(time = 200)
clock = time.clock()

# 四种主要颜色的阈值
RED_THRESHOLD   = [(0, 57, 27, 127, 7, 127),
                   (0, 56, 9, 85, -2, 53)]# 红
GREEN_THRESHOLD = [(32, 100, -128, -12, -128, 127),
                   (42, 100, -128, -19, -128, 127)]# 绿
BLUE_THRESHOLD  = [(34, 64, -18, 10, -128, -39),
                   (30, 100, -30, -5, -48, -9)]# 蓝
BROWN_THRESHOLD = [(23, 100, -11, 39, -24, 64),
                   (37, 100, -17, 13, 6, 127)]# 棕


roi = (15, 10, 133, 100)

# 关键参数：坐标距离阈值
# 两个框的中心距离小于此值，就认为是“过近”，只保留一个
DISTANCE_THRESHOLD = 30  # 可根据实际调整

# 计算两个坐标的距离
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def detect_colors(img):
    # 分别查找各颜色色块，得到各色块对象列表
    red_blobs   = img.find_blobs(RED_THRESHOLD, roi = roi,
    pixels_threshold=200, area_threshold=200, merge=False)
    green_blobs   = img.find_blobs(GREEN_THRESHOLD, roi = roi,
    pixels_threshold=200, area_threshold=200, merge=False)
    blue_blobs   = img.find_blobs(BLUE_THRESHOLD, roi = roi,
    pixels_threshold=200, area_threshold=200, merge=False)
    brown_blobs   = img.find_blobs(BROWN_THRESHOLD, roi = roi,
    pixels_threshold=200, area_threshold=200, merge=True)

    all_blobs_with_color = [] # 用来存储（色块对象，颜色名称）元组的列表
    for blob in red_blobs:
        all_blobs_with_color.append((blob, 'red'))
    for blob in green_blobs:
        all_blobs_with_color.append((blob, 'green'))
    for blob in blue_blobs:
        all_blobs_with_color.append((blob, 'blue'))
    for blob in brown_blobs:
        all_blobs_with_color.append((blob, 'brown'))
    return all_blobs_with_color # 返回这个列表

# 对所有色块去重
def filter_all_blobs(blobs):
    filtered = [] # 一个用以存储所有去重后（色块对象，颜色名字）的列表
    for item in blobs:
        blob = item[0]
        # 过滤宽度过大的色块
        if blob.w() > 60:
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
    return filtered # 返回去重后的列表

while(True):
    clock.tick()
    img = sensor.snapshot()

    all_blobs_with_color = detect_colors(img) # 获取所有对象和它们的名字组成的列表

    filtered_blobs_with_color = filter_all_blobs(all_blobs_with_color) # 将这个列表进行去重

    # 绘制筛选后的色块（不同颜色用不同框区分）
    # 颜色映射：键=颜色名称，值=RGB颜色值
    draw_colors = {
        'red': (255, 0, 0),
        'green': (0, 255, 0),
        'blue': (0, 0, 255),
        'brown': (255, 255, 255)
    }
    for item in filtered_blobs_with_color:
        # 绘制该颜色的所有筛选后色块
        blob = item[0]
        color_name = item[1]
        img.draw_rectangle(blob.rect(), color=draw_colors[color_name])  # 画矩形框
        img.draw_cross(blob.cx(), blob.cy(), color=draw_colors[color_name])  # 画中心点
    print(f"FPS: {clock.fps()}")
