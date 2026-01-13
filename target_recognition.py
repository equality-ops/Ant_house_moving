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

thresholds = [
    (32, 100, -128, -12, -128, 127),
    (42, 100, -128, -19, -128, 127),  # 网球
    (0, 57, 27, 127, 7, 127),
    (0, 56, 9, 85, -2, 53),           # 红沙包
    (34, 64, -18, 10, -128, -39),
    (30, 100, -30, -5, -48, -9),      # 蓝沙包
    (23, 100, -11, 39, -24, 64),
    (37, 100, -17, 13, 6, 127)        # 棕熊
]

roi = (15, 10, 133, 100)

# 关键参数：坐标距离阈值（像素）
# 两个框的中心距离小于此值，就认为是“过近”，只保留一个
DISTANCE_THRESHOLD = 30  # 可根据实际调整

# 计算两个坐标的距离
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

while(True):
    clock.tick()
    img = sensor.snapshot()

    # 第一步：获取所有符合基础过滤的色块（merge=False）
    all_blobs = img.find_blobs(thresholds,
                               roi = roi,
                               pixels_threshold=300,
                               area_threshold=300,
                               merge=False)

    # 第二步：筛选出“不重叠”的框（坐标过近的只保留一个）
    filtered_blobs = []
    for blob in all_blobs:
        if blob.w() > 60:
            continue  # 宽度过大，舍弃该框
        # 取当前框的中心坐标
        cx, cy = blob.cx(), blob.cy()
        # 标记是否需要保留当前框
        keep_blob = True

        # 对比已保留的框，判断距离是否过近
        for saved_blob in filtered_blobs:
            saved_cx, saved_cy = saved_blob.cx(), saved_blob.cy()
            distance = calculate_distance(cx, cy, saved_cx, saved_cy)
            if distance < DISTANCE_THRESHOLD:
                keep_blob = False  # 距离过近，不保留
                break

        # 若距离都不近，保留当前框
        if keep_blob:
            filtered_blobs.append(blob)

    # 第三步：只画筛选后的框（无重叠）
    for blob in filtered_blobs:
        img.draw_rectangle(blob.rect())

    # 显示帧率和筛选前后的框数量（调试用）
    print(f"FPS：{clock.fps()}")
