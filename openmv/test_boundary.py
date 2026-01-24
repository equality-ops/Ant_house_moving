# 导入必要的库
import sensor
import time
import image

# 初始化摄像头
sensor.reset()
sensor.set_vflip(False)      # 不翻转垂直方向
sensor.set_hmirror(False)    # 不镜像水平方向
sensor.set_pixformat(sensor.RGB565)   # 彩色模式
sensor.set_framesize(sensor.QQVGA)    # 分辨率：160x120
sensor.skip_frames(time=2000)         # 等待稳定

# 创建时钟对象用于计算 FPS
clock = time.clock()

# 设置颜色阈值（LAB 色彩空间）
# THRESHOLD = (L_min, L_max, A_min, A_max, B_min, B_max)
THRESHOLD = (57, 90, -26, -2, 50, 91)  # 示例：检测暗色区域（如黑色线）

while True:
    clock.tick()  # 开始计时
    img = sensor.snapshot().binary([THRESHOLD])  # 拍摄图像

    # 二值化处理：只保留满足阈值的像素
    img.binary([THRESHOLD])


    # 使用线性回归拟合直线
    # 注意：[(100,100)] 是错误写法！应为实际阈值或 [THRESHOLD]
    line = img.get_regression([(100, 100)], robust=True)

    # 判断是否检测到有效线条
    if line and line.magnitude() > 8:  # magnitude > 8 表示线性较好
        # 在图像上画出拟合的直线
        img.draw_line(line.line(), color=(255, 0, 0), thickness=2)
    else:
        # 如果未检测到线，发送空字符给主控
        print("no founded")

    # 输出当前帧率
    print("FPS: %.2f" % clock.fps())
