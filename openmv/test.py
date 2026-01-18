    import sensor, image, time

    sensor.reset() # 初始化摄像头
    sensor.set_pixformat(sensor.RGB565) # 格式为 RGB565.
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(10) # 跳过10帧，使新设置生效
    sensor.set_auto_whitebal(False)               # Create a clock object to track the FPS.

    ROI=(80,30,15,15)

    while(True):
        img = sensor.snapshot()         # Take a picture and return the image.
        statistics=img.get_statistics(roi=ROI)
        color_l=statistics.l_mode()
        color_a=statistics.a_mode()
        color_b=statistics.b_mode()
        print(color_l,color_a,color_b)
        img.draw_rectangle(ROI)