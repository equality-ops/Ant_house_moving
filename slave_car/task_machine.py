# 视觉伺服测试函数
'''
def test_vision_servo():
    global counter
    if my_state.state == my_state.READY_NAVIGATE:
        if my_vision_manager.if_send_servo_command == False:
            my_vision_manager.if_send_servo_command = True
            my_vision_manager.my_order_manager.mode_target()
        # my_plan.finish_navigate = False
        target_point = my_art_protocol.coordinate_receive()
        if target_point:
            my_vision_manager.current_servo_object = target_point[2]
            my_vision_manager.ready_servo_and_orbit(target_point)
            my_state.state = my_state.SERVO
            # 测试
            my_beep.test()
    elif my_state.state == my_state.SERVO:
        my_vision_manager.visual_servo_control()
        if my_vision_manager.finish_servo == True:
            my_state.state = my_state.STOP
            my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
            # 重置标志位
            my_vision_manager.if_send_servo_command = False
            my_vision_manager.finish_servo = False
            # 测试
            # my_beep.test()
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(-120.0)
        if my_vision_manager.finish_orbit == True:
                counter += 1
                if counter >= 50:
                    my_vision_manager.finish_orbit = False
                    my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                    my_state.state = my_state.MOVE
                    my_plan.move_v_max = 60
                    # 测试
                    # my_beep.test()
    elif my_state.state == my_state.MOVE:
        my_plan.navigate([[my_car.x_current, my_car.y_current-150.0]])
        if my_plan.finish_navigate == True:
            my_plan.finish_navigate = False
            my_state.state = my_state.STOP
            # 测试
            # my_beep.test()
    elif my_state.state == my_state.STOP:
        my_plan.stop()

# 测试环绕控制函数
def test_orbit_control():
    if my_state.state == my_state.READY_NAVIGATE:
        my_state.state = my_state.ORBIT
    elif my_state.state == my_state.ORBIT:
        my_vision_manager.orbit_control(120.0)
        if my_vision_manager.finish_orbit == True:
            my_vision_manager.finish_orbit = False
            my_plan.turn_angle_target = my_car.now_yaw * 180.0 / MATH.PI
            my_state.state = my_state.STOP
            # 测试
            my_beep.test()
    elif my_state.state == my_state.STOP:
        pass

# 视觉伺服辅助apriltag码矫正
def test_apriltag_calibrate():
    if my_state.state == my_state.READY_NAVIGATE:
        my_order_manager.mode_apriltag()
        my_state.state = my_state.CALIBRATE
    elif my_state.state == my_state.CALIBRATE:
        # my_vision_manager.apriltag_calibrate_control()
        my_vision_manager.improved_aptiltag_calibrate()
        if my_vision_manager.if_finish_calibrate == True:
            my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
            my_state.state = my_state.STOP
    elif my_state.state == my_state.STOP:
        my_plan.stop()

# 双车版的任务执行机
def collaborative_task_machine():
    global counter
    if my_state.state_work == DOWN:
        if my_state.state == my_state.READY_NAVIGATE:
            path_message = my_slave_protocol.get_path_list()
            if path_message:
                my_slave_protocol.aimed_object = path_message[0] 
                plan_data.current_path = [path_message[2]] # 封装成列表形式供 navigate 遍历
                my_slave_protocol.send_slave_state("get")
                my_state.state = my_state.NAVIGATE
                # 当传来的坐标点的纵坐标大于170.0时，将状态工作设为UP，控制小车绕到矩形上边沿
                if my_slave_protocol.aimed_object == 'P':
                    if plan_data.current_path[0][1] > 170.0:
                        my_state.state_work = UP
                        return 
                    # 当传来的坐标点为从车起点时，将状态工作设为RETURN_WORK，控制小车返回起点
                    elif abs(plan_data.current_path[0][0] - plan_data.fixed_point[0][0]) < 1.0 and abs(plan_data.current_path[0][1] - plan_data.fixed_point[0][1]) < 1.0:
                        my_state.state_work = RETURN_WORK
                        my_state.state = my_state.RETURN
                        return
        elif my_state.state == my_state.NAVIGATE:
            my_plan.navigate(plan_data.current_path, 0.0)
            if my_plan.finish_navigate == True:
                if my_slave_protocol.aimed_object == 'P':
                    my_plan.finish_navigate = False
                    my_state.state = my_state.READY_NAVIGATE
                else:
                    if my_vision_manager.if_send_servo_command == False:
                        my_vision_manager.my_order_manager.mode_target()
                        my_vision_manager.if_send_servo_command = True
                    target_point = my_art_protocol.coordinate_receive()
                    # 与主车发送的物体种类进行对比，若相同则开始搬运，否则lost
                    if target_point and (target_point[2] == ord(my_slave_protocol.aimed_object)):
                        counter = 0
                        my_vision_manager.current_servo_object = target_point[2]
                        my_vision_manager.ready_servo_and_orbit(target_point)
                        my_plan.reset_navigate()
                        my_state.state = my_state.SERVO
                    else:
                        counter += 1
                        # 若连续2s没有收到openart发来的消息,强制小车进入视觉伺服模式
                        if counter >= 200:
                            counter = 0
                            my_vision_manager.if_lost_object = True
                            my_plan.reset_navigate()
                            my_state.state = my_state.SERVO
                            my_vision_manager.target_rel_turn_angle = my_plan.turn_angle_target

        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                my_plan.navigate([[my_car.x_current-15.0, my_car.y_current], [my_car.x_current-15.0, my_car.y_current-5.0], [my_car.x_current+15.0, my_car.y_current-5.0], [my_car.x_current+15.0, my_car.y_current+5.0], [my_car.x_current, my_car.y_current+5.0], plan_data.fixed_point[5]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point and (target_point[2] == ord(my_slave_protocol.aimed_object)):
                    my_vision_manager.current_servo_object = target_point[2]
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_vision_manager.if_lost_object = False

                # 如果小车未找到物体，向主车发送lost指令
                if my_plan.finish_navigate == True:
                    my_vision_manager.failed_servo_count += 1
                    # 重置视觉伺服失败次数
                    if my_vision_manager.failed_servo_count >= 2:
                        my_vision_manager.failed_servo_count = 0
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_send_servo_command = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 向主车发送丢失消息
                    my_slave_protocol.send_slave_state("lost")
                    my_state.state = my_state.READY_NAVIGATE

            if my_vision_manager.finish_servo == True:
                counter += 1
                # 延时200ms
                if counter >= 20:
                    # 重置计数器
                    counter = 0
                    my_state.state = my_state.ORBIT
                    # 重置标志位
                    my_vision_manager.if_send_servo_command = False
                    my_vision_manager.finish_servo = False
                    # 重置视觉伺服失败次数
                    my_vision_manager.failed_servo_count = 0
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
        elif my_state.state == my_state.ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                counter += 1
                # 延时200ms防止惯性过冲
                if counter >= 20:
                    # 测试
                    counter = 0
                    my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                    # 测试搬运角度是否合适
                    # my_state.state = my_state.STOP
                    my_state.state = my_state.MOVE
                    my_slave_protocol.send_slave_state("finish")
                    # 提前设置小车转向目标角度为当前角度
                    my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI            
        elif my_state.state == my_state.MOVE:
            # 搬运小熊时搬远一些防止与主车或者物体卡住
            if my_vision_manager.current_servo_object == ord('W') or my_vision_manager.current_servo_object == ord('B'):
                my_plan.navigate([[my_car.x_current+my_plan.error_x, -35.0]])
            else:
                my_plan.navigate([[my_car.x_current+my_plan.error_x, -25.0]])
            if my_plan.finish_navigate == True:
                counter = 0
                if my_slave_protocol.get_start_signal():
                    my_plan.finish_navigate = False
                    my_vision_manager.car_position = DOWN_RIGHT
                    my_state.state = my_state.CALIBRATE
                    my_order_manager.finish()
        elif my_state.state == my_state.CALIBRATE:
            # 延时800ms在进行apriltag矫正防止与主车相碰
            if counter <= 80:
                counter += 1
            else:
                if my_vision_manager.if_lost_object == False:
                    my_vision_manager.apriltag_calibrate_control()
                else:
                    # 控制小车前后移动寻找apriltag码
                    my_plan.navigate([[my_car.x_current+25.0, my_car.y_current], [my_car.x_current+25.0, my_car.y_current-15.0], [my_car.x_current-10.0, my_car.y_current-15.0], [my_car.x_current-10.0, my_car.y_current+15.0], [my_car.x_current, my_car.y_current+15.0]], my_vision_manager.target_rel_turn_angle)

                    target_point = my_art_protocol.apriltag_receive()
                    if target_point:
                        my_plan.reset_navigate()
                        my_vision_manager.counter = 0
                        my_vision_manager.calibrate_times = 0
                        my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                    # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车返回等待模式
                    if my_plan.finish_navigate == True:
                        counter = 0
                        # 重置标志位
                        my_plan.finish_navigate = False
                        my_vision_manager.if_lost_object = False
                        my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                        # 将openart置为等待模式
                        my_order_manager.finish()
                        my_state.state = my_state.READY_NAVIGATE

                if my_vision_manager.if_finish_calibrate == True:
                    counter = 0
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    my_state.state = my_state.READY_NAVIGATE
    elif my_state.state_work == UP:
        if my_state.state == my_state.READY_NAVIGATE:
            path_message = my_slave_protocol.get_path_list()
            if path_message:
                my_slave_protocol.aimed_object = path_message[0] 
                plan_data.current_path = [path_message[2]]
                my_slave_protocol.send_slave_state("get")
                my_state.state = my_state.NAVIGATE
                # 当传来的坐标点为从车起点时，将状态工作设为RETURN_WORK，控制小车返回起点
                if abs(plan_data.current_path[0][0] - plan_data.fixed_point[0][0]) < 1.0 and abs(plan_data.current_path[0][1] - plan_data.fixed_point[0][1]) < 1.0 and my_slave_protocol.aimed_object == 'P':
                    my_state.state_work = RETURN_WORK
                    my_state.state = my_state.RETURN
                    return
        elif my_state.state == my_state.NAVIGATE:
            my_plan.navigate(plan_data.current_path, 180.0)
            if my_plan.finish_navigate == True:
                # 按照主车发送的路径提前移动到指定的矩形边沿附近
                if my_slave_protocol.aimed_object == 'P':
                    my_plan.finish_navigate = False
                    my_state.state = my_state.READY_NAVIGATE
                else:
                    if my_vision_manager.if_send_servo_command == False:
                        my_vision_manager.my_order_manager.mode_target()
                        my_vision_manager.if_send_servo_command = True
                    target_point = my_art_protocol.coordinate_receive()
                    # 与主车发送的物体种类进行对比，若相同则开始搬运，否则lost
                    if target_point and (target_point[2] == ord(my_slave_protocol.aimed_object)):
                        counter = 0
                        my_vision_manager.current_servo_object = target_point[2]
                        my_vision_manager.ready_servo_and_orbit(target_point)
                        my_plan.reset_navigate()
                        my_state.state = my_state.SERVO
                    else:
                        counter += 1
                        # 若连续2s没有收到openart发来的消息,强制小车进入视觉伺服模式
                        if counter >= 200:
                            counter = 0
                            my_vision_manager.if_lost_object = True
                            my_plan.reset_navigate()
                            my_state.state = my_state.SERVO
                            my_vision_manager.target_rel_turn_angle = my_plan.turn_angle_target
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                my_plan.navigate([[my_car.x_current+15.0, my_car.y_current], [my_car.x_current+15.0, my_car.y_current+5.0], [my_car.x_current-15.0, my_car.y_current+5.0], [my_car.x_current-15.0, my_car.y_current-5.0], [my_car.x_current, my_car.y_current-5.0], plan_data.fixed_point[6]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point and (target_point[2] == ord(my_slave_protocol.aimed_object)):
                    my_vision_manager.current_servo_object = target_point[2]
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_vision_manager.failed_servo_count += 1
                    # 重置视觉伺服失败次数
                    if my_vision_manager.failed_servo_count >= 2:
                        my_vision_manager.failed_servo_count = 0
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_send_servo_command = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 向主车发送丢失消息
                    my_slave_protocol.send_slave_state("lost")
                    my_state.state = my_state.READY_NAVIGATE

            if my_vision_manager.finish_servo == True:
                counter += 1
                # 延时200ms
                if counter >= 20:
                    # 重置计数器
                    counter = 0
                    my_state.state = my_state.ORBIT
                    # 重置视觉伺服失败次数
                    my_vision_manager.failed_servo_count = 0
                    # 重置标志位
                    my_vision_manager.if_send_servo_command = False
                    my_vision_manager.finish_servo = False
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
        elif my_state.state == my_state.ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                counter += 1
                # 延时200ms防止惯性过冲
                if counter >= 20:
                    counter = 0
                    my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                    my_state.state = my_state.MOVE
                    # 提前设置小车转向目标角度为当前角度
                    my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                    my_slave_protocol.send_slave_state("finish")
        elif my_state.state == my_state.MOVE:
            # 搬运小熊时搬远一些防止与主车或者物体卡住
            if my_vision_manager.current_servo_object == ord('W') or my_vision_manager.current_servo_object == ord('B'):
                my_plan.navigate([[my_car.x_current-my_plan.error_x, 275.0]])
            else:
                my_plan.navigate([[my_car.x_current-my_plan.error_x, 265.0]])
            if my_plan.finish_navigate == True:
                counter = 0
                if my_slave_protocol.get_start_signal():
                    my_plan.finish_navigate = False
                    my_vision_manager.car_position = UP_LEFT
                    my_state.state = my_state.CALIBRATE
                    my_order_manager.finish()
        elif my_state.state == my_state.CALIBRATE:
            # 延时0.8s在进行apriltag矫正防止与主车相碰
            if counter <= 80:
                counter += 1
            else:
                if my_vision_manager.if_lost_object == False:
                    my_vision_manager.apriltag_calibrate_control()
                else:
                    # 控制小车移动寻找apriltag码
                    my_plan.navigate([[my_car.x_current-25.0, my_car.y_current], [my_car.x_current-25.0, my_car.y_current+15.0], [my_car.x_current+10.0, my_car.y_current+15.0], [my_car.x_current+10.0, my_car.y_current-15.0], [my_car.x_current, my_car.y_current-15.0]], my_vision_manager.target_rel_turn_angle)
                    
                    target_point = my_art_protocol.apriltag_receive()
                    if target_point:
                        my_plan.reset_navigate()
                        my_vision_manager.counter = 0
                        my_vision_manager.calibrate_times = 0
                        my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                    # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车返回等待模式
                    if my_plan.finish_navigate == True:
                        counter = 0
                        # 重置标志位
                        my_plan.finish_navigate = False
                        my_vision_manager.if_lost_object = False
                        my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                        # 将openart置为等待模式
                        my_order_manager.finish()
                        my_state.state = my_state.READY_NAVIGATE

                if my_vision_manager.if_finish_calibrate == True:
                    counter = 0
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    my_state.state = my_state.READY_NAVIGATE
    elif my_state.state_work == RETURN_WORK:
        if my_state.state == my_state.RETURN:
            # 最终返回从车的起点（避免回程途中与主车碰撞）
            my_plan.navigate([[25.0, -20.0]])
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.STOP
                my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
        elif my_state.state == my_state.STOP:
            my_plan.stop()

            [[ 2.53412974e+00, -6.38412017e-02, -1.99675761e+02],
            [-3.71806303e-16, -2.58229617e+00,  2.68558802e+02],
            [-1.26680170e-17,  8.36909871e-02,  1.00000000e+00]]
    # 视觉伺服
    # my_uart3.write(f"object: {my_vision_manager.current_servo_object}, servo_pid.target_y: {servo_pid.target_y}, object_radius: {my_vision_manager.object_radius}, orbit_angle: {my_vision_manager.orbit_angle}\n")
    # my_uart3.write(f"servo_pid.target_y: {servo_pid.target_y}, object_radius: {my_vision_manager.orbit_radius}\n")
    # my_uart3.write("x: {:<f}, y: {:<f}, speed: {:<f}, yaw: {:<f},  {:<f},{:<f}\n".format(servo_pid.actual_x, servo_pid.actual_y, my_vision_manager.target_rel_speed, my_vision_manager.target_rel_yaw, servo_pid.pwm_output_x, servo_pid.pwm_output_y))
    # my_uart3.write(f"{my_vision_manager.target_rel_speed},{my_vision_manager.target_rel_yaw},{my_vision_manager.target_rel_turn_angle}\r\n")
    # my_uart3.write(f"{my_vision_manager.target_rel_speed_x},{my_vision_manager.target_rel_speed_y},{my_vision_manager.target_rel_yaw}\r\n")
    # my_uart3.write("{:<f},{:<f}\n".format(ant_plan.my_vision_manager.target_rel_yaw, ant_plan.my_vision_manager.target_rel_yaw_fil))
    # my_uart3.write(f"{my_vision_manager.angle_buffer},{my_vision_manager.calibrate_times}\n")
    # my_uart3.write(f"{my_vision_manager.angle_buffer}\n")
    # my_uart3.write(f"{my_vision_manager.relative_raw_x},{my_vision_manager.relative_raw_y}\n")
    # my_uart3.write(f"{my_vision_manager.real_servo_point}\n")
    # 速度环输出波形图调参
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ul_pid.pwm_output, motor_ul_pid.derivative * motor_ul_pid.kd, motor_ul_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f}\n".format(motor_ur_pid.target, motor_ur_pid.actual, motor_ur_pid.pwm_output, motor_ur_pid.derivative * motor_ur_pid.kd, motor_ur_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_md_pid.target, motor_md_pid.actual, motor_md_pid.pwm_output, motor_md_pid.derivative * motor_md_pid.kd, motor_md_pid.integral))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(motor_ul_pid.target, motor_ul_pid.actual, motor_ur_pid.target, motor_ur_pid.actual,motor_md_pid.target, motor_md_pid.actual))
        
    # 角度环输出
    # my_uart3.write(f"{angle_pid.pwm_output},{angle_pid.target},{angle_pid.actual}\n")
    # my_uart3.write(f"{pose_data.gyro_z}, {pose_data.gyro_z_bias}\n")
    # imu原始数据
    # my_uart3.write("acc = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[0], pose_data.imu_data[1], pose_data.imu_data[2]))
    # my_uart3.write("gyro = {:>6d}, {:>6d}, {:>6d}\n".format(pose_data.imu_data[3], pose_data.imu_data[4], pose_data.imu_data[5]))
                                                                          
    # 里程计：
    # my_uart3.write(f"{my_plan.target_yaw}\n")
    # my_uart3.write("ul: {:<f}, ur: {:<f}, md: {:<f}\n".format(my_car.encouder_ul, my_car.encouder_ur, my_car.encouder_md))
    # my_uart3.write("now: {:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_car.now_yaw * 180 / PI, angle_pid.pwm_output))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(my_car.x_current, my_car.y_current, my_plan.rest_distance, my_plan.target_v, my_car.now_yaw * 180 / PI, my_plan.arrive_flag))
    
    # my_uart3.write(f"{my_car.angle_pid.target}, {my_car.angle_pid.actual}, {my_car.angle_pid.nowError}, {my_state.state}\n")
    
    # 路径规划
    # my_uart3.write(f"{my_plan.current_path}\n")
    
    # tof传感器测试
    # my_uart3.write(f"{tof_distance_fil.update(tof.get())},{tof.get()}\r\n")

    # 测试边线校准
    # my_uart3.write(f"{my_plan.calibrate_angle}\n")
    
    # 速度规划
    # my_uart3.write(("target_v: %d, rest_dis: %.3f, dec_speed_index: %d\r\n") % (ant_plan.my_plan.target_v, ant_plan.my_plan.rest_distance, ant_plan.my_plan.dec_speed_index))
    # my_uart3.write(f"{my_plan.stage}")
    # 检测自转角是否准确
    # my_uart3.write("{:<f}\n".format(my_car.now_yaw * 180 / PI))
    
    # 观察速度
    # my_uart3.write(f"{motor_ul_pid.target},{motor_ul_pid.actual}\n")
    
    # 检测四元数解算结果是否准确
    # my_uart3.write(f"{pose_data.now_pitch},{pose_data.now_roll},{pose_data.now_yaw}\n")
    # my_uart3.write(f"{pose_data.q[0]},{pose_data.q[1]},{pose_data.q[2]},{pose_data.q[3]}\n")

    # 检测gkd项数量级
    # my_uart3.write(f"{pose_data.gyro_z * my_car.gkd}, {pose_data.gyro_z}\n")
    
    # 环绕测试
    # my_uart3.write(f"{my_vision_manager.orbit_radius}\n")
    # my_uart3.write(f"{my_vision_manager.x_coordinate},{orbit_pid.pwm_output_x},{orbit_pid.pwm_output_y},{my_vision_manager.target_rel_yaw},{my_vision_manager.target_rel_turn_angle}\n")
    # 卡尔曼滤波（速度）
    # my_uart3.write("{:<f},{:<f},{:<f}\n".format(ant_motor.my_car.car_speed_x, ant_motor.speed_x_fil.update(ant_motor.my_car.car_speed_x), ant_motor.speed_x_fil2.filtering(ant_motor.my_car.car_speed_x)))
    # my_uart3.write("{:<f},{:<f},{:<f},{:<f},{:<f},{:<f}\n".format(pose_data.encoder_data_ul, pose_data.encoder_data_ul_2,pose_data.encoder_data_ur, pose_data.encoder_data_ur_2,pose_data.encoder_data_md, pose_data.encoder_data_md_2))
    # my_uart3.write("{:<f},{:<f},{:<f}\n".format(pose_data.encoder_data_ul,pose_data.encoder_data_ur,pose_data.encoder_data_md))

    # 任务机
    # my_uart3.write(f"servo: {my_vision_manager.target_rel_turn_angle}, plan: {my_plan.turn_angle_target}\n")
    # my_uart3.write(f"state_work: {my_state.state_work}, state: {my_state.state}, yaw: {my_car.now_yaw * 180 / PI}, current_object: {my_vision_manager.current_servo_object}, {my_plan.turn_angle_target}\n")

    # 搬运检查模式
    # my_uart3.write(f"pickup_check: {my_vision_manager.lost_object_frames}\n")
'''