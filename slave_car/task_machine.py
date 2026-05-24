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
                plan_data.current_path = path_message[1]
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
                plan_data.current_path = path_message[1]
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

'''