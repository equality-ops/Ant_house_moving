'''
# 双车版的任务执行机
def collaborative_task_machine():
    global counter, if_send_preparing_path
    if my_state.state_work == DOWN:
        if my_state.state == my_state.NAVIGATE:
            if if_send_preparing_path == False:
                my_main_protocol.send_path(ord('P'), [[15.0, 0.0], [plan_data.fixed_point[5][0], plan_data.fixed_point[5][1]]])
                if_send_preparing_path = True

            my_plan.navigate([[160.0, plan_data.fixed_point[1][1]]], 0.0)
            # my_plan.navigate([[35.0, -15.0]], 0.0)
            if my_plan.finish_navigate == True:
                # 重置标志位
                my_plan.finish_navigate = False
                if my_vision_manager.failed_servo_count >= 2:
                    my_vision_manager.failed_servo_count = 0
                    my_state.state = my_state.NAVIGATE
                    my_state.state_work = UP
                    if_send_preparing_path = False
                else:
                    my_state.state = my_state.SCAN
                    my_order_manager.mode_target()
                    my_art_protocol.send_object_kind('C')
        elif my_state.state == my_state.SCAN:
            my_plan.navigate([plan_data.fixed_point[1], plan_data.fixed_point[3]], 0.0)
            # my_plan.navigate([plan_data.fixed_point[3]], 0.0)
            if my_plan.finish_navigate == False:
                target_point = my_art_protocol.coordinate_receive()
                if target_point and target_point[1] > my_vision_manager.dist_threshold and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B') or target_point[2] == ord('E') or target_point[2] == ord('W')):  
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_state.state = my_state.SERVO
                    # 测试
                    my_beep.test()
            else:
                my_plan.finish_navigate = False
                if_send_preparing_path = False
                # 此时矩形下区域已没有物体，控制小车移动到上区域寻找物体
                my_state.state_work = UP
                my_state.state = my_state.NAVIGATE
                # 将openart置为等待模式
                my_order_manager.finish()
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                # 若丢失物体则按矩形轨迹行驶寻找物体
                my_plan.navigate([[my_car.x_current+11.0, my_car.y_current], [my_car.x_current+11.0, my_car.y_current-11.0], [my_car.x_current-11.0, my_car.y_current-11.0], [my_car.x_current-11.0, my_car.y_current], [my_car.x_current, my_car.y_current]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point and target_point[2] == my_vision_manager.current_servo_object and target_point[1] > my_vision_manager.dist_threshold:
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 重回扫描点继续寻找物体
                    # my_plan.return_to_scan_point = True
                    my_state.state = my_state.NAVIGATE

            if my_vision_manager.finish_servo == True:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(my_vision_manager.current_servo_object, [[my_car.x_current, plan_data.fixed_point[1][1]-10.0], [my_car.x_current, my_car.y_current-8.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    # 重置标志位
                    my_vision_manager.finish_servo = False
                    my_plan.if_send_path = False

                    my_state.state = my_state.ORBIT
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
                    # 将下一次的扫描点置为当前点偏左，控制小车在该区域内寻找物体
                    # plan_data.fixed_point[1][0] = my_car.x_current-10.0
        elif my_state.state == my_state.ORBIT:
            # 延时100ms，等待稳定后再开始环绕
            if counter <= 10:
                counter += 1
            else:
                my_vision_manager.orbit_control(my_vision_manager.orbit_angle)
                if my_vision_manager.finish_orbit == True:
                    order = my_main_protocol.get_slave_state()
                    if order == "finish":
                        counter = 0
                        # 重置从车视觉伺服失败次数
                        my_vision_manager.failed_servo_count = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_state.state = my_state.MOVE
                        # 提前设置小车转向目标角度为当前角度
                        my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                    elif order == "lost":
                        counter = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_vision_manager.failed_servo_count += 1
                        # my_state.state = my_state.REVERSE_ORBIT
        elif my_state.state == my_state.MOVE:
            # 控制小车夹紧物体，控制主车提前停止
            my_plan.navigate([[my_car.x_current+my_plan.error_x, -20.0]])
            if my_plan.finish_navigate == True:
                counter = 0
                my_plan.finish_navigate = False
                my_vision_manager.car_position = DOWN_LEFT
                my_state.state = my_state.CALIBRATE
        elif my_state.state == my_state.CALIBRATE:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.apriltag_calibrate_control()
            else:
                # 控制小车前后移动寻找apriltag码
                my_plan.navigate([[my_car.x_current-25.0, my_car.y_current], [my_car.x_current-25.0, my_car.y_current-15.0], [my_car.x_current+10.0, my_car.y_current-15.0], [my_car.x_current+10.0, my_car.y_current+15.0], [my_car.x_current, my_car.y_current+15.0]], my_vision_manager.target_rel_turn_angle)

                target_point = my_art_protocol.apriltag_receive()
                if target_point:
                    my_plan.reset_navigate()
                    my_vision_manager.counter = 0
                    my_vision_manager.calibrate_times = 0
                    my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车回到扫描点进行扫描
                if my_plan.finish_navigate == True:
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    my_state.state = my_state.NAVIGATE
                    # 主车给从车发消息让从车完成矫正
                    my_main_protocol.send_start()
            if my_vision_manager.if_finish_calibrate == True:
                my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                my_state.state = my_state.NAVIGATE
                # 主车给从车发消息让从车完成矫正
                my_main_protocol.send_start()
        # 让小车通过反向环绕恢复原位
        """
        elif my_state.state == my_state.REVERSE_ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                # 重回扫描点继续寻找物体
                my_plan.return_to_scan_point = True
                my_state.state = my_state.NAVIGATE
        """
    elif my_state.state_work == UP:
        if my_state.state == my_state.NAVIGATE:
            if if_send_preparing_path == False:
                # 操控从车从矩形区域左边沿行驶
                my_main_protocol.send_path(ord('P'), [[125.0, 220.0], [plan_data.fixed_point[6][0], plan_data.fixed_point[6][1]]])
                # 之后不用再重置该标志位
                if_send_preparing_path = True
                
            # my_plan.navigate([plan_data.fixed_point[2]], 180.0)
            my_plan.navigate([[160.0, plan_data.fixed_point[2][1]]], 180.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                if my_vision_manager.failed_servo_count >= 2:
                    my_vision_manager.failed_servo_count = 0
                    my_state.state_work = CHECK
                    my_state.state = my_state.NAVIGATE
                else:
                    my_state.state = my_state.SCAN
                    my_vision_manager.my_order_manager.mode_target()
                    my_art_protocol.send_object_kind('C')
        elif my_state.state == my_state.SCAN:
                # my_plan.navigate([plan_data.fixed_point[4]], 180.0)
                my_plan.navigate([plan_data.fixed_point[2], plan_data.fixed_point[4]], 180.0)
                if my_plan.finish_navigate == False:
                    target_point = my_art_protocol.coordinate_receive()
                    if target_point and target_point[1] > my_vision_manager.dist_threshold and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B') or target_point[2] == ord('E') or target_point[2] == ord('W')):
                        my_vision_manager.ready_servo_and_orbit(target_point)
                        my_plan.reset_navigate()
                        my_state.state = my_state.SERVO
                        # 测试
                        my_beep.test()
                else:
                    # 此时矩形上区域已没有物体，控制小车检查区域内是否还有物体遗漏
                    my_plan.finish_navigate = False
                    my_state.if_move_easy_object = True
                    my_state.state_work = CHECK
                    my_state.state = my_state.NAVIGATE
                    my_order_manager.finish()
        elif my_state.state == my_state.SERVO:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.visual_servo_control()
            else:
                # 若丢失物体则按矩形轨迹行驶寻找物体
                my_plan.navigate([[my_car.x_current-11.0, my_car.y_current], [my_car.x_current-11.0, my_car.y_current+11.0], [my_car.x_current+11.0, my_car.y_current+11.0], [my_car.x_current+11.0, my_car.y_current], [my_car.x_current, my_car.y_current]], my_vision_manager.target_rel_turn_angle)
                target_point = my_art_protocol.coordinate_receive()
                if target_point and target_point[2] == my_vision_manager.current_servo_object and target_point[1] > my_vision_manager.dist_threshold:
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_vision_manager.if_lost_object = False

                # 如果小车在寻找物体过程中完成了一个矩形轨迹但仍未找到物体，则认为该边的区域内没有物体，控制小车再次进行扫描
                if my_plan.finish_navigate == True:
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 重回扫描点继续寻找物体
                    # my_plan.return_to_scan_point = True
                    my_state.state = my_state.NAVIGATE

            if my_vision_manager.finish_servo == True:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(my_vision_manager.current_servo_object, [[my_car.x_current, plan_data.fixed_point[2][1]+10.0], [my_car.x_current, my_car.y_current+8.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    # 重置标志位
                    my_vision_manager.finish_servo = False
                    my_plan.if_send_path = False

                    my_state.state = my_state.ORBIT
                    # 在采集tof数据时固定小车姿态角
                    my_vision_manager.orbit_turn_angle = my_car.now_yaw * 180 / MATH.PI
                    # 将下一次的扫描点置为当前点偏右，控制小车在该区域内寻找物体
                    # plan_data.fixed_point[2][0] = my_car.x_current+10.0
        elif my_state.state == my_state.ORBIT:
            # 延时100ms，等待视觉伺服稳定后再开始环绕
            if counter <= 10:
                counter += 1
            else:
                my_vision_manager.orbit_control(my_vision_manager.orbit_angle)
                if my_vision_manager.finish_orbit == True:
                    order = my_main_protocol.get_slave_state()
                    if order == "finish":
                        counter = 0
                        # 重置从车视觉伺服失败次数
                        my_vision_manager.failed_servo_count = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_state.state = my_state.MOVE
                        # 提前设置小车转向目标角度为当前角度
                        my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
                    elif order == "lost":
                        counter = 0
                        my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                        my_vision_manager.failed_servo_count += 1
                        # my_state.state = my_state.REVERSE_ORBIT
        elif my_state.state == my_state.MOVE:
            # 控制小车夹紧物体，控制主车提前停止
            my_plan.navigate([[my_car.x_current-my_plan.error_x, 260.0]])
            if my_plan.finish_navigate == True:
                counter = 0
                my_plan.finish_navigate = False
                my_vision_manager.car_position = UP_RIGHT
                my_state.state = my_state.CALIBRATE
        elif my_state.state == my_state.CALIBRATE:
            if my_vision_manager.if_lost_object == False:
                my_vision_manager.apriltag_calibrate_control()
            else:
                # 控制小车前后移动寻找apriltag码
                my_plan.navigate([[my_car.x_current+25.0, my_car.y_current], [my_car.x_current+25.0, my_car.y_current+15.0], [my_car.x_current-10.0, my_car.y_current+15.0], [my_car.x_current-10.0, my_car.y_current-15.0], [my_car.x_current, my_car.y_current-15.0]], -90)

                target_point = my_art_protocol.apriltag_receive()
                if target_point:
                    my_plan.reset_navigate()
                    my_vision_manager.counter = 0
                    my_vision_manager.calibrate_times = 0
                    my_vision_manager.if_lost_object, my_vision_manager.if_gain_calibrate_angle = False, False

                # 若找不到apriltag码但完成了一个来回的移动轨迹，则认为该边的区域内没有apriltag码，控制小车回到扫描点进行扫描
                if my_plan.finish_navigate == True:
                    # 重置标志位
                    my_plan.finish_navigate = False
                    my_vision_manager.if_lost_object = False
                    my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    my_state.state = my_state.NAVIGATE
                    # 主车给从车发消息让从车完成矫正
                    my_main_protocol.send_start()
            if my_vision_manager.if_finish_calibrate == True:
                # 主车完成矫正后给从车发消息让从车完成矫正
                my_main_protocol.send_start()
                my_vision_manager.if_finish_calibrate, my_vision_manager.if_gain_calibrate_angle, my_vision_manager.if_ready_calibrate = False, False, False
                if my_state.if_move_easy_object == False:
                    my_state.state = my_state.NAVIGATE
                else:
                    my_state.state_work = CHECK
                    my_state.state = my_state.NAVIGATE
        # 让小车通过反向环绕恢复原位
        """
        elif my_state.state == my_state.REVERSE_ORBIT:
            my_vision_manager.orbit_control(-my_vision_manager.orbit_angle)
            if my_vision_manager.finish_orbit == True:
                my_vision_manager.finish_orbit, my_vision_manager.if_gain_dis = False, False
                # 重回扫描点继续寻找物体
                my_plan.return_to_scan_point = True
                my_state.state = my_state.NAVIGATE
        """    
    elif my_state.state_work == CHECK:
        if my_state.state == my_state.NAVIGATE:
            my_plan.navigate([plan_data.fixed_point[4]], 180.0)
            if my_plan.finish_navigate == True:
                # 提前让从车到目标点等候
                my_main_protocol.send_path(ord('P'), [[137.0, 240.0]])
                my_plan.finish_navigate = False
                my_state.state = my_state.SCAN
                my_order_manager.mode_target()
                my_art_protocol.send_object_kind('C')
        elif my_state.state == my_state.SCAN:
            my_plan.navigate([[110.0, 140.0], [210.0, 140.0]], 180.0)
            if my_plan.finish_navigate == False:
                target_point = my_art_protocol.coordinate_receive()
                if target_point and (target_point[2] == ord('S') or target_point[2] == ord('T') or target_point[2] == ord('B') or target_point[2] == ord('E') or target_point[2] == ord('W')):
                    my_vision_manager.ready_servo_and_orbit(target_point)
                    my_plan.reset_navigate()
                    my_state.state_work = UP
                    my_state.state = my_state.SERVO
            else:
                if my_plan.if_send_path == False:
                    my_main_protocol.send_path(ord('P'), [[15.0, -15.0]])
                    my_plan.if_send_path = True

                if my_main_protocol.get_slave_state() == "get":
                    my_plan.if_send_path = False
                    my_plan.finish_navigate = False
                    # 将openart置为等待模式
                    my_order_manager.finish()
                    # 此时矩形区域内已没有物体，控制小车返回发车区
                    my_state.state_work = RETURN_WORK
                    my_state.state = my_state.RETURN
    elif my_state.state_work == RETURN_WORK:
        if my_state.state == my_state.RETURN:
            # 最终返回主车的起点（避免回程途中与从车碰撞）
            my_plan.navigate([[25.0, -40.0]], 180.0)
            if my_plan.finish_navigate == True:
                my_plan.finish_navigate = False
                my_state.state = my_state.STOP
                my_plan.turn_angle_target = my_car.now_yaw * 180 / MATH.PI
        elif my_state.state == my_state.STOP:
            my_plan.stop()
'''