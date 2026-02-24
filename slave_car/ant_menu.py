import time
import gc

class Menu:
    def __init__(self, flash_sys, beep, key_up, key_down, key_left, key_right, lcd):   
        # 注入 flash 系统对象
        self.flash_sys = flash_sys  
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        # 注入按键对象
        self.key_up = key_up
        self.key_down = key_down
        self.key_left = key_left
        self.key_right = key_right
        # 每个按键上次低电平时间
        self.last_left_time = 0
        self.last_right_time = 0
        self.last_up_time = 0
        self.last_down_time = 0
        # 注入 LCD 对象
        self.lcd = lcd

        ###########################读取所需参数############################
        # PID 参数
        self.angle_normal_kp = self.flash_sys.find_value("angle_normal_kp")          # type: float
        self.angle_normal_ki = self.flash_sys.find_value("angle_normal_ki")          # type: float
        self.angle_normal_kd = self.flash_sys.find_value("angle_normal_kd")          # type: float
        self.integral_limitmax = self.flash_sys.find_value("integral_limitmax")      # type: int
        self.pwmout_limitmax = self.flash_sys.find_value("pwmout_limitmax")          # type: int
        self.angle_integral_limitmax = self.flash_sys.find_value("angle_integral_limitmax")  # type: int
        self.angle_pwmout_limitmax = self.flash_sys.find_value("angle_pwmout_limitmax")      # type: int

        # A/B 设置
        self.A = self.flash_sys.find_value("A")                                      # type: int
        self.B = self.flash_sys.find_value("B")                                      # type: int

        # kp 分段系数
        self.kp_mid = self.flash_sys.find_value("kp_mid")                            # type: float
        self.kp_low = self.flash_sys.find_value("kp_low")                            # type: float

        ###################【机械参数（单位：cm）】###################
        self.wheel_radius = self.flash_sys.find_value("wheel_radius")                # type: float
        self.car_radius = self.flash_sys.find_value("car_radius")                    # type: float

        ###################【系数】####################
        self.speed_conversion_gamma = self.flash_sys.find_value("speed_conversion_gamma")  # type: float
        self.gkd = self.flash_sys.find_value("gkd")                                  # type: float
        self.speed_fuse_ratio = self.flash_sys.find_value("speed_fuse_ratio")        # type: float
        self.gyro_z_supply = self.flash_sys.find_value("gyro_z_supply")              # type: float

        ###################【时间规划】####################
        self.motor_control_T = self.flash_sys.find_value("motor_control_T")          # type: int
        self.collect_dt = self.flash_sys.find_value("collect_dt")                    # type: float
        self.plan_calculate_T = self.flash_sys.find_value("plan_calculate_T")        # type: int
        self.uart_and_menu_T = self.flash_sys.find_value("uart_and_menu_T")          # type: int
        self.boost_time_threshold = self.flash_sys.find_value("boost_time_threshold")  # type: int

        ###################【路径规划】####################
        self.plan_arrive_threshold = self.flash_sys.find_value("plan_arrive_threshold")  # type: float
        self.plan_point_transition_T = self.flash_sys.find_value("plan_point_transition_T")  # type: int
        self.dec_ratio = self.flash_sys.find_value("dec_ratio")                      # type: float

        ###################【速度规划】####################
        self.min_start_v = self.flash_sys.find_value("min_start_v")                  # type: int
        self.long_v_max = self.flash_sys.find_value("long_v_max")                    # type: int
        self.short_v_max = self.flash_sys.find_value("short_v_max")                  # type: int
        self.dead_zone_v = self.flash_sys.find_value("dead_zone_v")                  # type: int

        ###################【视觉伺服】####################
        self.servo_kp_x = self.flash_sys.find_value("servo_kp_x")                    # type: float
        self.servo_kd_x = self.flash_sys.find_value("servo_kd_x")                    # type: float
        self.servo_kp_y = self.flash_sys.find_value("servo_kp_y")                    # type: float
        self.servo_kd_y = self.flash_sys.find_value("servo_kd_y")                    # type: float
        self.servo_target_x = self.flash_sys.find_value("servo_target_x")            # type: int
        self.servo_target_y = self.flash_sys.find_value("servo_target_y")            # type: float
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")              # type: int
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")              # type: int
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")    # type: int
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")    # type: int
        self.servo_pwmout_limitmax = self.flash_sys.find_value("servo_pwmout_limitmax")  # type: int

        ###################【环绕控制】####################
        self.max_orbit_speed = self.flash_sys.find_value("max_orbit_speed")          # type: int
        self.min_orbit_speed = self.flash_sys.find_value("min_orbit_speed")          # type: int

        #### 测试
        self.ur_high_kp = self.flash_sys.find_value("ur_high_kp")  # type: float
        self.ur_high_ki = self.flash_sys.find_value("ur_high_ki")  # type: float
        self.ur_high_kd = self.flash_sys.find_value("ur_high_kd")  # type: float

        ###############################变量定义###########################
        # 当前菜单项
        self.change_page_to = 1  # 将菜单定位到哪一页
        self.Current_line = 1  # 当前行
        self.Start_line, self.End_line = 1, 9 # 显示的起始行，结束行
        # 按键引脚定义
        self.LEFT, self.RIGHT, self.UP, self.DOWN = "left", "right", "up", "down"
        # 步长定义
        self.step_values = [0.1, 1, 5, 10, 100]
        self.current_step_index = 0
        # 行间距
        self.LineSpacing = 18


        ##############################函数定义###############################
        # 保存数据
    def update_config_value(self, file_path, key, new_value):
        lines = []
        found = False

        with open(file_path, 'r') as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('#') or not stripped:
                    lines.append(line) # 保留原样
                    continue
            if '=' in stripped:
                k, v = stripped.split('=', 1)
                k = k.strip()
                if k == key:
                    new_line = f"{k} = {new_value}\n"
                    lines.append(new_line)
                    found = True
                else:
                    lines.append(line)
            else:
                lines.append(line)

        with open(file_path, 'w') as f:
            for line in lines:
                f.write(line)
        # 使用方法
        # self.update_config_value("config.txt", "ul_normal_kp", self.ul_normal_kp)

    # 统一保存数据
    def save_data(self):
        if self.change_page_to == 1:
            # PID 参数
            self.update_config_value("main_config.txt", "angle_normal_kp", self.angle_normal_kp)
            self.update_config_value("main_config.txt", "angle_normal_ki", self.angle_normal_ki)
            self.update_config_value("main_config.txt", "angle_normal_kd", self.angle_normal_kd)
            self.update_config_value("main_config.txt", "integral_limitmax", self.integral_limitmax)
            self.update_config_value("main_config.txt", "pwmout_limitmax", self.pwmout_limitmax)
            self.update_config_value("main_config.txt", "angle_integral_limitmax", self.angle_integral_limitmax)
            self.update_config_value("main_config.txt", "angle_pwmout_limitmax", self.angle_pwmout_limitmax)

            # A/B 设置
            self.update_config_value("main_config.txt", "A", self.A)
            self.update_config_value("main_config.txt", "B", self.B)

            # kp 分段系数
            self.update_config_value("main_config.txt", "kp_mid", self.kp_mid)
            self.update_config_value("main_config.txt", "kp_low", self.kp_low)

        elif self.change_page_to == 2:
            ###################【机械参数（单位：cm）】###################
            self.update_config_value("main_config.txt", "wheel_radius", self.wheel_radius)
            self.update_config_value("main_config.txt", "car_radius", self.car_radius)

        elif self.change_page_to == 3:
            ###################【系数】####################
            self.update_config_value("main_config.txt", "speed_conversion_gamma", self.speed_conversion_gamma)
            self.update_config_value("main_config.txt", "gkd", self.gkd)
            self.update_config_value("main_config.txt", "speed_fuse_ratio", self.speed_fuse_ratio)
            self.update_config_value("main_config.txt", "gyro_z_supply", self.gyro_z_supply)

        elif self.change_page_to == 4:
            ###################【时间规划】####################
            self.update_config_value("main_config.txt", "motor_control_T", self.motor_control_T)
            self.update_config_value("main_config.txt", "collect_dt", self.collect_dt)
            self.update_config_value("main_config.txt", "plan_calculate_T", self.plan_calculate_T)
            self.update_config_value("main_config.txt", "uart_and_menu_T", self.uart_and_menu_T)
            self.update_config_value("main_config.txt", "boost_time_threshold", self.boost_time_threshold)

        elif self.change_page_to == 5:
            ###################【路径规划】####################
            self.update_config_value("main_config.txt", "plan_arrive_threshold", self.plan_arrive_threshold)
            self.update_config_value("main_config.txt", "plan_point_transition_T", self.plan_point_transition_T)
            self.update_config_value("main_config.txt", "dec_ratio", self.dec_ratio)

        elif self.change_page_to == 6:
            ###################【速度规划】####################
            self.update_config_value("main_config.txt", "min_start_v", self.min_start_v)
            self.update_config_value("main_config.txt", "long_v_max", self.long_v_max)
            self.update_config_value("main_config.txt", "short_v_max", self.short_v_max)
            self.update_config_value("main_config.txt", "dead_zone_v", self.dead_zone_v)

        elif self.change_page_to == 7:
            ###################【视觉伺服】####################
            self.update_config_value("main_config.txt", "servo_kp_x", self.servo_kp_x)
            self.update_config_value("main_config.txt", "servo_kd_x", self.servo_kd_x)
            self.update_config_value("main_config.txt", "servo_kp_y", self.servo_kp_y)
            self.update_config_value("main_config.txt", "servo_kd_y", self.servo_kd_y)
            self.update_config_value("main_config.txt", "servo_target_x", self.servo_target_x)
            self.update_config_value("main_config.txt", "servo_target_y", self.servo_target_y)
            self.update_config_value("main_config.txt", "min_rel_speed", self.min_rel_speed)
            self.update_config_value("main_config.txt", "max_rel_speed", self.max_rel_speed)
            self.update_config_value("main_config.txt", "finish_threshold_x", self.finish_threshold_x)
            self.update_config_value("main_config.txt", "finish_threshold_y", self.finish_threshold_y)
            self.update_config_value("main_config.txt", "servo_pwmout_limitmax", self.servo_pwmout_limitmax)

        elif self.change_page_to == 8:
            ###################【环绕控制】####################
            self.update_config_value("main_config.txt", "max_orbit_speed", self.max_orbit_speed)
            self.update_config_value("main_config.txt", "min_orbit_speed", self.min_orbit_speed)

    # 数据统一处理
    def data_processing(self, key):
        ###########################【PID】######################（page 1）
        if self.change_page_to == 1:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.current_step_index = (self.current_step_index - 1) % len(self.step_values)
                elif key == self.RIGHT:
                    self.current_step_index = (self.current_step_index + 1) % len(self.step_values)
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.angle_normal_kp -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.angle_normal_kp += self.step_values[self.current_step_index]
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.angle_normal_ki -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.angle_normal_ki += self.step_values[self.current_step_index]
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.angle_normal_kd -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.angle_normal_kd += self.step_values[self.current_step_index]
            elif self.Current_line == 5:
                if key == self.LEFT:
                    self.integral_limitmax -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.integral_limitmax += self.step_values[self.current_step_index]
            elif self.Current_line == 6:
                if key == self.LEFT:
                    self.pwmout_limitmax -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.pwmout_limitmax += self.step_values[self.current_step_index]
            elif self.Current_line == 7:
                if key == self.LEFT:
                    self.angle_integral_limitmax -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.angle_integral_limitmax += self.step_values[self.current_step_index]
            elif self.Current_line == 8:
                if key == self.LEFT:
                    self.angle_pwmout_limitmax -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.angle_pwmout_limitmax += self.step_values[self.current_step_index]
            elif self.Current_line == 9:
                if key == self.LEFT:
                    self.A -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.A += self.step_values[self.current_step_index]
            elif self.Current_line == 10:
                if key == self.LEFT:
                    self.B -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.B += self.step_values[self.current_step_index]
            elif self.Current_line == 11:
                if key == self.LEFT:
                    self.kp_mid -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.kp_mid += self.step_values[self.current_step_index]
            elif self.Current_line == 12:
                if key == self.LEFT:
                    self.kp_low -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.kp_low += self.step_values[self.current_step_index]
            elif self.Current_line == 13:
                if key == self.RIGHT:
                    self.save_data()
        # ###################【机械参数（单位：cm）】################### (page 2)
        if self.change_page_to == 2:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.wheel_radius -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.wheel_radius += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.car_radius -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.car_radius += self.step_values[self.current_step_index]
            elif self.Current_line == 3:  # save line
                if key == self.RIGHT:
                    self.save_data()

        # ###################【系数】#################### (page 3)
        elif self.change_page_to == 3:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.speed_conversion_gamma -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.speed_conversion_gamma += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.gkd -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.gkd += self.step_values[self.current_step_index]
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.speed_fuse_ratio -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.speed_fuse_ratio += self.step_values[self.current_step_index]
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.gyro_z_supply -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.gyro_z_supply += self.step_values[self.current_step_index]
            elif self.Current_line == 5:  # save line
                if key == self.RIGHT:
                    self.save_data()

        # ###################【时间规划】#################### (page 4)
        elif self.change_page_to == 4:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.motor_control_T -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.motor_control_T += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.collect_dt -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.collect_dt += self.step_values[self.current_step_index]
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.plan_calculate_T -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.plan_calculate_T += self.step_values[self.current_step_index]
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.uart_and_menu_T -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.uart_and_menu_T += self.step_values[self.current_step_index]
            elif self.Current_line == 5:
                if key == self.LEFT:
                    self.boost_time_threshold -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.boost_time_threshold += self.step_values[self.current_step_index]
            elif self.Current_line == 6:  # save line
                if key == self.RIGHT:
                    self.save_data()

        # ###################【路径规划】#################### (page 5)
        elif self.change_page_to == 5:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.plan_arrive_threshold -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.plan_arrive_threshold += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.plan_point_transition_T -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.plan_point_transition_T += self.step_values[self.current_step_index]
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.dec_ratio -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.dec_ratio += self.step_values[self.current_step_index]
            elif self.Current_line == 4:  # save line
                if key == self.RIGHT:
                    self.save_data()

        # ###################【速度规划】#################### (page 6)
        elif self.change_page_to == 6:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.min_start_v -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.min_start_v += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.long_v_max -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.long_v_max += self.step_values[self.current_step_index]
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.short_v_max -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.short_v_max += self.step_values[self.current_step_index]
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.dead_zone_v -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.dead_zone_v += self.step_values[self.current_step_index]
            elif self.Current_line == 5:  # save line
                if key == self.RIGHT:
                    self.save_data()

        # ###################【视觉伺服】#################### (page 7)
        elif self.change_page_to == 7:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.servo_kp_x -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_kp_x += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.servo_kd_x -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_kd_x += self.step_values[self.current_step_index]
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.servo_kp_y -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_kp_y += self.step_values[self.current_step_index]
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.servo_kd_y -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_kd_y += self.step_values[self.current_step_index]
            elif self.Current_line == 5:
                if key == self.LEFT:
                    self.servo_target_x -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_target_x += self.step_values[self.current_step_index]
            elif self.Current_line == 6:
                if key == self.LEFT:
                    self.servo_target_y -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_target_y += self.step_values[self.current_step_index]
            elif self.Current_line == 7:
                if key == self.LEFT:
                    self.min_rel_speed -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.min_rel_speed += self.step_values[self.current_step_index]
            elif self.Current_line == 8:
                if key == self.LEFT:
                    self.max_rel_speed -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.max_rel_speed += self.step_values[self.current_step_index]
            elif self.Current_line == 9:
                if key == self.LEFT:
                    self.finish_threshold_x -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.finish_threshold_x += self.step_values[self.current_step_index]
            elif self.Current_line == 10:
                if key == self.LEFT:
                    self.finish_threshold_y -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.finish_threshold_y += self.step_values[self.current_step_index]
            elif self.Current_line == 11:
                if key == self.LEFT:
                    self.servo_pwmout_limitmax -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.servo_pwmout_limitmax += self.step_values[self.current_step_index]
            elif self.Current_line == 12:  # save line
                if key == self.RIGHT:
                    self.save_data()

        # ###################【环绕控制】#################### (page 8)
        elif self.change_page_to == 8:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.max_orbit_speed -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.max_orbit_speed += self.step_values[self.current_step_index]
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.min_orbit_speed -= self.step_values[self.current_step_index]
                elif key == self.RIGHT:
                    self.min_orbit_speed += self.step_values[self.current_step_index]
            elif self.Current_line == 3:  # save line
                if key == self.RIGHT:
                    self.save_data()


    # 检测按键状态
    # 记得不要写阻塞
    def read_key(self, debounce_ms = 40):
        current_time = time.ticks_ms()
        # 检测是否按下（低电平有效）
        if self.key_left.value() == 0:
            if self.last_left_time == 0:
                self.last_left_time = current_time
            elif time.ticks_diff(current_time, self.last_left_time) >= debounce_ms:
                self.beep.key_test()
                self.last_left_time = 0
                return self.LEFT
            else:
                self.last_left_time = 0

        if self.key_right.value() == 0:
            if self.last_right_time == 0:
                self.last_right_time = current_time
            elif time.ticks_diff(current_time, self.last_right_time) >= debounce_ms:
                self.beep.key_test()
                self.last_right_time = 0
                return self.RIGHT
            else:
                self.last_right_time = 0

        if self.key_up.value() == 0:
            if self.last_up_time == 0:
                self.last_up_time = current_time
            elif time.ticks_diff(current_time, self.last_up_time) >= debounce_ms:
                self.beep.key_test()
                self.last_up_time = 0
                return self.UP
            else:
                self.last_up_time = 0

        if self.key_down.value() == 0:
            if self.last_down_time == 0:
                self.last_down_time = current_time
            elif time.ticks_diff(current_time, self.last_down_time) >= debounce_ms:
                self.beep.key_test()
                self.last_down_time = 0
                return self.DOWN
            else:
                self.last_down_time = 0
    
        return None  # 无按键按下


    # 显示箭头  
    def show_arrow(self):
        for i in range(self.Start_line, self.End_line + 1):
            if i == self.Current_line:
                self.lcd.str16(200, self.LineSpacing * i, "<--", 0xFFFF)
            else:
                self.lcd.str16(200, self.LineSpacing * i, "   ", 0xFFFF)
    # 箭头上移  
    def arrow_up(self, key):
        if key == self.UP:
            if self.Current_line > self.Start_line:
                self.Current_line -= 1
            else:
                self.Current_line = self.End_line
        self.show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

    # 箭头下移
    def arrow_down(self, key):
        if key == self.DOWN:
            if self.Current_line < self.End_line:
                self.Current_line += 1
            else:
                self.Current_line = self.Start_line
        self.show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

    # 箭头的移动,包含上移和下移
    def move_arrow(self, key):
        self.arrow_up(key)
        self.arrow_down(key)

    # 监测指定的跳转页面行是否被按下，并指定目标页面
    def detect_change_page(self, key): ############################################################################################
        if self.Current_line == self.End_line:
            if key == self.LEFT:
                if self.change_page_to == 1:
                    self.change_page_to = 8
                else:
                    self.change_page_to -= 1
            elif key == self.RIGHT:
                if self.change_page_to == 8:
                    self.change_page_to = 1
                else:
                    self.change_page_to += 1
            return True
        else:
            return False

    # 第1页菜单数据显示
    def Menu_Page1_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"n_kp  :{self.angle_normal_kp:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"n_ki  :{self.angle_normal_ki:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 4, f"n_kd  :{self.angle_normal_kd:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, f"int_l :{self.integral_limitmax:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 6, f"pwm_l :{self.pwmout_limitmax:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 7, f"a_int_l:{self.angle_integral_limitmax:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 8, f"a_pwm_l:{self.angle_pwmout_limitmax:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 9, f"A     :{self.A:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 10, f"B     :{self.B:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 11, f"kp_m  :{self.kp_mid:6.2f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 12, f"kp_l  :{self.kp_low:6.2f}    ", 0xFFFF)

    # 第1页菜单显示
    def Menu_Page_1(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 14, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "PID", 0xFFFF)
        self.Menu_Page1_data_show()
        self.lcd.str16(20, self.LineSpacing * 13, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 14, "turn          ", 0xFFFF)

    # 第2页菜单数据显示
    def Menu_Page2_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"wheel_r :{self.wheel_radius:6.2f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"car_r   :{self.car_radius:6.2f}    ", 0xFFFF)

    # 第2页菜单显示
    def Menu_Page_2(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 5, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "MECH", 0xFFFF)
        self.Menu_Page2_data_show()
        self.lcd.str16(20, self.LineSpacing * 4, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, "turn          ", 0xFFFF)
        
    # 第3页菜单数据显示
    def Menu_Page3_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"spd_gam :{self.speed_conversion_gamma:8.4f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"gkd     :{self.gkd:8.4f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 4, f"fuse_rat:{self.speed_fuse_ratio:8.4f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, f"gyro_z  :{self.gyro_z_supply:8.4f}    ", 0xFFFF)

    # 第3页菜单显示
    def Menu_Page_3(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 7, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "COEF", 0xFFFF)
        self.Menu_Page3_data_show()
        self.lcd.str16(20, self.LineSpacing * 6, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 7, "turn          ", 0xFFFF)
        
    # 第4页菜单数据显示
    def Menu_Page4_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"motor_T :{self.motor_control_T:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"collect_dt:{self.collect_dt:7.3f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 4, f"plan_T  :{self.plan_calculate_T:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, f"uart_T  :{self.uart_and_menu_T:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 6, f"boost_T :{self.boost_time_threshold:6d}    ", 0xFFFF)

    # 第4页菜单显示
    def Menu_Page_4(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 8, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "TIME", 0xFFFF)
        self.Menu_Page4_data_show()
        self.lcd.str16(20, self.LineSpacing * 7, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 8, "turn          ", 0xFFFF)

    # 第5页菜单数据显示
    def Menu_Page5_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"arrive_th:{self.plan_arrive_threshold:7.2f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"trans_T :{self.plan_point_transition_T:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 4, f"dec_rat :{self.dec_ratio:7.2f}    ", 0xFFFF)

    # 第5页菜单显示
    def Menu_Page_5(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 6, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "PATH", 0xFFFF)
        self.Menu_Page5_data_show()
        self.lcd.str16(20, self.LineSpacing * 5, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 6, "turn          ", 0xFFFF)

    # 第6页菜单数据显示
    def Menu_Page6_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"min_v   :{self.min_start_v:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"long_v  :{self.long_v_max:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 4, f"short_v :{self.short_v_max:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, f"dead_v  :{self.dead_zone_v:6d}    ", 0xFFFF)

    # 第6页菜单显示
    def Menu_Page_6(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 7, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "SPEED", 0xFFFF)
        self.Menu_Page6_data_show()
        self.lcd.str16(20, self.LineSpacing * 6, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 7, "turn          ", 0xFFFF)

    # 第7页菜单数据显示
    def Menu_Page7_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"kp_x   :{self.servo_kp_x:7.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"kd_x   :{self.servo_kd_x:7.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 4, f"kp_y   :{self.servo_kp_y:7.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, f"kd_y   :{self.servo_kd_y:7.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 6, f"tar_x  :{self.servo_target_x:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 7, f"tar_y  :{self.servo_target_y:7.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 8, f"min_spd:{self.min_rel_speed:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 9, f"max_spd:{self.max_rel_speed:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 10, f"fin_x  :{self.finish_threshold_x:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 11, f"fin_y  :{self.finish_threshold_y:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 12, f"servo_pw:{self.servo_pwmout_limitmax:5d}    ", 0xFFFF)

    # 第7页菜单显示
    def Menu_Page_7(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 14, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "SERVO", 0xFFFF)
        self.Menu_Page7_data_show()
        self.lcd.str16(20, self.LineSpacing * 13, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 14, "turn          ", 0xFFFF)

    # 第8页菜单数据显示
    def Menu_Page8_data_show(self):
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 2, f"max_orb:{self.max_orbit_speed:6d}    ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 3, f"min_orb:{self.min_orbit_speed:6d}    ", 0xFFFF)

    # 第8页菜单显示
    def Menu_Page_8(self):
        gc.collect()
        self.Start_line, self.End_line, self.Current_line = 1, 5, 1
        self.lcd.clear(0x0000)  # 清屏
        self.lcd.str16(100, 0, "ORBIT", 0xFFFF)
        self.Menu_Page8_data_show()
        self.lcd.str16(20, self.LineSpacing * 4, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * 5, "turn          ", 0xFFFF)


    #函数：菜单选择与切换
    def menu_switch(self):
        if(self.change_page_to == 1):
            self.Menu_Page_1()
        elif(self.change_page_to == 2):
            self.Menu_Page_2()
        elif(self.change_page_to == 3):
            self.Menu_Page_3()
        elif(self.change_page_to == 4):
            self.Menu_Page_4()
        elif(self.change_page_to == 5):
            self.Menu_Page_5()
        elif(self.change_page_to == 6):
            self.Menu_Page_6()
        elif(self.change_page_to == 7):
            self.Menu_Page_7()
        elif(self.change_page_to == 8):
            self.Menu_Page_8()
            
"""
# === 主循环 ===
while True:
    last_change_page_to = change_page_to
    key = read_key()
    if key == UP:
        arrow_up()
    elif key == DOWN:
        arrow_down()
    elif key in (LEFT, RIGHT):
        data_processing()

    if detect_change_page():
        if change_page_to != last_change_page_to:
            menu_switch()
            show_arrow()
    
    time.sleep_ms(50)
"""