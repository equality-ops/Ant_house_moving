import time

PRIMARY = 0
PID = 1
Mechanical_Parameter = 2
Coefficient = 3
Temporal_Planning = 4
Path_Planning = 5
Velocity_Planning = 6
Visual_Servoing = 7
Orbit_Control = 8



class Menu:
    def __init__(self, flash_sys, beep, key_up, key_down, key_left, key_right, key_confirm, lcd):   
        # 注入 flash 系统对象
        self.flash_sys = flash_sys  
        # 注入蜂鸣器对象，用于警报
        self.beep = beep
        # 注入按键对象
        self.key_up = key_up
        self.key_down = key_down
        self.key_left = key_left
        self.key_right = key_right
        self.key_confirm = key_confirm
        # 每个按键上次低电平时间
        self.last_left_time = 0
        self.last_right_time = 0
        self.last_up_time = 0
        self.last_down_time = 0
        self.last_confirm_time = 0
        # 注入 LCD 对象
        self.lcd = lcd

        ###########################读取所需参数############################
        
        # PID
        self.ul_extreme_kp = self.flash_sys.find_value("ul_extreme_kp")  # type: float
        self.ul_extreme_ki = self.flash_sys.find_value("ul_extreme_ki")  # type: float
        self.ul_extreme_kd = self.flash_sys.find_value("ul_extreme_kd")  # type: float
        self.ur_extreme_kp = self.flash_sys.find_value("ur_extreme_kp")  # type: float
        self.ur_extreme_ki = self.flash_sys.find_value("ur_extreme_ki")  # type: float
        self.ur_extreme_kd = self.flash_sys.find_value("ur_extreme_kd")  # type: float
        self.md_extreme_kp = self.flash_sys.find_value("md_extreme_kp")  # type: float
        self.md_extreme_ki = self.flash_sys.find_value("md_extreme_ki")  # type: float
        self.md_extreme_kd = self.flash_sys.find_value("md_extreme_kd")  # type: float
        self.ul_high_kp = self.flash_sys.find_value("ul_high_kp")  # type: float
        self.ul_high_ki = self.flash_sys.find_value("ul_high_ki")  # type: float
        self.ul_high_kd = self.flash_sys.find_value("ul_high_kd")  # type: float
        self.ur_high_kp = self.flash_sys.find_value("ur_high_kp")  # type: float
        self.ur_high_ki = self.flash_sys.find_value("ur_high_ki")  # type: float
        self.ur_high_kd = self.flash_sys.find_value("ur_high_kd")  # type: float
        self.md_high_kp = self.flash_sys.find_value("md_high_kp")  # type: float
        self.md_high_ki = self.flash_sys.find_value("md_high_ki")  # type: float
        self.md_high_kd = self.flash_sys.find_value("md_high_kd")  # type: float
        self.ul_mid_kp = self.flash_sys.find_value("ul_mid_kp")  # type: float
        self.ul_mid_ki = self.flash_sys.find_value("ul_mid_ki")  # type: float
        self.ul_mid_kd = self.flash_sys.find_value("ul_mid_kd")  # type: float
        self.ur_mid_kp = self.flash_sys.find_value("ur_mid_kp")  # type: float
        self.ur_mid_ki = self.flash_sys.find_value("ur_mid_ki")  # type: float
        self.ur_mid_kd = self.flash_sys.find_value("ur_mid_kd")  # type: float
        self.md_mid_kp = self.flash_sys.find_value("md_mid_kp")  # type: float
        self.md_mid_ki = self.flash_sys.find_value("md_mid_ki")  # type: float
        self.md_mid_kd = self.flash_sys.find_value("md_mid_kd")  # type: float
        self.ul_low_kp = self.flash_sys.find_value("ul_low_kp")  # type: float
        self.ul_low_ki = self.flash_sys.find_value("ul_low_ki")  # type: float
        self.ul_low_kd = self.flash_sys.find_value("ul_low_kd")  # type: float
        self.ur_low_kp = self.flash_sys.find_value("ur_low_kp")  # type: float
        self.ur_low_ki = self.flash_sys.find_value("ur_low_ki")  # type: float
        self.ur_low_kd = self.flash_sys.find_value("ur_low_kd")  # type: float
        self.md_low_kp = self.flash_sys.find_value("md_low_kp")  # type: float
        self.md_low_ki = self.flash_sys.find_value("md_low_ki")  # type: float
        self.md_low_kd = self.flash_sys.find_value("md_low_kd")  # type: float
        self.angle_normal_kp = self.flash_sys.find_value("angle_normal_kp")  # type: float
        self.angle_normal_ki = self.flash_sys.find_value("angle_normal_ki")  # type: float
        self.angle_normal_kd = self.flash_sys.find_value("angle_normal_kd")  # type: float
        self.integral_limitmax = self.flash_sys.find_value("integral_limitmax")  # type: int
        self.pwmout_limitmax = self.flash_sys.find_value("pwmout_limitmax")  # type: int
        self.angle_integral_limitmax = self.flash_sys.find_value("angle_integral_limitmax")  # type: int
        self.angle_pwmout_limitmax = self.flash_sys.find_value("angle_pwmout_limitmax")  # type: int
        self.A = self.flash_sys.find_value("A")  # type: int
        self.B = self.flash_sys.find_value("B")  # type: int
        self.kp_mid = self.flash_sys.find_value("kp_mid")  # type: float
        self.kp_low = self.flash_sys.find_value("kp_low")  # type: float

        # mechanical parameter
        self.wheel_radius = self.flash_sys.find_value("wheel_radius")  # type: float
        self.car_radius = self.flash_sys.find_value("car_radius")  # type: float

        # coefficient
        self.gkd = self.flash_sys.find_value("gkd")  # type: float
        self.speed_fuse_ratio = self.flash_sys.find_value("speed_fuse_ratio")  # type: float
        self.gyro_z_supply = self.flash_sys.find_value("gyro_z_supply")  # type: float

        # temporal planning
        self.motor_control_T = self.flash_sys.find_value("motor_control_T")  # type: int
        self.collect_dt = self.flash_sys.find_value("collect_dt")  # type: float
        self.plan_calculate_T = self.flash_sys.find_value("plan_calculate_T")  # type: int
        self.uart_and_menu_T = self.flash_sys.find_value("uart_and_menu_T")  # type: int
        self.boost_time_threshold = self.flash_sys.find_value("boost_time_threshold")  # type: int

        # path planning
        self.plan_arrive_threshold = self.flash_sys.find_value("plan_arrive_threshold")  # type: float
        self.plan_point_transition_T = self.flash_sys.find_value("plan_point_transition_T")  # type: int
        self.dec_ratio = self.flash_sys.find_value("dec_ratio")  # type: float
        self.error_correct_x_50_1 = self.flash_sys.find_value("error_correct_x_50_1")  # type: float
        self.error_correct_y_50_1 = self.flash_sys.find_value("error_correct_y_50_1")  # type: float
        self.error_correct_x_50_2 = self.flash_sys.find_value("error_correct_x_50_2")  # type: float
        self.error_correct_y_50_2 = self.flash_sys.find_value("error_correct_y_50_2")  # type: float
        self.error_correct_x_50_3 = self.flash_sys.find_value("error_correct_x_50_3")  # type: float
        self.error_correct_y_50_3 = self.flash_sys.find_value("error_correct_y_50_3")  # type: float
        self.error_correct_x_50_4 = self.flash_sys.find_value("error_correct_x_50_4")  # type: float
        self.error_correct_y_50_4 = self.flash_sys.find_value("error_correct_y_50_4")  # type: float
        self.error_correct_x_50_5 = self.flash_sys.find_value("error_correct_x_50_5")  # type: float
        self.error_correct_y_50_5 = self.flash_sys.find_value("error_correct_y_50_5")  # type: float
        self.error_correct_x_50_6 = self.flash_sys.find_value("error_correct_x_50_6")  # type: float
        self.error_correct_y_50_6 = self.flash_sys.find_value("error_correct_y_50_6")  # type: float
        self.error_correct_x_50_7 = self.flash_sys.find_value("error_correct_x_50_7")  # type: float
        self.error_correct_y_50_7 = self.flash_sys.find_value("error_correct_y_50_7")  # type: float
        self.error_correct_x_50_8 = self.flash_sys.find_value("error_correct_x_50_8")  # type: float
        self.error_correct_y_50_8 = self.flash_sys.find_value("error_correct_y_50_8")  # type: float

        # velocity planning
        self.min_start_v = self.flash_sys.find_value("min_start_v")  # type: int
        self.long_v_max = self.flash_sys.find_value("long_v_max")  # type: int
        self.short_v_max = self.flash_sys.find_value("short_v_max")  # type: int
        self.dead_zone_v = self.flash_sys.find_value("dead_zone_v")  # type: int

        # visual servoing
        self.servo_kp_x = self.flash_sys.find_value("servo_kp_x")  # type: float
        self.servo_kd_x = self.flash_sys.find_value("servo_kd_x")  # type: float
        self.servo_kp_y = self.flash_sys.find_value("servo_kp_y")  # type: float
        self.servo_kd_y = self.flash_sys.find_value("servo_kd_y")  # type: float
        self.servo_target_x = self.flash_sys.find_value("servo_target_x")  # type: int
        self.servo_target_y = self.flash_sys.find_value("servo_target_y")  # type: float
        self.min_rel_speed = self.flash_sys.find_value("min_rel_speed")  # type: int
        self.max_rel_speed = self.flash_sys.find_value("max_rel_speed")  # type: int
        self.finish_threshold_x = self.flash_sys.find_value("finish_threshold_x")  # type: int
        self.finish_threshold_y = self.flash_sys.find_value("finish_threshold_y")  # type: int
        self.servo_pwmout_limitmax = self.flash_sys.find_value("servo_pwmout_limitmax")  # type: int

        # orbit control
        self.max_orbit_speed = self.flash_sys.find_value("max_orbit_speed")  # type: int
        self.min_orbit_speed = self.flash_sys.find_value("min_orbit_speed")  # type: int



        ###############################变量定义###########################
        # 当前菜单项
        ### self.change_page_to = PRIMARY  # 将菜单定位到哪一页
        self.current_category = None # 当前所在类别
        self.current_subpage = 1 # 子菜单当前页索引 (0开始)
        self.total_subpages = 1 # 当前类别的总页数
        self.Current_line = 1  # 菜单当前行
        self.Start_line, self.End_line = 1, 8 # 显示的起始行，结束行
        # 按键引脚定义
        self.LEFT, self.RIGHT, self.UP, self.DOWN, self.CONFIRM = "left", "right", "up", "down", "confirm"

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

        if not found:
            lines.append(f"{key} = {new_value}\n")
        with open(file_path, 'w') as f:
            for line in lines:
                f.write(line)
        # 使用方法
        # self.update_config_value("config.txt", "ul_normal_kp", self.ul_normal_kp)

    # 统一保存数据
    def save_data(self):
        # ===== PID 分区（5个子页）=====
        if self.current_category == PID:
            if self.current_subpage == 1:  # 极限参数页（9参数）
                self.update_config_value("main_config.txt", "ul_extreme_kp", self.ul_extreme_kp)
                self.update_config_value("main_config.txt", "ul_extreme_ki", self.ul_extreme_ki)
                self.update_config_value("main_config.txt", "ul_extreme_kd", self.ul_extreme_kd)
                self.update_config_value("main_config.txt", "ur_extreme_kp", self.ur_extreme_kp)
                self.update_config_value("main_config.txt", "ur_extreme_ki", self.ur_extreme_ki)
                self.update_config_value("main_config.txt", "ur_extreme_kd", self.ur_extreme_kd)
                self.update_config_value("main_config.txt", "md_extreme_kp", self.md_extreme_kp)
                self.update_config_value("main_config.txt", "md_extreme_ki", self.md_extreme_ki)
                self.update_config_value("main_config.txt", "md_extreme_kd", self.md_extreme_kd)
            
            elif self.current_subpage == 2:  # 高速参数页（9参数）
                self.update_config_value("main_config.txt", "ul_high_kp", self.ul_high_kp)
                self.update_config_value("main_config.txt", "ul_high_ki", self.ul_high_ki)
                self.update_config_value("main_config.txt", "ul_high_kd", self.ul_high_kd)
                self.update_config_value("main_config.txt", "ur_high_kp", self.ur_high_kp)
                self.update_config_value("main_config.txt", "ur_high_ki", self.ur_high_ki)
                self.update_config_value("main_config.txt", "ur_high_kd", self.ur_high_kd)
                self.update_config_value("main_config.txt", "md_high_kp", self.md_high_kp)
                self.update_config_value("main_config.txt", "md_high_ki", self.md_high_ki)
                self.update_config_value("main_config.txt", "md_high_kd", self.md_high_kd)
            
            elif self.current_subpage == 3:  # 中速参数页（9参数）
                self.update_config_value("main_config.txt", "ul_mid_kp", self.ul_mid_kp)
                self.update_config_value("main_config.txt", "ul_mid_ki", self.ul_mid_ki)
                self.update_config_value("main_config.txt", "ul_mid_kd", self.ul_mid_kd)
                self.update_config_value("main_config.txt", "ur_mid_kp", self.ur_mid_kp)
                self.update_config_value("main_config.txt", "ur_mid_ki", self.ur_mid_ki)
                self.update_config_value("main_config.txt", "ur_mid_kd", self.ur_mid_kd)
                self.update_config_value("main_config.txt", "md_mid_kp", self.md_mid_kp)
                self.update_config_value("main_config.txt", "md_mid_ki", self.md_mid_ki)
                self.update_config_value("main_config.txt", "md_mid_kd", self.md_mid_kd)
            
            elif self.current_subpage == 4:  # 低速参数页（9参数）
                self.update_config_value("main_config.txt", "ul_low_kp", self.ul_low_kp)
                self.update_config_value("main_config.txt", "ul_low_ki", self.ul_low_ki)
                self.update_config_value("main_config.txt", "ul_low_kd", self.ul_low_kd)
                self.update_config_value("main_config.txt", "ur_low_kp", self.ur_low_kp)
                self.update_config_value("main_config.txt", "ur_low_ki", self.ur_low_ki)
                self.update_config_value("main_config.txt", "ur_low_kd", self.ur_low_kd)
                self.update_config_value("main_config.txt", "md_low_kp", self.md_low_kp)
                self.update_config_value("main_config.txt", "md_low_ki", self.md_low_ki)
                self.update_config_value("main_config.txt", "md_low_kd", self.md_low_kd)
            
            elif self.current_subpage == 5:  # 角度/全局参数页（11参数）
                self.update_config_value("main_config.txt", "angle_normal_kp", self.angle_normal_kp)
                self.update_config_value("main_config.txt", "angle_normal_ki", self.angle_normal_ki)
                self.update_config_value("main_config.txt", "angle_normal_kd", self.angle_normal_kd)
                self.update_config_value("main_config.txt", "integral_limitmax", self.integral_limitmax)
                self.update_config_value("main_config.txt", "pwmout_limitmax", self.pwmout_limitmax)
                self.update_config_value("main_config.txt", "angle_integral_limitmax", self.angle_integral_limitmax)
                self.update_config_value("main_config.txt", "angle_pwmout_limitmax", self.angle_pwmout_limitmax)
                self.update_config_value("main_config.txt", "A", self.A)
                self.update_config_value("main_config.txt", "B", self.B)
                self.update_config_value("main_config.txt", "kp_mid", self.kp_mid)
                self.update_config_value("main_config.txt", "kp_low", self.kp_low)
        
        # ===== Path_Planning 分区（2个子页）=====
        elif self.current_category == Path_Planning:
            if self.current_subpage == 1:  # 第1页（9参数）
                self.update_config_value("main_config.txt", "plan_arrive_threshold", self.plan_arrive_threshold)
                self.update_config_value("main_config.txt", "plan_point_transition_T", self.plan_point_transition_T)
                self.update_config_value("main_config.txt", "dec_ratio", self.dec_ratio)
                self.update_config_value("main_config.txt", "error_correct_x_50_1", self.error_correct_x_50_1)
                self.update_config_value("main_config.txt", "error_correct_y_50_1", self.error_correct_y_50_1)
                self.update_config_value("main_config.txt", "error_correct_x_50_2", self.error_correct_x_50_2)
                self.update_config_value("main_config.txt", "error_correct_y_50_2", self.error_correct_y_50_2)
                self.update_config_value("main_config.txt", "error_correct_x_50_3", self.error_correct_x_50_3)
                self.update_config_value("main_config.txt", "error_correct_y_50_3", self.error_correct_y_50_3)
            
            elif self.current_subpage == 2:  # 第2页（10参数）
                self.update_config_value("main_config.txt", "error_correct_x_50_4", self.error_correct_x_50_4)
                self.update_config_value("main_config.txt", "error_correct_y_50_4", self.error_correct_y_50_4)
                self.update_config_value("main_config.txt", "error_correct_x_50_5", self.error_correct_x_50_5)
                self.update_config_value("main_config.txt", "error_correct_y_50_5", self.error_correct_y_50_5)
                self.update_config_value("main_config.txt", "error_correct_x_50_6", self.error_correct_x_50_6)
                self.update_config_value("main_config.txt", "error_correct_y_50_6", self.error_correct_y_50_6)
                self.update_config_value("main_config.txt", "error_correct_x_50_7", self.error_correct_x_50_7)
                self.update_config_value("main_config.txt", "error_correct_y_50_7", self.error_correct_y_50_7)
                self.update_config_value("main_config.txt", "error_correct_x_50_8", self.error_correct_x_50_8)
                self.update_config_value("main_config.txt", "error_correct_y_50_8", self.error_correct_y_50_8)
        
        # ===== 无子页分区：直接保存全部参数 =====
        elif self.current_category == Mechanical_Parameter:  # 2参数
            self.update_config_value("main_config.txt", "wheel_radius", self.wheel_radius)
            self.update_config_value("main_config.txt", "car_radius", self.car_radius)
        
        elif self.current_category == Coefficient:  # 3参数
            self.update_config_value("main_config.txt", "gkd", self.gkd)
            self.update_config_value("main_config.txt", "speed_fuse_ratio", self.speed_fuse_ratio)
            self.update_config_value("main_config.txt", "gyro_z_supply", self.gyro_z_supply)
        
        elif self.current_category == Temporal_Planning:  # 5参数
            self.update_config_value("main_config.txt", "motor_control_T", self.motor_control_T)
            self.update_config_value("main_config.txt", "collect_dt", self.collect_dt)
            self.update_config_value("main_config.txt", "plan_calculate_T", self.plan_calculate_T)
            self.update_config_value("main_config.txt", "uart_and_menu_T", self.uart_and_menu_T)
            self.update_config_value("main_config.txt", "boost_time_threshold", self.boost_time_threshold)
        
        elif self.current_category == Velocity_Planning:  # 4参数
            self.update_config_value("main_config.txt", "min_start_v", self.min_start_v)
            self.update_config_value("main_config.txt", "long_v_max", self.long_v_max)
            self.update_config_value("main_config.txt", "short_v_max", self.short_v_max)
            self.update_config_value("main_config.txt", "dead_zone_v", self.dead_zone_v)
        
        elif self.current_category == Visual_Servoing:  # 11参数
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
        
        elif self.current_category == Orbit_Control:  # 2参数
            self.update_config_value("main_config.txt", "max_orbit_speed", self.max_orbit_speed)
            self.update_config_value("main_config.txt", "min_orbit_speed", self.min_orbit_speed)

    def data_processing(self, key):
        # ===== PID 分区 (5个子页) =====
        if self.current_category == PID:
            if self.current_subpage == 1:  # 极限参数页 (参数1-9, save=10)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.ul_extreme_kp = max(0.0, self.ul_extreme_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ul_extreme_kp += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.ul_extreme_ki = max(0.0, self.ul_extreme_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ul_extreme_ki += 0.001
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.ul_extreme_kd = max(0.0, self.ul_extreme_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ul_extreme_kd += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.ur_extreme_kp = max(0.0, self.ur_extreme_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ur_extreme_kp += 0.1
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.ur_extreme_ki = max(0.0, self.ur_extreme_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ur_extreme_ki += 0.001
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.ur_extreme_kd = max(0.0, self.ur_extreme_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ur_extreme_kd += 0.1
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.md_extreme_kp = max(0.0, self.md_extreme_kp - 0.1)
                    elif key == self.RIGHT:
                        self.md_extreme_kp += 0.1
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.md_extreme_ki = max(0.0, self.md_extreme_ki - 0.001)
                    elif key == self.RIGHT:
                        self.md_extreme_ki += 0.001
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.md_extreme_kd = max(0.0, self.md_extreme_kd - 0.1)
                    elif key == self.RIGHT:
                        self.md_extreme_kd += 0.1
                # SAVE 行处理 (第10行)
                if self.Current_line == 10 and key == self.CONFIRM:
                    self.save_data()
            
            elif self.current_subpage == 2:  # 高速参数页 (参数1-9, save=10)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.ul_high_kp = max(0.0, self.ul_high_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ul_high_kp += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.ul_high_ki = max(0.0, self.ul_high_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ul_high_ki += 0.001
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.ul_high_kd = max(0.0, self.ul_high_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ul_high_kd += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.ur_high_kp = max(0.0, self.ur_high_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ur_high_kp += 0.1
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.ur_high_ki = max(0.0, self.ur_high_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ur_high_ki += 0.001
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.ur_high_kd = max(0.0, self.ur_high_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ur_high_kd += 0.1
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.md_high_kp = max(0.0, self.md_high_kp - 0.1)
                    elif key == self.RIGHT:
                        self.md_high_kp += 0.1
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.md_high_ki = max(0.0, self.md_high_ki - 0.001)
                    elif key == self.RIGHT:
                        self.md_high_ki += 0.001
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.md_high_kd = max(0.0, self.md_high_kd - 0.1)
                    elif key == self.RIGHT:
                        self.md_high_kd += 0.1
                if self.Current_line == 10 and key == self.CONFIRM:
                    self.save_data()
            
            elif self.current_subpage == 3:  # 中速参数页 (参数1-9, save=10)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.ul_mid_kp = max(0.0, self.ul_mid_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ul_mid_kp += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.ul_mid_ki = max(0.0, self.ul_mid_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ul_mid_ki += 0.001
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.ul_mid_kd = max(0.0, self.ul_mid_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ul_mid_kd += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.ur_mid_kp = max(0.0, self.ur_mid_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ur_mid_kp += 0.1
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.ur_mid_ki = max(0.0, self.ur_mid_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ur_mid_ki += 0.001
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.ur_mid_kd = max(0.0, self.ur_mid_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ur_mid_kd += 0.1
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.md_mid_kp = max(0.0, self.md_mid_kp - 0.1)
                    elif key == self.RIGHT:
                        self.md_mid_kp += 0.1
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.md_mid_ki = max(0.0, self.md_mid_ki - 0.001)
                    elif key == self.RIGHT:
                        self.md_mid_ki += 0.001
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.md_mid_kd = max(0.0, self.md_mid_kd - 0.1)
                    elif key == self.RIGHT:
                        self.md_mid_kd += 0.1
                if self.Current_line == 10 and key == self.CONFIRM:
                    self.save_data()
            
            elif self.current_subpage == 4:  # 低速参数页 (参数1-9, save=10)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.ul_low_kp = max(0.0, self.ul_low_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ul_low_kp += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.ul_low_ki = max(0.0, self.ul_low_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ul_low_ki += 0.001
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.ul_low_kd = max(0.0, self.ul_low_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ul_low_kd += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.ur_low_kp = max(0.0, self.ur_low_kp - 0.1)
                    elif key == self.RIGHT:
                        self.ur_low_kp += 0.1
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.ur_low_ki = max(0.0, self.ur_low_ki - 0.001)
                    elif key == self.RIGHT:
                        self.ur_low_ki += 0.001
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.ur_low_kd = max(0.0, self.ur_low_kd - 0.1)
                    elif key == self.RIGHT:
                        self.ur_low_kd += 0.1
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.md_low_kp = max(0.0, self.md_low_kp - 0.1)
                    elif key == self.RIGHT:
                        self.md_low_kp += 0.1
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.md_low_ki = max(0.0, self.md_low_ki - 0.001)
                    elif key == self.RIGHT:
                        self.md_low_ki += 0.001
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.md_low_kd = max(0.0, self.md_low_kd - 0.1)
                    elif key == self.RIGHT:
                        self.md_low_kd += 0.1
                if self.Current_line == 10 and key == self.CONFIRM:
                    self.save_data()
            
            elif self.current_subpage == 5:  # 角度/全局参数页 (参数1-11, save=12)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.angle_normal_kp = max(0.0, self.angle_normal_kp - 0.1)
                    elif key == self.RIGHT:
                        self.angle_normal_kp += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.angle_normal_ki = max(0.0, self.angle_normal_ki - 0.01)
                    elif key == self.RIGHT:
                        self.angle_normal_ki += 0.01
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.angle_normal_kd = max(0.0, self.angle_normal_kd - 0.1)
                    elif key == self.RIGHT:
                        self.angle_normal_kd += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.integral_limitmax = max(0, self.integral_limitmax - 100)
                    elif key == self.RIGHT:
                        self.integral_limitmax += 100
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.pwmout_limitmax = max(0, self.pwmout_limitmax - 100)
                    elif key == self.RIGHT:
                        self.pwmout_limitmax += 100
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.angle_integral_limitmax = max(0, self.angle_integral_limitmax - 10)
                    elif key == self.RIGHT:
                        self.angle_integral_limitmax += 10
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.angle_pwmout_limitmax = max(0, self.angle_pwmout_limitmax - 10)
                    elif key == self.RIGHT:
                        self.angle_pwmout_limitmax += 10
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.A = max(0, self.A - 5)
                    elif key == self.RIGHT:
                        self.A += 5
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.B = max(0, self.B - 5)
                    elif key == self.RIGHT:
                        self.B += 5
                elif self.Current_line == 10:
                    if key == self.LEFT:
                        self.kp_mid = max(0, self.kp_mid - 5)
                    elif key == self.RIGHT:
                        self.kp_mid += 5
                elif self.Current_line == 11:
                    if key == self.LEFT:
                        self.kp_low = max(0, self.kp_low - 5)
                    elif key == self.RIGHT:
                        self.kp_low += 5
                if self.Current_line == 12 and key == self.CONFIRM:
                    self.save_data()
        
        # ===== Path_Planning 分区 (2个子页) =====
        elif self.current_category == Path_Planning:
            if self.current_subpage == 1:  # 第1页 (参数1-9, save=10)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.plan_arrive_threshold = max(0.0, self.plan_arrive_threshold - 0.1)
                    elif key == self.RIGHT:
                        self.plan_arrive_threshold += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.plan_point_transition_T = max(1, self.plan_point_transition_T - 1)
                    elif key == self.RIGHT:
                        self.plan_point_transition_T += 1
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.dec_ratio = max(0.0, self.dec_ratio - 0.1)
                    elif key == self.RIGHT:
                        self.dec_ratio += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.error_correct_x_50_1 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_1 += 0.1
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.error_correct_y_50_1 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_1 += 0.1
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.error_correct_x_50_2 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_2 += 0.1
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.error_correct_y_50_2 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_2 += 0.1
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.error_correct_x_50_3 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_3 += 0.1
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.error_correct_y_50_3 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_3 += 0.1
                if self.Current_line == 10 and key == self.CONFIRM:
                    self.save_data()
            
            elif self.current_subpage == 2:  # 第2页 (参数1-10, save=11)
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.error_correct_x_50_4 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_4 += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.error_correct_y_50_4 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_4 += 0.1
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.error_correct_x_50_5 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_5 += 0.1
                elif self.Current_line == 4:
                    if key == self.LEFT:
                        self.error_correct_y_50_5 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_5 += 0.1
                elif self.Current_line == 5:
                    if key == self.LEFT:
                        self.error_correct_x_50_6 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_6 += 0.1
                elif self.Current_line == 6:
                    if key == self.LEFT:
                        self.error_correct_y_50_6 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_6 += 0.1
                elif self.Current_line == 7:
                    if key == self.LEFT:
                        self.error_correct_x_50_7 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_7 += 0.1
                elif self.Current_line == 8:
                    if key == self.LEFT:
                        self.error_correct_y_50_7 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_7 += 0.1
                elif self.Current_line == 9:
                    if key == self.LEFT:
                        self.error_correct_x_50_8 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_x_50_8 += 0.1
                elif self.Current_line == 10:
                    if key == self.LEFT:
                        self.error_correct_y_50_8 -= 0.1
                    elif key == self.RIGHT:
                        self.error_correct_y_50_8 += 0.1
                if self.Current_line == 11 and key == self.CONFIRM:
                    self.save_data()
        
        # ===== 单页分区 (save行 = 参数行数+1) =====
        elif self.current_category == Mechanical_Parameter:  # 2参数, save=3
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.wheel_radius = max(0.01, self.wheel_radius - 0.1)  # 防止为0
                elif key == self.RIGHT:
                    self.wheel_radius += 0.1
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.car_radius = max(0.01, self.car_radius - 0.1)
                elif key == self.RIGHT:
                    self.car_radius += 0.1
            if self.Current_line == 3 and key == self.CONFIRM:
                self.save_data()
        
        elif self.current_category == Coefficient:  # 3参数, save=4
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.gkd = min(0.0, self.gkd - 0.01)  # 保持负值特性
                elif key == self.RIGHT:
                    self.gkd = min(0.0, self.gkd + 0.01)
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.speed_fuse_ratio = max(0.0, min(1.0, self.speed_fuse_ratio - 0.01))
                elif key == self.RIGHT:
                    self.speed_fuse_ratio = max(0.0, min(1.0, self.speed_fuse_ratio + 0.01))
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.gyro_z_supply = max(0.0, self.gyro_z_supply - 0.01)
                elif key == self.RIGHT:
                    self.gyro_z_supply += 0.01
            if self.Current_line == 4 and key == self.CONFIRM:
                self.save_data()
        
        elif self.current_category == Temporal_Planning:  # 5参数, save=6
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.motor_control_T = max(1, self.motor_control_T - 1)
                elif key == self.RIGHT:
                    self.motor_control_T += 1
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.collect_dt = max(0.001, self.collect_dt - 0.001)
                elif key == self.RIGHT:
                    self.collect_dt += 0.001
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.plan_calculate_T = max(1, self.plan_calculate_T - 1)
                elif key == self.RIGHT:
                    self.plan_calculate_T += 1
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.uart_and_menu_T = max(1, self.uart_and_menu_T - 1)
                elif key == self.RIGHT:
                    self.uart_and_menu_T += 1
            elif self.Current_line == 5:
                if key == self.LEFT:
                    self.boost_time_threshold = max(0, self.boost_time_threshold - 1)
                elif key == self.RIGHT:
                    self.boost_time_threshold += 1
            if self.Current_line == 6 and key == self.CONFIRM:
                self.save_data()
        
        elif self.current_category == Velocity_Planning:  # 4参数, save=5
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.min_start_v = max(0, self.min_start_v - 1)
                elif key == self.RIGHT:
                    self.min_start_v += 1
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.long_v_max = max(0, self.long_v_max - 10)
                elif key == self.RIGHT:
                    self.long_v_max += 10
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.short_v_max = max(0, self.short_v_max - 10)
                elif key == self.RIGHT:
                    self.short_v_max += 10
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.dead_zone_v = max(0, self.dead_zone_v - 1)
                elif key == self.RIGHT:
                    self.dead_zone_v += 1
            if self.Current_line == 5 and key == self.CONFIRM:
                self.save_data()
        
        elif self.current_category == Visual_Servoing:  # 11参数, save=12
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.servo_kp_x = max(0.0, self.servo_kp_x - 0.1)
                elif key == self.RIGHT:
                    self.servo_kp_x += 0.1
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.servo_kd_x = max(0.0, self.servo_kd_x - 0.1)
                elif key == self.RIGHT:
                    self.servo_kd_x += 0.1
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.servo_kp_y = max(0.0, self.servo_kp_y - 0.1)
                elif key == self.RIGHT:
                    self.servo_kp_y += 0.1
            elif self.Current_line == 4:
                if key == self.LEFT:
                    self.servo_kd_y = max(0.0, self.servo_kd_y - 0.1)
                elif key == self.RIGHT:
                    self.servo_kd_y += 0.1
            elif self.Current_line == 5:
                if key == self.LEFT:
                    self.servo_target_x = max(0, self.servo_target_x - 1)
                elif key == self.RIGHT:
                    self.servo_target_x += 1
            elif self.Current_line == 6:
                if key == self.LEFT:
                    self.servo_target_y = max(0.0, self.servo_target_y - 0.1)
                elif key == self.RIGHT:
                    self.servo_target_y += 0.1
            elif self.Current_line == 7:
                if key == self.LEFT:
                    self.min_rel_speed = max(0, self.min_rel_speed - 10)
                elif key == self.RIGHT:
                    self.min_rel_speed += 10
            elif self.Current_line == 8:
                if key == self.LEFT:
                    self.max_rel_speed = max(0, self.max_rel_speed - 10)
                elif key == self.RIGHT:
                    self.max_rel_speed += 10
            elif self.Current_line == 9:
                if key == self.LEFT:
                    self.finish_threshold_x = max(0, self.finish_threshold_x - 1)
                elif key == self.RIGHT:
                    self.finish_threshold_x += 1
            elif self.Current_line == 10:
                if key == self.LEFT:
                    self.finish_threshold_y = max(0, self.finish_threshold_y - 1)
                elif key == self.RIGHT:
                    self.finish_threshold_y += 1
            elif self.Current_line == 11:
                if key == self.LEFT:
                    self.servo_pwmout_limitmax = max(0, self.servo_pwmout_limitmax - 1)
                elif key == self.RIGHT:
                    self.servo_pwmout_limitmax += 1
            if self.Current_line == 12 and key == self.CONFIRM:
                self.save_data()
        
        elif self.current_category == Orbit_Control:  # 2参数, save=3
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.max_orbit_speed = max(0, self.max_orbit_speed - 10)
                elif key == self.RIGHT:
                    self.max_orbit_speed += 10
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.min_orbit_speed = max(0, self.min_orbit_speed - 10)
                elif key == self.RIGHT:
                    self.min_orbit_speed += 10
            if self.Current_line == 3 and key == self.CONFIRM:
                self.save_data()          
        """
        if self.change_page_to == 1:
            if self.Current_line == 1:
                if key == self.LEFT:
                    self.ul_normal_kp -= 0.1
                elif key == self.RIGHT:
                    self.ul_normal_kp += 0.1
            elif self.Current_line == 2:
                if key == self.LEFT:
                    self.ul_normal_ki -= 0.1
                elif key == self.RIGHT:
                    self.ul_normal_ki += 0.1
            elif self.Current_line == 3:
                if key == self.LEFT:
                    self.ul_normal_kd -= 0.1
                elif key == self.RIGHT:
                    self.ul_normal_kd += 0.1
            elif self.Current_line == 4:
                if key == self.RIGHT:
                    self.save_data()
            elif self.change_page_to == 2:
                if self.Current_line == 1:
                    if key == self.LEFT:
                        self.ur_normal_kp -= 0.1
                    elif key == self.RIGHT:
                        self.ur_normal_kp += 0.1
                elif self.Current_line == 2:
                    if key == self.LEFT:
                        self.ur_normal_ki -= 0.1
                    elif key == self.RIGHT:
                        self.ur_normal_ki += 0.1
                elif self.Current_line == 3:
                    if key == self.LEFT:
                        self.ur_normal_kd -= 0.1
                    elif key == self.RIGHT:
                        self.ur_normal_kd += 0.1
                elif self.Current_line == 4:
                    if key == self.RIGHT:
                        self.save_data()
            """



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
        
        if self.key_confirm.value() == 0:
            if self.last_confirm_time == 0:
                self.last_confirm_time = current_time
            elif time.ticks_diff(current_time, self.last_down_time) >= debounce_ms:
                self.beep.key_test()
                self.last_confirm_time = 0
                return self.CONFIRM
            else:
                self.last_confirm_time = 0
    
        return None  # 无按键按下


    # 显示箭头  
    def show_arrow(self):
        for i in range(self.Start_line, self.End_line + 1):
            if i == self.Current_line:
                self.lcd.str16(150, 64 + 32 * (i - 1), "<--", 0xFFFF)
            else:
                self.lcd.str16(150, 64 + 32 * (i - 1), "   ", 0xFFFF)
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

    # 监测指定的跳转页面行是否被按下，并指定目标页面
    def detect_change_page(self, key):
        if self.Current_line == self.End_line:
            if key == self.CONFIRM:
                self.current_category = None
                self.menu_switch()
            return True
        if self.current_category == PID:
            if self.Current_line == self.End_line - 1:
                if key == self.LEFT:
                    if self.current_subpage == 1:
                        self.current_subpage = 5
                    else:
                        self.current_subpage -= 1
                    return True
                elif key == self.RIGHT:
                    if self.current_subpage == 5:
                        self.current_subpage = 1
                    else:
                        self.current_subpage += 1
                    return True
                return False
        elif self.current_category == Path_Planning:
            if self.Current_line == self.End_line - 1:
                if key == self.LEFT:
                    if self.current_subpage == 1:
                        self.current_subpage = 2
                    else:
                        self.current_subpage -= 1
                    return True
                elif key == self.RIGHT:
                    if self.current_subpage == 2:
                        self.current_subpage = 1
                    else:
                        self.current_subpage += 1
                    return True
                return False
        """
        if self.Current_line == self.End_line:
            if key == self.LEFT:
                if self.current_subpage == 1:
                    self.current_subpage =2
                else:
                    self.current_subpage -= 1
            elif key == self.RIGHT:
                if self.current_subpage == 2:
                    self.current_subpage = 1
                else:
                    self.current_subpage += 1
            return True
        else:
            return False
        """
    """
    # 第1页菜单数据显示
    def Menu_Page1_data_show(self):
        self.lcd.str16(60, 64, f"l_p:{self.ul_high_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"l_i:{self.ul_high_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"l_d:{self.ul_high_kd:.2f}", 0xFFFF)

    # 第1页菜单显示
    def Menu_Page_1(self):
        self.Start_line,self.End_line,self.Current_line=1,5,1
        self.lcd.clear(0x0000)
        self.Menu_Page1_data_show()
        self.lcd.str16(60, 64 + 32 * 3, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, "turn", 0xFFFF)

    # 第2页菜单数据显示
    def Menu_Page2_data_show(self):
        self.lcd.str16(60, 64, f"r_p:{self.ur_normal_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"r_i:{self.ur_normal_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"r_d:{self.ur_normal_kd:.2f}", 0xFFFF)
    
    # 第2页菜单显示
    def Menu_Page_2(self):
        self.Start_line,self.End_line,self.Current_line=1,5,1
        self.lcd.clear(0x0000)
        self.Menu_Page2_data_show()
        self.lcd.str16(60, 64 + 32 * 3, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, "turn", 0xFFFF)
    """

    # 主菜单显示
    def show_primary_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,8,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64 + 32 * 3, "PID", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, "Mechanical Parameter", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, "Coefficient", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, "Temporal Planning", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, "Path Planning", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, "Velocity Planning", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, "Visual Servoing", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "Orbit Control", 0xFFFF)

    def show_PID_menu_1(self):
        self.Start_line,self.End_line,self.Current_line=1,12,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"ul_ext_kp:{self.ul_extreme_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"ul_ext_ki:{self.ul_extreme_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"ul_ext_kd:{self.ul_extreme_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"ur_ext_kp:{self.ur_extreme_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"ur_ext_ki:{self.ur_extreme_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"ur_ext_kd:{self.ur_extreme_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"md_ext_kp:{self.md_extreme_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"md_ext_ki:{self.md_extreme_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"md_ext_kd:{self.md_extreme_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "return", 0xFFFF)

    def show_PID_menu_2(self):
        self.Start_line,self.End_line,self.Current_line=1,12,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"ul_high_kp:{self.ul_high_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"ul_high_ki:{self.ul_high_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"ul_high_kd:{self.ul_high_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"ur_high_kp:{self.ur_high_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"ur_high_ki:{self.ur_high_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"ur_high_kd:{self.ur_high_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"md_high_kp:{self.md_high_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"md_high_ki:{self.md_high_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"md_high_kd:{self.md_high_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "return", 0xFFFF)

    def show_PID_menu_3(self):
        self.Start_line,self.End_line,self.Current_line=1,12,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"ul_mid_kp:{self.ul_mid_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"ul_mid_ki:{self.ul_mid_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"ul_mid_kd:{self.ul_mid_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"ur_mid_kp:{self.ur_mid_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"ur_mid_ki:{self.ur_mid_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"ur_mid_kd:{self.ur_mid_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"md_mid_kp:{self.md_mid_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"md_mid_ki:{self.md_mid_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"md_mid_kd:{self.md_mid_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "return", 0xFFFF)

    def show_PID_menu_4(self):
        self.Start_line,self.End_line,self.Current_line=1,12,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"ul_low_kp:{self.ul_low_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"ul_low_ki:{self.ul_low_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"ul_low_kd:{self.ul_low_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"ur_low_kp:{self.ur_low_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"ur_low_ki:{self.ur_low_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"ur_low_kd:{self.ur_low_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"md_low_kp:{self.md_low_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"md_low_ki:{self.md_low_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"md_low_kd:{self.md_low_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "return", 0xFFFF)

    def show_PID_menu_5(self):
        self.Start_line,self.End_line,self.Current_line=1,14,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"ang_nor_kp:{self.angle_normal_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"ang_nor_ki:{self.angle_normal_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"ang_nor_kd:{self.angle_normal_kd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"int_lmt_max:{self.integral_limitmax:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"pwm_lmt_max:{self.pwmout_limitmax:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"ang_int_lmt_max:{self.angle_integral_limitmax:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"ang_pwm_lmt_max:{self.angle_pwmout_limitmax:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"A:{self.A:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"B:{self.B:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, f"kp_mid:{self.kp_mid:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, f"kp_low:{self.kp_low:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 12, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 13, "return", 0xFFFF)

    def show_Path_Planning_menu_1(self):
        self.Start_line,self.End_line,self.Current_line=1,12,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"arr_thres:{self.plan_arrive_threshold:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"pt_trns_T:{self.plan_point_transition_T:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"dec_ratio:{self.dec_ratio:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"err_x_50_1:{self.error_correct_x_50_1:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"err_y_50_1:{self.error_correct_y_50_1:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"err_x_50_2:{self.error_correct_x_50_2:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"err_y_50_2:{self.error_correct_y_50_2:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"err_x_50_3:{self.error_correct_x_50_3:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"err_y_50_3:{self.error_correct_y_50_3:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "return", 0xFFFF)

    def show_Path_Planning_menu_2(self):
        self.Start_line,self.End_line,self.Current_line=1,13,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"err_x_50_4:{self.error_correct_x_50_4:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"err_y_50_4:{self.error_correct_y_50_4:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"err_x_50_5:{self.error_correct_x_50_5:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"err_y_50_5:{self.error_correct_y_50_5:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"err_x_50_6:{self.error_correct_x_50_6:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"err_y_50_6:{self.error_correct_y_50_6:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"err_x_50_7:{self.error_correct_x_50_7:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"err_y_50_7:{self.error_correct_y_50_7:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"err_x_50_8:{self.error_correct_x_50_8:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, f"err_y_50_8:{self.error_correct_y_50_8:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "turn", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 12, "return", 0xFFFF)

    def show_Mechanical_Parameter_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,4,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"wheel_r:{self.wheel_radius:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"car_r:{self.car_radius:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, "return", 0xFFFF)

    def show_Coefficient_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,5,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"gkd:{self.gkd:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"spd_fuse_ratio:{self.speed_fuse_ratio:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"gyro_z_sup:{self.gyro_z_supply:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, "return", 0xFFFF)

    def show_Temporal_Planning_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,7,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"mtr_ctrl_T:{self.motor_control_T:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"coll_dt:{self.collect_dt:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"pln_calc_T:{self.plan_calculate_T:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"uart_mnu_T:{self.uart_and_menu_T:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"bst_tshld:{self.boost_time_threshold:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, "return", 0xFFFF)

    def show_Velocity_Planning_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,6,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"min_strt_v:{self.min_start_v:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"lng_v_max:{self.long_v_max:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"shrt_v_max:{self.short_v_max:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"dz_v:{self.dead_zone_v:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, "return", 0xFFFF)

    def show_Visual_Servoing_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,13,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"sv_kp_x:{self.servo_kp_x:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"sv_kd_x:{self.servo_kd_x:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"sv_kp_y:{self.servo_kp_y:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, f"sv_kd_y:{self.servo_kd_y:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 4, f"sv_tgt_x:{self.servo_target_x:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 5, f"sv_tgt_y:{self.servo_target_y:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 6, f"min_rel_spd:{self.min_rel_speed:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 7, f"max_rel_spd:{self.max_rel_speed:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 8, f"fin_thr_x:{self.finish_threshold_x:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 9, f"fin_thr_y:{self.finish_threshold_y:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 10, f"sv_pwm_lim:{self.servo_pwmout_limitmax:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 11, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 12, "return", 0xFFFF)

    def show_Orbit_Control_menu(self):
        self.Start_line,self.End_line,self.Current_line=1,4,1
        self.lcd.clear(0x0000)
        self.lcd.str16(60, 64, f"mx_orb_spd:{self.max_orbit_speed:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"mn_orb_spd:{self.min_orbit_speed:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, "save", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 3, "return", 0xFFFF)

    #函数：菜单选择与切换
    def menu_switch(self):
        if self.current_category == None:
            self.show_primary_menu()  # 主菜单
        
        # ===== 多子页分区：需判断 current_subpage =====
        elif self.current_category == PID:
            if self.current_subpage == 1:
                self.show_PID_menu_1()
            elif self.current_subpage == 2:
                self.show_PID_menu_2()
            elif self.current_subpage == 3:
                self.show_PID_menu_3()
            elif self.current_subpage == 4:
                self.show_PID_menu_4()
            elif self.current_subpage == 5:
                self.show_PID_menu_5()
        
        elif self.current_category == Path_Planning:
            if self.current_subpage == 1:
                self.show_Path_Planning_menu_1()
            elif self.current_subpage == 2:
                self.show_Path_Planning_menu_2()
        
        # ===== 单页分区：直接调用对应显示函数 =====
        elif self.current_category == Mechanical_Parameter:
            self.show_Mechanical_Parameter_menu()
        elif self.current_category == Coefficient:
            self.show_Coefficient_menu()
        elif self.current_category == Temporal_Planning:
            self.show_Temporal_Planning_menu()
        elif self.current_category == Velocity_Planning:
            self.show_Velocity_Planning_menu()
        elif self.current_category == Visual_Servoing:
            self.show_Visual_Servoing_menu()
        elif self.current_category == Orbit_Control:
            self.show_Orbit_Control_menu()
        
        # 异常状态返回主菜单
        else:
            self.current_category = None
            self.show_primary_menu()

            
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