import time

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
        self.ul_normal_kp = self.flash_sys.find_value("ul_normal_kp")  # type: float
        self.ul_normal_ki = self.flash_sys.find_value("ul_normal_ki")  # type: float
        self.ul_normal_kd = self.flash_sys.find_value("ul_normal_kd")  # type: float
        self.ur_normal_kp = self.flash_sys.find_value("ur_normal_kp")  # type: float
        self.ur_normal_ki = self.flash_sys.find_value("ur_normal_ki")  # type: float
        self.ur_normal_kd = self.flash_sys.find_value("ur_normal_kd")  # type: float


        ###############################变量定义###########################
        # 当前菜单项
        self.change_page_to = 1  # 将菜单定位到哪一页
        self.Current_line = 1  # 当前行
        self.Start_line, self.End_line = 1, 5 # 显示的起始行，结束行
        # 按键引脚定义
        self.LEFT, self.RIGHT, self.UP, self.DOWN = "left", "right", "up", "down"

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
        self.update_config_value("config.txt", "ul_normal_kp", self.ul_normal_kp)
        self.update_config_value("config.txt", "ul_normal_ki", self.ul_normal_ki)
        self.update_config_value("config.txt", "ul_normal_kd", self.ul_normal_kd)
        self.update_config_value("config.txt", "ur_normal_kp", self.ur_normal_kp)
        self.update_config_value("config.txt", "ur_normal_ki", self.ur_normal_ki)
        self.update_config_value("config.txt", "ur_normal_kd", self.ur_normal_kd)

    # 数据统一处理
    def data_processing(self, key):
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



    # 检测按键状态
    # 记得不要写阻塞
    def read_key(self, debounce_ms = 1):
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
    # 判断按键状态，清除状态并且进行箭头的移动

    # 箭头的移动,包含上移和下移
    def move_arrow(self, key):
        self.arrow_up(key)
        self.arrow_down(key)

    # 监测指定的跳转页面行是否被按下，并指定目标页面
    def detect_change_page(self, key):
        if self.Current_line == self.End_line:
            if key == self.LEFT:
                if self.change_page_to == 1:
                    self.change_page_to =2
                else:
                    self.change_page_to -= 1
            elif key == self.RIGHT:
                if self.change_page_to == 2:
                    self.change_page_to = 1
                else:
                    self.change_page_to += 1
            return True
        else:
            return False


    # 第1页菜单数据显示
    def Menu_Page1_data_show(self):
        self.lcd.str16(60, 64, f"l_p:{self.ul_normal_kp:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 1, f"l_i:{self.ul_normal_ki:.2f}", 0xFFFF)
        self.lcd.str16(60, 64 + 32 * 2, f"l_d:{self.ul_normal_kd:.2f}", 0xFFFF)

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


    #函数：菜单选择与切换
    def menu_switch(self):
        if(self.change_page_to == 1):
            self.Menu_Page_1()
        elif(self.change_page_to == 2):
            self.Menu_Page_2()
            
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