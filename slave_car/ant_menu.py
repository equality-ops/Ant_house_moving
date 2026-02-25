import time
import gc
import os

class Menu:
    def __init__(self, flash_sys, beep, key_up, key_down, key_left, key_right, lcd):   
        # 核心优化：所有参数强制转为浮点数，避免类型错误
        self.config = {
            # PID 参数
            "angle_normal_kp": float(flash_sys.find_value("angle_normal_kp")),
            "angle_normal_ki": float(flash_sys.find_value("angle_normal_ki")),
            "angle_normal_kd": float(flash_sys.find_value("angle_normal_kd")),
            "integral_limitmax": float(flash_sys.find_value("integral_limitmax")),
            "pwmout_limitmax": float(flash_sys.find_value("pwmout_limitmax")),
            "angle_integral_limitmax": float(flash_sys.find_value("angle_integral_limitmax")),
            "angle_pwmout_limitmax": float(flash_sys.find_value("angle_pwmout_limitmax")),
            # A/B 设置
            "A": float(flash_sys.find_value("A")),
            "B": float(flash_sys.find_value("B")),
            # kp 分段系数
            "kp_mid": float(flash_sys.find_value("kp_mid")),
            "kp_low": float(flash_sys.find_value("kp_low")),
            # 机械参数
            "wheel_radius": float(flash_sys.find_value("wheel_radius")),
            "car_radius": float(flash_sys.find_value("car_radius")),
            # 系数
            "speed_conversion_gamma": float(flash_sys.find_value("speed_conversion_gamma")),
            "gkd": float(flash_sys.find_value("gkd")),
            "speed_fuse_ratio": float(flash_sys.find_value("speed_fuse_ratio")),
            "gyro_z_supply": float(flash_sys.find_value("gyro_z_supply")),
            # 时间规划
            "motor_control_T": float(flash_sys.find_value("motor_control_T")),
            "collect_dt": float(flash_sys.find_value("collect_dt")),
            "plan_calculate_T": float(flash_sys.find_value("plan_calculate_T")),
            "uart_and_menu_T": float(flash_sys.find_value("uart_and_menu_T")),
            "boost_time_threshold": float(flash_sys.find_value("boost_time_threshold")),
            # 路径规划
            "plan_arrive_threshold": float(flash_sys.find_value("plan_arrive_threshold")),
            "plan_point_transition_T": float(flash_sys.find_value("plan_point_transition_T")),
            "dec_ratio": float(flash_sys.find_value("dec_ratio")),
            # 速度规划
            "min_start_v": float(flash_sys.find_value("min_start_v")),
            "long_v_max": float(flash_sys.find_value("long_v_max")),
            "short_v_max": float(flash_sys.find_value("short_v_max")),
            "dead_zone_v": float(flash_sys.find_value("dead_zone_v")),
            # 视觉伺服
            "servo_kp_x": float(flash_sys.find_value("servo_kp_x")),
            "servo_kd_x": float(flash_sys.find_value("servo_kd_x")),
            "servo_kp_y": float(flash_sys.find_value("servo_kp_y")),
            "servo_kd_y": float(flash_sys.find_value("servo_kd_y")),
            "servo_target_x": float(flash_sys.find_value("servo_target_x")),
            "servo_target_y": float(flash_sys.find_value("servo_target_y")),
            "min_rel_speed": float(flash_sys.find_value("min_rel_speed")),
            "max_rel_speed": float(flash_sys.find_value("max_rel_speed")),
            "finish_threshold_x": float(flash_sys.find_value("finish_threshold_x")),
            "finish_threshold_y": float(flash_sys.find_value("finish_threshold_y")),
            "servo_pwmout_limitmax": float(flash_sys.find_value("servo_pwmout_limitmax")),
            # 环绕控制
            "max_orbit_speed": float(flash_sys.find_value("max_orbit_speed")),
            "min_orbit_speed": float(flash_sys.find_value("min_orbit_speed")),
            # 测试参数
            "ur_high_kp": float(flash_sys.find_value("ur_high_kp")),
            "ur_high_ki": float(flash_sys.find_value("ur_high_ki")),
            "ur_high_kd": float(flash_sys.find_value("ur_high_kd")),
        }

        # 新增：参数名-缩略名映射字典（精准控制每个参数的显示名）
        self.param_short_name = {
            # PID 参数
            "angle_normal_kp": "n_kp",
            "angle_normal_ki": "n_ki",
            "angle_normal_kd": "n_kd",
            "integral_limitmax": "int_l",
            "pwmout_limitmax": "pwm_l",
            "angle_integral_limitmax": "a_int_l",
            "angle_pwmout_limitmax": "a_pwm_l",
            # A/B 设置
            "A": "A",
            "B": "B",
            # kp 分段系数
            "kp_mid": "kp_m",
            "kp_low": "kp_l",
            # 机械参数
            "wheel_radius": "wheel_r",
            "car_radius": "car_r",
            # 系数
            "speed_conversion_gamma": "spd_gam",
            "gkd": "gkd",
            "speed_fuse_ratio": "fuse_rat",
            "gyro_z_supply": "gyro_z",
            # 时间规划
            "motor_control_T": "motor_T",
            "collect_dt": "collect_dt",
            "plan_calculate_T": "plan_T",
            "uart_and_menu_T": "uart_T",
            "boost_time_threshold": "boost_T",
            # 路径规划
            "plan_arrive_threshold": "arrive_th",
            "plan_point_transition_T": "trans_T",
            "dec_ratio": "dec_rat",
            # 速度规划
            "min_start_v": "min_v",
            "long_v_max": "long_v",
            "short_v_max": "short_v",
            "dead_zone_v": "dead_v",
            # 视觉伺服
            "servo_kp_x": "kp_x",
            "servo_kd_x": "kd_x",
            "servo_kp_y": "kp_y",
            "servo_kd_y": "kd_y",
            "servo_target_x": "tar_x",
            "servo_target_y": "tar_y",
            "min_rel_speed": "min_spd",
            "max_rel_speed": "max_spd",
            "finish_threshold_x": "fin_x",
            "finish_threshold_y": "fin_y",
            "servo_pwmout_limitmax": "servo_pw",
            # 环绕控制
            "max_orbit_speed": "max_orb",
            "min_orbit_speed": "min_orb",
            # 测试参数
            "ur_high_kp": "ur_kp",
            "ur_high_ki": "ur_ki",
            "ur_high_kd": "ur_kd",
        }

        # 注入外部对象（弱引用，避免循环引用）
        self.flash_sys = flash_sys
        self.beep = beep
        self.lcd = lcd
        self.keys = {
            "up": key_up, "down": key_down, "left": key_left, "right": key_right
        }

        # 菜单状态（核心状态保留为实例属性）
        self.change_page_to = 1
        self.Current_line = 1  # 初始箭头在第一行
        self.Start_line, self.End_line = 1, 9
        self.KEY_NAMES = {"left": "left", "right": "right", "up": "up", "down": "down"}
        self.step_values = (0.1, 1.0, 5.0, 10.0, 100.0)  # 所有步长转为浮点数
        self.current_step_index = 0
        self.LineSpacing = 18
        
        # 屏幕配置（统一管理，便于修改）
        self.LCD_WIDTH = 240
        self.TITLE_X = 80  # 240宽度屏标题居中x坐标
        self.ARROW_X = 200  # 箭头x坐标
        self.CLEAR_SPACES = " " * 20  # 适配240宽度的清空空格数（足够覆盖）

        # 按键时间戳（初始化为0）
        self.key_timestamps = {
            "left": 0, "right": 0, "up": 0, "down": 0
        }

        # 新增：解决参数修改的核心状态
        self.last_change_page_to = self.change_page_to  # 页面标记（解决NameError）
        self.current_key = None  # 保存当前按键（中断中临时存储）
        self.need_refresh = False  # 标记是否需要刷新LCD（解决修改后不显示）
        
        # 强制一次GC
        gc.collect()

    # 新增：局部刷新参数行（所有参数统一显示为1位小数）
    def refresh_param_line(self, line_num, config_key, fmt):
        """
        仅刷新指定行的参数，避免全屏刷新
        :param line_num: 要刷新的行号
        :param config_key: 参数名
        :param fmt: 格式化字符串（兼容原有格式，但强制转为1位小数显示）
        """
        # 从映射字典获取精准缩略名
        short_name = self.param_short_name.get(config_key, config_key[:6])
        # 只清空当前参数行
        self.lcd.str16(0, self.LineSpacing * line_num, self.CLEAR_SPACES, 0x0000)
        # 重绘最新参数值（强制保留1位小数，解决0.1步长修改报错）
        val = round(self.config[config_key], 1)  # 强制保留1位小数
        display_fmt = "6.1f"  # 统一为1位小数格式，避免类型错误
        self.lcd.str16(20, self.LineSpacing * line_num, f"{short_name} :{val:{display_fmt}}    ", 0xFFFF)
        gc.collect()

    # 新增：第二页分类配置（易维护）
    def _get_page_extra_text(self, page_num):
        """返回指定页面的额外分类文本配置"""
        extra_text = {
            2: [
                (4, "COEF", 0xFFFF),   # 第4行显示COEF
                (9, "ORBIT", 0xFFFF)   # 第9行显示ORBIT
            ]
        }
        return extra_text.get(page_num, [])

    # 新增：绘制页面额外文本
    def _draw_extra_text(self, page_num):
        """绘制页面的额外分类文本"""
        extra_text_list = self._get_page_extra_text(page_num)
        for line_num, text, color in extra_text_list:
            # 先清空该行的分类文本区域，再绘制
            self.lcd.str16(self.TITLE_X, self.LineSpacing * line_num, " " * 10, 0x0000)
            self.lcd.str16(self.TITLE_X, self.LineSpacing * line_num, text, color)
        gc.collect()

    # 显式销毁方法，释放引用
    def destroy(self):
        # 清空大字典
        self.config.clear()
        # 清空参数名映射字典
        self.param_short_name.clear()
        # 解除外部对象引用
        self.flash_sys = None
        self.beep = None
        self.lcd = None
        self.keys.clear()
        # 清空新增状态
        self.last_change_page_to = None
        self.current_key = None
        self.need_refresh = False
        # 强制GC
        gc.collect()

    # 批量更新配置（优化：所有参数保存为1位小数）
    def update_config_values(self, file_path, updates):
        temp_file_path = file_path + ".tmp"
        try:
            with open(file_path, 'r') as f_in, open(temp_file_path, 'w') as f_out:
                for line in f_in:
                    stripped = line.strip()
                    if not stripped or stripped.startswith('#'):
                        f_out.write(line)
                        continue
                    if '=' in stripped:
                        key_part, val_part = stripped.split('=', 1)
                        key = key_part.strip()
                        if key in updates:
                            # 强制保存为1位小数，避免整数/浮点数混合
                            f_out.write(f"{key} = {updates[key]:.1f}\n")
                        else:
                            f_out.write(line)
                    else:
                        f_out.write(line)
            os.remove(file_path)
            os.rename(temp_file_path, file_path)
        except Exception as e:
            try:
                os.remove(temp_file_path)
            except:
                pass
            print(f"Config update error: {e}")
        finally:
            gc.collect()  # 操作文件后立即GC

    # 统一保存数据（优化：所有参数保留1位小数）
    def save_data(self):
        file_path = self.flash_sys.file_path
        updates = {}
        page_configs = {
            1: ["angle_normal_kp", "angle_normal_ki", "angle_normal_kd", "integral_limitmax",
                "pwmout_limitmax", "angle_integral_limitmax", "angle_pwmout_limitmax", "A", "B",
                "kp_mid", "kp_low"],
            2: ["wheel_radius", "car_radius", "speed_conversion_gamma", "gkd",
                "speed_fuse_ratio", "gyro_z_supply", "max_orbit_speed", "min_orbit_speed"],
            3: ["motor_control_T", "collect_dt", "plan_calculate_T", "uart_and_menu_T",
                "boost_time_threshold"],
            4: ["plan_arrive_threshold", "plan_point_transition_T", "dec_ratio"],
            5: ["min_start_v", "long_v_max", "short_v_max", "dead_zone_v"],
            6: ["servo_kp_x", "servo_kd_x", "servo_kp_y", "servo_kd_y", "servo_target_x",
                "servo_target_y", "min_rel_speed", "max_rel_speed", "finish_threshold_x",
                "finish_threshold_y", "servo_pwmout_limitmax"]
        }
        # 批量生成更新字典（所有参数强制保留1位小数）
        if self.change_page_to in page_configs:
            for key in page_configs[self.change_page_to]:
                updates[key] = round(self.config[key], 1)  # 核心：保留1位小数
        if updates:
            self.update_config_values(file_path, updates)
        
        # 保存后箭头重置到第一行
        self.Current_line = 1
        self.need_refresh = True
        gc.collect()

    # 检测按键状态（优化：减少临时变量，及时GC）
    def read_key(self, debounce_ms=40):
        current_time = time.ticks_ms()
        pressed_key = None
        # 遍历按键，减少重复代码
        for key_name, key_obj in self.keys.items():
            if key_obj.value() == 0:
                if self.key_timestamps[key_name] == 0:
                    self.key_timestamps[key_name] = current_time
                elif time.ticks_diff(current_time, self.key_timestamps[key_name]) >= debounce_ms:
                    self.beep.key_test()
                    self.key_timestamps[key_name] = 0
                    pressed_key = self.KEY_NAMES[key_name]
                    break  # 只处理一个按键，减少开销
            else:
                self.key_timestamps[key_name] = 0  # 释放未按下的按键时间戳
        # 及时回收临时内存
        gc.collect()
        return pressed_key

    # 核心修改：所有参数运算后保留1位小数，解决0.1步长修改报错
    def data_processing(self, key):
        step = self.step_values[self.current_step_index]
        modified_key = None  # 标记被修改的参数键
        modified_line = None  # 标记被修改的行号
        modified_fmt = None   # 标记参数格式化字符串
        
        # 步骤1：处理步长切换（仅刷新步长行）
        if self.Current_line == 1:
            old_index = self.current_step_index
            if key == self.KEY_NAMES["left"]:
                self.current_step_index = (self.current_step_index - 1) % len(self.step_values)
            elif key == self.KEY_NAMES["right"]:
                self.current_step_index = (self.current_step_index + 1) % len(self.step_values)
            # 步长变化才刷新步长行
            if old_index != self.current_step_index:
                self.lcd.str16(0, self.LineSpacing * 1, self.CLEAR_SPACES, 0x0000)
                self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
            gc.collect()
            return

        # 步骤2：按页面和行映射配置键（所有整数格式改为6.1f）
        page_line_map = {
            1: {
                2: ("angle_normal_kp", "6.1f"), 3: ("angle_normal_ki", "6.1f"), 4: ("angle_normal_kd", "6.1f"),
                5: ("integral_limitmax", "6.1f"), 6: ("pwmout_limitmax", "6.1f"), 7: ("angle_integral_limitmax", "6.1f"),
                8: ("angle_pwmout_limitmax", "6.1f"), 9: ("A", "6.1f"), 10: ("B", "6.1f"), 11: ("kp_mid", "6.1f"), 12: ("kp_low", "6.1f")
            },
            2: {
                2: ("wheel_radius", "6.1f"), 3: ("car_radius", "6.1f"), 5: ("speed_conversion_gamma", "6.1f"),
                6: ("gkd", "6.1f"), 7: ("speed_fuse_ratio", "6.1f"), 8: ("gyro_z_supply", "6.1f"),
                9: ("", ""),  # 空行
                10: ("max_orbit_speed", "6.1f"), 11: ("min_orbit_speed", "6.1f")
            },
            3: {
                2: ("motor_control_T", "6.1f"), 3: ("collect_dt", "6.1f"), 4: ("plan_calculate_T", "6.1f"),
                5: ("uart_and_menu_T", "6.1f"), 6: ("boost_time_threshold", "6.1f")
            },
            4: {
                2: ("plan_arrive_threshold", "6.1f"), 3: ("plan_point_transition_T", "6.1f"), 4: ("dec_ratio", "6.1f")
            },
            5: {
                2: ("min_start_v", "6.1f"), 3: ("long_v_max", "6.1f"), 4: ("short_v_max", "6.1f"), 5: ("dead_zone_v", "6.1f")
            },
            6: {
                2: ("servo_kp_x", "6.1f"), 3: ("servo_kd_x", "6.1f"), 4: ("servo_kp_y", "6.1f"), 5: ("servo_kd_y", "6.1f"),
                6: ("servo_target_x", "6.1f"), 7: ("servo_target_y", "6.1f"), 8: ("min_rel_speed", "6.1f"), 9: ("max_rel_speed", "6.1f"),
                10: ("finish_threshold_x", "6.1f"), 11: ("finish_threshold_y", "6.1f"), 12: ("servo_pwmout_limitmax", "6.1f")
            }
        }

        # 步骤3：更新配置值（核心：运算后保留1位小数，避免精度错误）
        if self.change_page_to in page_line_map and self.Current_line in page_line_map[self.change_page_to]:
            config_key, fmt = page_line_map[self.change_page_to][self.Current_line]
            # 跳过空参数行
            if config_key == "" or fmt == "":
                gc.collect()
                return
            modified_key = config_key
            modified_line = self.Current_line
            modified_fmt = fmt
            # 核心修改：运算后保留1位小数，解决0.1步长累加报错
            if key == self.KEY_NAMES["left"]:
                new_val = round(self.config[config_key] - step, 1)
            elif key == self.KEY_NAMES["right"]:
                new_val = round(self.config[config_key] + step, 1)
            self.config[config_key] = new_val  # 赋值保留1位小数的结果
            # 仅刷新当前修改的参数行
            self.refresh_param_line(modified_line, modified_key, modified_fmt)

        # 步骤4：处理保存
        save_line_map = {1:13, 2:12, 3:7, 4:5, 5:6, 6:13}
        if self.Current_line == save_line_map.get(self.change_page_to, 0) and key == self.KEY_NAMES["right"]:
            self.save_data()
        
        gc.collect()

    # 刷新当前页面（移除全屏清屏，仅处理箭头和必要刷新）
    def refresh_current_page(self):
        """仅刷新箭头和必要区域，无全屏清屏"""
        if not self.need_refresh:
            return
        # 仅重新绘制箭头（无页面重绘）
        # 先清空所有箭头位置
        for i in range(self.Start_line, self.End_line + 1):
            self.lcd.str16(self.ARROW_X, self.LineSpacing * i, "   ", 0xFFFF)
        # 绘制当前箭头
        self.lcd.str16(self.ARROW_X, self.LineSpacing * self.Current_line, "<--", 0xFFFF)
        self.need_refresh = False
        gc.collect()

    # 箭头控制（仅操作箭头区域，无其他清屏）
    def show_arrow(self):
        """仅清空当前箭头位置并绘制，无其他LCD操作"""
        # 清空上一个箭头位置
        self.lcd.str16(self.ARROW_X, self.LineSpacing * self.Current_line, "   ", 0xFFFF)
        # 绘制新箭头
        self.lcd.str16(self.ARROW_X, self.LineSpacing * self.Current_line, "<--", 0xFFFF)
        gc.collect()

    def arrow_up(self, key):
        if key == self.KEY_NAMES["up"] and self.Current_line > self.Start_line:
            self.Current_line -= 1
        elif key == self.KEY_NAMES["up"]:
            self.Current_line = self.End_line
        self.show_arrow()
        self.need_refresh = True

    def arrow_down(self, key):
        if key == self.KEY_NAMES["down"] and self.Current_line < self.End_line:
            self.Current_line += 1
        elif key == self.KEY_NAMES["down"]:
            self.Current_line = self.Start_line
        self.show_arrow()
        self.need_refresh = True

    def move_arrow(self, key):
        self.arrow_up(key)
        self.arrow_down(key)

    # 页面切换检测 + 翻页后重置箭头到第一行
    def detect_change_page(self, key):
        if self.Current_line == self.End_line:
            if key == self.KEY_NAMES["left"]:
                self.change_page_to = 6 if self.change_page_to == 1 else self.change_page_to - 1
            elif key == self.KEY_NAMES["right"]:
                self.change_page_to = 1 if self.change_page_to == 6 else self.change_page_to + 1
            
            # 翻页后箭头重置到第一行
            self.Current_line = 1
            self.last_change_page_to = self.change_page_to
            # 翻页时才重新渲染整个页面（仅此时清屏）
            self.lcd.clear(0x0000)
            self.menu_switch()
            self.need_refresh = True
            gc.collect()
            return True
        return False

    # 页面显示（优化：所有参数统一显示为1位小数）
    def _show_page(self, title, data_lines, save_line, turn_line, page_num):
        gc.collect()
        # 清空标题行和步长行（仅首次加载页面时）
        self.lcd.str16(0, 0, self.CLEAR_SPACES, 0x0000)
        self.lcd.str16(0, self.LineSpacing * 1, self.CLEAR_SPACES, 0x0000)
        
        # 绘制标题（居中：80开始）
        self.lcd.str16(self.TITLE_X, 0, title, 0xFFFF)
        # 显示步长（强制1位小数）
        self.lcd.str16(20, self.LineSpacing * 1, f"step :{self.step_values[self.current_step_index]:6.1f}    ", 0xFFFF)
        
        # 处理数据行：所有参数统一显示为1位小数
        max_line = max(data_lines.keys()) if data_lines else 1
        for line_num in range(2, max_line + 1):
            if line_num in data_lines:
                config_key, fmt = data_lines[line_num]
                if config_key == "" or fmt == "":
                    continue
                # 从映射字典获取精准缩略名
                short_name = self.param_short_name.get(config_key, config_key[:6])
                # 清空当前参数行并绘制（统一1位小数）
                self.lcd.str16(0, self.LineSpacing * line_num, self.CLEAR_SPACES, 0x0000)
                val = round(self.config[config_key], 1)  # 强制保留1位小数
                self.lcd.str16(20, self.LineSpacing * line_num, f"{short_name} :{val:6.1f}    ", 0xFFFF)
        
        # 处理save/turn行
        for line_num in range(save_line, turn_line + 1):
            self.lcd.str16(0, self.LineSpacing * line_num, self.CLEAR_SPACES, 0x0000)
        self.lcd.str16(20, self.LineSpacing * save_line, "save          ", 0xFFFF)
        self.lcd.str16(20, self.LineSpacing * turn_line, "turn          ", 0xFFFF)
        
        # 绘制页面额外分类文本（如第二页的COEF/ORBIT）
        self._draw_extra_text(page_num)
        gc.collect()

    # 各页面显示（所有参数统一显示为1位小数）
    def Menu_Page_1(self):
        self.Start_line, self.End_line = 1, 14
        data_lines = {
            2: ("angle_normal_kp", "6.1f"), 3: ("angle_normal_ki", "6.1f"),
            4: ("angle_normal_kd", "6.1f"), 5: ("integral_limitmax", "6.1f"),
            6: ("pwmout_limitmax", "6.1f"), 7: ("angle_integral_limitmax", "6.1f"),
            8: ("angle_pwmout_limitmax", "6.1f"), 9: ("A", "6.1f"), 10: ("B", "6.1f"),
            11: ("kp_mid", "6.1f"), 12: ("kp_low", "6.1f")
        }
        self._show_page("PID", data_lines, 13, 14, 1)

    def Menu_Page_2(self):
        self.Start_line, self.End_line = 1, 13
        data_lines = {
            2: ("wheel_radius", "6.1f"), 3: ("car_radius", "6.1f"),
            5: ("speed_conversion_gamma", "6.1f"), 6: ("gkd", "6.1f"),
            7: ("speed_fuse_ratio", "6.1f"), 8: ("gyro_z_supply", "6.1f"),
            10: ("max_orbit_speed", "6.1f"), 11: ("min_orbit_speed", "6.1f")
        }
        self._show_page("MECH", data_lines, 12, 13, 2)

    def Menu_Page_3(self):
        self.Start_line, self.End_line = 1, 8
        data_lines = {
            2: ("motor_control_T", "6.1f"), 3: ("collect_dt", "6.1f"),
            4: ("plan_calculate_T", "6.1f"), 5: ("uart_and_menu_T", "6.1f"),
            6: ("boost_time_threshold", "6.1f")
        }
        self._show_page("TIME", data_lines, 7, 8, 3)

    def Menu_Page_4(self):
        self.Start_line, self.End_line = 1, 6
        data_lines = {
            2: ("plan_arrive_threshold", "6.1f"), 3: ("plan_point_transition_T", "6.1f"),
            4: ("dec_ratio", "6.1f")
        }
        self._show_page("PATH", data_lines, 5, 6, 4)

    def Menu_Page_5(self):
        self.Start_line, self.End_line = 1, 7
        data_lines = {
            2: ("min_start_v", "6.1f"), 3: ("long_v_max", "6.1f"),
            4: ("short_v_max", "6.1f"), 5: ("dead_zone_v", "6.1f")
        }
        self._show_page("SPEED", data_lines, 6, 7, 5)

    def Menu_Page_6(self):
        self.Start_line, self.End_line = 1, 14
        data_lines = {
            2: ("servo_kp_x", "6.1f"), 3: ("servo_kd_x", "6.1f"),
            4: ("servo_kp_y", "6.1f"), 5: ("servo_kd_y", "6.1f"),
            6: ("servo_target_x", "6.1f"), 7: ("servo_target_y", "6.1f"),
            8: ("min_rel_speed", "6.1f"), 9: ("max_rel_speed", "6.1f"),
            10: ("finish_threshold_x", "6.1f"), 11: ("finish_threshold_y", "6.1f"),
            12: ("servo_pwmout_limitmax", "6.1f")
        }
        self._show_page("SERVO", data_lines, 13, 14, 6)
        
    def menu_switch(self):
        page_methods = {
            1: self.Menu_Page_1, 2: self.Menu_Page_2, 3: self.Menu_Page_3,
            4: self.Menu_Page_4, 5: self.Menu_Page_5, 6: self.Menu_Page_6
        }
        if self.change_page_to in page_methods:
            page_methods[self.change_page_to]()
        gc.collect()

    # 适配中断的轻量按键处理入口
    def handle_key_from_interrupt(self, key):
        """
        从中断接收按键，处理逻辑并刷新显示
        :param key: 从read_key获取的按键值
        """
        if not key:
            return
        
        # 处理上下键（仅移动箭头，无清屏）
        if key in (self.KEY_NAMES["up"], self.KEY_NAMES["down"]):
            self.move_arrow(key)
        
        # 处理左右键
        elif key in (self.KEY_NAMES["left"], self.KEY_NAMES["right"]):
            # 检测页面切换（仅翻页时清屏）
            page_changed = self.detect_change_page(key)
            # 非页面切换则修改参数（仅刷新参数行）
            if not page_changed:
                self.data_processing(key)
        
        # 刷新箭头（无其他操作）
        self.refresh_current_page()
