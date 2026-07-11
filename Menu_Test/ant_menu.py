from machine import *
import gc
import time
import os

class Menu:
    def __init__(self, flash_sys, beep, ips200pro):
        # 所有参数强制转为浮点数，避免类型错误
        self.config = {
            # PID 参数
            "angle_normal_kp": float(flash_sys.find_value("angle_normal_kp")) if flash_sys.find_value("angle_normal_kp") else 10.0,
            "angle_normal_ki": float(flash_sys.find_value("angle_normal_ki")) if flash_sys.find_value("angle_normal_ki") else 0.0,
            "angle_normal_kd": float(flash_sys.find_value("angle_normal_kd")) if flash_sys.find_value("angle_normal_kd") else 20.0,
            "integral_limitmax": float(flash_sys.find_value("integral_limitmax")) if flash_sys.find_value("integral_limitmax") else 14000.0,
            "pwmout_limitmax": float(flash_sys.find_value("pwmout_limitmax")) if flash_sys.find_value("pwmout_limitmax") else 8000.0,
            "high_angle_pwmout_limitmax": float(flash_sys.find_value("high_angle_pwmout_limitmax")) if flash_sys.find_value("high_angle_pwmout_limitmax") else 600.0,
            "low_angle_pwmout_limitmax": float(flash_sys.find_value("low_angle_pwmout_limitmax")) if flash_sys.find_value("low_angle_pwmout_limitmax") else 200.0,
            # A/B 设置
            "A": float(flash_sys.find_value("A")) if flash_sys.find_value("A") else 500.0,
            "B": float(flash_sys.find_value("B")) if flash_sys.find_value("B") else 200.0,
            # 系数
            "gkd": float(flash_sys.find_value("gkd")) if flash_sys.find_value("gkd") else -0.19,
            "speed_fuse_ratio": float(flash_sys.find_value("speed_fuse_ratio")) if flash_sys.find_value("speed_fuse_ratio") else 0.2,
            # 时间规划
            "motor_control_T": float(flash_sys.find_value("motor_control_T")) if flash_sys.find_value("motor_control_T") else 2.0,
            "collect_dt": float(flash_sys.find_value("collect_dt")) if flash_sys.find_value("collect_dt") else 0.002,
            "plan_calculate_T": float(flash_sys.find_value("plan_calculate_T")) if flash_sys.find_value("plan_calculate_T") else 10.0,
            "uart_and_menu_T": float(flash_sys.find_value("uart_and_menu_T")) if flash_sys.find_value("uart_and_menu_T") else 53.0,
            "boost_time_threshold": float(flash_sys.find_value("boost_time_threshold")) if flash_sys.find_value("boost_time_threshold") else 60.0,
            # 路径规划
            "plan_arrive_threshold": float(flash_sys.find_value("plan_arrive_threshold")) if flash_sys.find_value("plan_arrive_threshold") else 1.0,
            "plan_point_transition_T": float(flash_sys.find_value("plan_point_transition_T")) if flash_sys.find_value("plan_point_transition_T") else 50.0,
            # 速度规划
            "min_start_v": float(flash_sys.find_value("min_start_v")) if flash_sys.find_value("min_start_v") else 40.0,
            "long_v_max": float(flash_sys.find_value("long_v_max")) if flash_sys.find_value("long_v_max") else 400.0,
            "short_v_max": float(flash_sys.find_value("short_v_max")) if flash_sys.find_value("short_v_max") else 50.0,
            "dead_zone_v": float(flash_sys.find_value("dead_zone_v")) if flash_sys.find_value("dead_zone_v") else 50.0,
            "transit_v": float(flash_sys.find_value("transit_v")) if flash_sys.find_value("transit_v") else 200.0,
            "orbit_v": float(flash_sys.find_value("orbit_v")) if flash_sys.find_value("orbit_v") else 100.0,
            "move_v_max": float(flash_sys.find_value("move_v_max")) if flash_sys.find_value("move_v_max") else 150.0,
            "scan_v_max": float(flash_sys.find_value("scan_v_max")) if flash_sys.find_value("scan_v_max") else 80.0,
            # 视觉伺服
            "servo_kp_x": float(flash_sys.find_value("servo_kp_x")) if flash_sys.find_value("servo_kp_x") else 8.0,
            "servo_kd_x": float(flash_sys.find_value("servo_kd_x")) if flash_sys.find_value("servo_kd_x") else 10.0,
            "servo_kp_y": float(flash_sys.find_value("servo_kp_y")) if flash_sys.find_value("servo_kp_y") else 8.0,
            "servo_kd_y": float(flash_sys.find_value("servo_kd_y")) if flash_sys.find_value("servo_kd_y") else 10.0,
            "servo_target_x": float(flash_sys.find_value("servo_target_x")) if flash_sys.find_value("servo_target_x") else 80.0,
            "servo_target_y_T": float(flash_sys.find_value("servo_target_y_T")) if flash_sys.find_value("servo_target_y_T") else 5.0,
            "servo_target_y_S": float(flash_sys.find_value("servo_target_y_S")) if flash_sys.find_value("servo_target_y_S") else 8.0,
            "servo_target_y_B": float(flash_sys.find_value("servo_target_y_B")) if flash_sys.find_value("servo_target_y_B") else 30.0,
            "min_rel_speed": float(flash_sys.find_value("min_rel_speed")) if flash_sys.find_value("min_rel_speed") else 50.0,
            "max_rel_speed": float(flash_sys.find_value("max_rel_speed")) if flash_sys.find_value("max_rel_speed") else 250.0,
            "finish_threshold_x": float(flash_sys.find_value("finish_threshold_x")) if flash_sys.find_value("finish_threshold_x") else 2.0,
            "finish_threshold_y": float(flash_sys.find_value("finish_threshold_y")) if flash_sys.find_value("finish_threshold_y") else 4.0,
            "servo_pwmout_limitmax": float(flash_sys.find_value("servo_pwmout_limitmax")) if flash_sys.find_value("servo_pwmout_limitmax") else 250.0,
            # 环绕控制
            "radius_T": float(flash_sys.find_value("radius_T")) if flash_sys.find_value("radius_T") else 2.5,
            "radius_S": float(flash_sys.find_value("radius_S")) if flash_sys.find_value("radius_S") else 4.5,
            "radius_B": float(flash_sys.find_value("radius_B")) if flash_sys.find_value("radius_B") else 13.0,
            # 砖块数量、角点 & 凸起中心
            "block_count": float(flash_sys.find_value("block_count")) if flash_sys.find_value("block_count") else 3.0,
            "block1_corner1": float(flash_sys.find_value("block1_corner1")) if flash_sys.find_value("block1_corner1") else 0.0,
            "block1_corner2": float(flash_sys.find_value("block1_corner2")) if flash_sys.find_value("block1_corner2") else 0.0,
            "block2_corner1": float(flash_sys.find_value("block2_corner1")) if flash_sys.find_value("block2_corner1") else 0.0,
            "block2_corner2": float(flash_sys.find_value("block2_corner2")) if flash_sys.find_value("block2_corner2") else 0.0,
            "block3_corner1": float(flash_sys.find_value("block3_corner1")) if flash_sys.find_value("block3_corner1") else 0.0,
            "block3_corner2": float(flash_sys.find_value("block3_corner2")) if flash_sys.find_value("block3_corner2") else 0.0,
            "bump_center": float(flash_sys.find_value("bump_center")) if flash_sys.find_value("bump_center") else 0.0,
        } # 用字典保存所需改的参数

        # 参数名-缩略名映射字典
        self.param_short_name = {
            # PID 参数
            "angle_normal_kp": "n-kp",
            "angle_normal_ki": "n-ki",
            "angle_normal_kd": "n-kd",
            "integral_limitmax": "int-l",
            "pwmout_limitmax": "pwm-l",
            "high_angle_pwmout_limitmax": "hi-pw",
            "low_angle_pwmout_limitmax": "lo-pw",
            "A": "A",
            "B": "B",
            "gkd": "gkd",
            "speed_fuse_ratio": "fuse",
            "motor_control_T": "mot-T",
            "collect_dt": "col-d",
            "plan_calculate_T": "pln-T",
            "uart_and_menu_T": "u-rt",
            "boost_time_threshold": "bst-T",
            "plan_arrive_threshold": "ariv",
            "plan_point_transition_T": "trn-T",
            "min_start_v": "min-v",
            "long_v_max": "lon-v",
            "short_v_max": "sho-v",
            "dead_zone_v": "dea-v",
            "transit_v": "tra-v",
            "orbit_v": "orb-v",
            "move_v_max": "mov-v",
            "scan_v_max": "sca-v",
            "servo_kp_x": "kp-x",
            "servo_kd_x": "kd-x",
            "servo_kp_y": "kp-y",
            "servo_kd_y": "kd-y",
            "servo_target_x": "tar-x",
            "servo_target_y_T": "t-yT",
            "servo_target_y_S": "t-yS",
            "servo_target_y_B": "t-yB",
            "min_rel_speed": "minSp",
            "max_rel_speed": "maxSp",
            "finish_threshold_x": "fin-x",
            "finish_threshold_y": "fin-y",
            "servo_pwmout_limitmax": "svPW",
            "radius_T": "rd-T",
            "radius_S": "rd-S",
            "radius_B": "rd-B",
            "block_count": "b-cnt",
            "block1_corner1": "b1-c1",
            "block1_corner2": "b1-c2",
            "block2_corner1": "b2-c1",
            "block2_corner2": "b2-c2",
            "block3_corner1": "b3-c1",
            "block3_corner2": "b3-c2",
            "bump_center": "bmp-c",
        }
        gc.collect()

        # IPS200PRO 显示
        self.pro = ips200pro
        self.menu_page = self.pro.page_create("MENU")
        self._init_label_pool()

        # 五向开关引脚（C0=上 C8=下 C9=左 C1=右 C14=按下）
        self.pin_up    = Pin('C0', Pin.IN, Pin.PULL_UP)
        self.pin_down  = Pin('C8', Pin.IN, Pin.PULL_UP)
        self.pin_left  = Pin('C9', Pin.IN, Pin.PULL_UP)
        self.pin_right = Pin('C1', Pin.IN, Pin.PULL_UP)
        self.pin_press = Pin('C14', Pin.IN, Pin.PULL_UP)
        self.beep_enabled = False
        self.key_debounce_ms = 150
        self.last_key_time = 0

        # 注入外部硬件对象
        self.flash_sys = flash_sys
        self.beep = beep

        # 菜单核心配置
        self.change_page_to = 1
        self.Current_line = 2
        self.Start_line, self.End_line = 1, 9
        # 步长
        self.step_values = (0.001, 0.01, 0.1, 1.0, 5.0, 10.0, 100.0)
        self.current_step_index = 3  # 默认步长 1.0
        # 行间距
        self.LineSpacing = 18
        self.PARAM_GAP = 12

        # 屏幕布局
        self.LCD_WIDTH = 200
        self.LABEL_X = 24
        self.ARROW_X = 190
        self.TITLE_EXTRA = 3

        # 五向开关状态
        self.is_param_selected = False
        self.selected_line = None
        self.COLOR_RED = 0xF800

        # 状态标记
        self.need_refresh = True

        # 预定义核心映射
        self._init_core_mappings()

        # 显示初始页
        self.menu_switch()
        self._redraw_current_arrow()

        gc.collect()

    POOL_SIZE = 16  # 最大页SERVO需要15行

    def _init_label_pool(self):
        self.pool = []
        for i in range(self.POOL_SIZE):
            lbl = self.pro.label_create(0, 0, 166, 18, "", self.pro.LABEL_CLIP)
            self.pro.set_hidden(lbl, True)
            self.pro.set_color(lbl, self.pro.COLOR_FOREGROUND, 0x0000)
            self.pro.set_color(lbl, self.pro.COLOR_BACKGROUND, 0xFFFF)
            self.pro.set_color(lbl, self.pro.COLOR_BORDER, 0x8410)
            self.pool.append(lbl)
        self.arrow_lbl = self.pro.label_create(0, 0, 22, 18, "<--", self.pro.LABEL_CLIP)
        self.pro.set_hidden(self.arrow_lbl, True)

    def _line_y(self, line_num):
        """计算第 line_num 行的 y 坐标（step与param之间加 PARAM_GAP）"""
        y = self.TITLE_EXTRA + self.LineSpacing * line_num
        if line_num >= 2:
            y += self.PARAM_GAP
        return y

    def _assign_label(self, idx, text, x, y):
        if idx < len(self.pool):
            self.pro.set_position(self.pool[idx], x, y)
            self.pro.label_string(self.pool[idx], text)
            self.pro.set_hidden(self.pool[idx], False)

    def _show_page(self, page_num):
        for lbl in self.pool:
            self.pro.set_hidden(lbl, True)
        self.line_label = {}
        pool_idx = 0

        title, start_line, end_line = self.page_meta.get(page_num, ("", 1, 14))
        self.Start_line, self.End_line = start_line, end_line
        self.pro.page_name(self.menu_page, title)

        # 步长行
        step_text = f"step: {self.step_values[self.current_step_index]:8.3f}"
        self._assign_label(pool_idx, step_text, self.LABEL_X, self._line_y(1))
        self.line_label[1] = pool_idx
        pool_idx += 1

        # 参数行
        data_lines = self.page_line_map.get(page_num, {})
        max_line = max(data_lines.keys()) if data_lines else 1
        for line_num in range(2, max_line + 1):
            if line_num in data_lines:
                config_key, _ = data_lines[line_num]
                display_text = self._format_param_display(config_key)
                self._assign_label(pool_idx, display_text, self.LABEL_X, self._line_y(line_num))
                self.line_label[line_num] = pool_idx
                pool_idx += 1

        # save行
        save_line = self.save_line_map.get(page_num, 0)
        if save_line:
            self._assign_label(pool_idx, "save", self.LABEL_X, self._line_y(save_line))
            self.line_label[save_line] = pool_idx
            pool_idx += 1

    def refresh_param_line(self, line_num, config_key):
        lbl_idx = self.line_label.get(line_num)
        if lbl_idx is not None:
            display_text = self._format_param_display(config_key)
            self.pro.label_string(self.pool[lbl_idx], display_text)
        self._redraw_current_arrow()
        gc.collect()

    def _redraw_current_arrow(self):
        y = self._line_y(self.Current_line)
        self.pro.set_position(self.arrow_lbl, self.ARROW_X, y)
        self.pro.set_hidden(self.arrow_lbl, False)

        arrow_color = self.COLOR_RED if (self.is_param_selected and self.selected_line == self.Current_line) else 0x0000
        self.pro.set_color(self.arrow_lbl, self.pro.COLOR_FOREGROUND, arrow_color)

    def refresh_current_page(self):
        """刷新箭头显示"""
        self._redraw_current_arrow()
        self.need_refresh = False
        gc.collect()

    def read_key(self):
        """读取五向开关（150ms防抖，蜂鸣器可关闭）"""
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_key_time) < self.key_debounce_ms:
            return None
        self.last_key_time = now

        pressed_key = None

        if self.pin_press.value() == 0:
            if self.beep_enabled:
                self.beep.key_test()
            return "press"

        if self.pin_up.value() == 0:
            if self.beep_enabled:
                self.beep.key_test()
            pressed_key = "up"
        elif self.pin_down.value() == 0:
            if self.beep_enabled:
                self.beep.key_test()
            pressed_key = "down"
        elif self.pin_left.value() == 0:
            if self.beep_enabled:
                self.beep.key_test()
            pressed_key = "left"
        elif self.pin_right.value() == 0:
            if self.beep_enabled:
                self.beep.key_test()
            pressed_key = "right"

        return pressed_key

    def toggle_param_select(self):
        """切换参数选中状态"""
        if not self._is_param_line(self.Current_line):
            self.is_param_selected = False
            self.selected_line = None
            self.refresh_current_page()
            return

        if self.is_param_selected and self.selected_line == self.Current_line:
            self.is_param_selected = False
            self.selected_line = None
        else:
            self.is_param_selected = True
            self.selected_line = self.Current_line

        self.refresh_current_page()

    def destroy(self):
        """释放内存"""
        try:
            for attr in ['config', 'param_short_name']:
                if hasattr(self, attr) and isinstance(getattr(self, attr), dict):
                    getattr(self, attr).clear()
            self.flash_sys = None
            self.beep = None
            self.pro = None
            self.pool = None
            self.arrow_lbl = None
            self.line_label = None
            self.is_param_selected = False
            self.selected_line = None
            self.need_refresh = False
        except Exception as e:
            print(f"Destroy error: {e}")
        gc.collect()

    def update_config_values(self, file_path, updates):
        """安全更新配置文件（临时文件+原子替换）"""
        temp_file_path = file_path + ".tmp"
        try:
            with open(file_path, 'r') as f_in, open(temp_file_path, 'w') as f_out:
                for line in f_in:
                    stripped = line.strip()
                    if not stripped or stripped.startswith('#'):
                        f_out.write(line)
                        continue
                    if '=' in stripped:
                        key, _ = stripped.split('=', 1)
                        key = key.strip()
                        if key in updates:
                            f_out.write(f"{key} = {updates[key]:.3f}\n")
                        else:
                            f_out.write(line)
                    else:
                        f_out.write(line)
            os.remove(file_path)
            os.rename(temp_file_path, file_path)
        except Exception as e:
            if os.path.exists(temp_file_path):
                os.remove(temp_file_path)
            print(f"Config update error: {e}")
        finally:
            gc.collect()

    def save_data(self):
        """保存当前页面参数（三位小数）"""
        updates = {k: round(self.config[k], 3) for k in self.page_configs.get(self.change_page_to, [])}
        if updates:
            self.update_config_values(self.flash_sys.file_path, updates)

        self.Current_line = 2
        self.is_param_selected = False
        self.selected_line = None
        self.need_refresh = True
        gc.collect()

    def data_processing(self, key):
        """处理参数修改（步长直接改，参数行需选中，save行确认保存）"""
        if self.Current_line == 1:
            old_index = self.current_step_index
            if key == "left":
                self.current_step_index = (self.current_step_index - 1) % len(self.step_values)
            elif key == "right":
                self.current_step_index = (self.current_step_index + 1) % len(self.step_values)

            if old_index != self.current_step_index:
                lbl_idx = self.line_label.get(1)
                if lbl_idx is not None:
                    step_text = f"step: {self.step_values[self.current_step_index]:8.3f}"
                    self.pro.label_string(self.pool[lbl_idx], step_text)
                self._redraw_current_arrow()
            gc.collect()
            return

        line_config = self.page_line_map.get(self.change_page_to, {}).get(self.Current_line)
        if line_config and self.is_param_selected and self.selected_line == self.Current_line:
            config_key, _ = line_config
            step = self.step_values[self.current_step_index]

            if key == "left":
                self.config[config_key] = round(self.config[config_key] - step, 3)
            elif key == "right":
                self.config[config_key] = round(self.config[config_key] + step, 3)

            self.refresh_param_line(self.Current_line, config_key)
            gc.collect()
            return

        if self.Current_line == self.save_line_map.get(self.change_page_to, 0) and key == "press":
            self.save_data()

        gc.collect()

    def move_arrow(self, key):
        """上下移动箭头，移动时取消参数选中"""
        if key == "up":
            self.Current_line = self.End_line if self.Current_line <= self.Start_line else self.Current_line - 1
        elif key == "down":
            self.Current_line = self.Start_line if self.Current_line >= self.End_line else self.Current_line + 1

        self.is_param_selected = False
        self.selected_line = None
        self._redraw_current_arrow()
        gc.collect()

    def detect_change_page(self, key):
        """检测并处理8页循环翻页"""
        is_save_line = self.Current_line == self.save_line_map.get(self.change_page_to, 0)
        if (self.is_param_selected is False and self.Current_line != 1) or is_save_line:
            if key == "left":
                self.change_page_to = 8 if self.change_page_to == 1 else self.change_page_to - 1
            elif key == "right":
                self.change_page_to = 1 if self.change_page_to == 8 else self.change_page_to + 1

            self.Current_line = 2
            self.current_step_index = 3  # 翻页步长重置为1.0
            self.is_param_selected = False
            self.selected_line = None
            self.menu_switch()
            self.refresh_current_page()
            gc.collect()
            return True
        return False

    def _init_core_mappings(self):
        """初始化8页参数映射（BLOCK为第1页）"""
        self.page_line_map = {
            1: {
                2: ("block_count", "6.3f"),
                3: ("block1_corner1", "6.3f"),
                4: ("block1_corner2", "6.3f"),
                5: ("block2_corner1", "6.3f"),
                6: ("block2_corner2", "6.3f"),
                7: ("block3_corner1", "6.3f"),
                8: ("block3_corner2", "6.3f"),
                9: ("bump_center", "6.3f"),
            },
            2: {
                2: ("angle_normal_kp", "6.3f"),
                3: ("angle_normal_ki", "6.3f"),
                4: ("angle_normal_kd", "6.3f"),
                5: ("integral_limitmax", "6.3f"),
                6: ("pwmout_limitmax", "6.3f"),
                7: ("high_angle_pwmout_limitmax", "6.3f"),
                8: ("low_angle_pwmout_limitmax", "6.3f"),
                9: ("A", "6.3f"),
                10: ("B", "6.3f")
            },
            3: {
                2: ("gkd", "6.3f"),
                3: ("speed_fuse_ratio", "6.3f")
            },
            4: {
                2: ("motor_control_T", "6.3f"),
                3: ("collect_dt", "6.3f"),
                4: ("plan_calculate_T", "6.3f"),
                5: ("uart_and_menu_T", "6.3f"),
                6: ("boost_time_threshold", "6.3f")
            },
            5: {
                2: ("plan_arrive_threshold", "6.3f"),
                3: ("plan_point_transition_T", "6.3f")
            },
            6: {
                2: ("min_start_v", "6.3f"),
                3: ("long_v_max", "6.3f"),
                4: ("short_v_max", "6.3f"),
                5: ("dead_zone_v", "6.3f"),
                6: ("transit_v", "6.3f"),
                7: ("orbit_v", "6.3f"),
                8: ("move_v_max", "6.3f"),
                9: ("scan_v_max", "6.3f")
            },
            7: {
                2: ("servo_kp_x", "6.3f"),
                3: ("servo_kd_x", "6.3f"),
                4: ("servo_kp_y", "6.3f"),
                5: ("servo_kd_y", "6.3f"),
                6: ("servo_target_x", "6.3f"),
                7: ("servo_target_y_T", "6.3f"),
                8: ("servo_target_y_S", "6.3f"),
                9: ("servo_target_y_B", "6.3f"),
                10: ("min_rel_speed", "6.3f"),
                11: ("max_rel_speed", "6.3f"),
                12: ("finish_threshold_x", "6.3f"),
                13: ("finish_threshold_y", "6.3f"),
                14: ("servo_pwmout_limitmax", "6.3f")
            },
            8: {
                2: ("radius_T", "6.3f"),
                3: ("radius_S", "6.3f"),
                4: ("radius_B", "6.3f")
            }
        }
        self.save_line_map = {
            1: 10, 2: 11, 3: 4, 4: 7, 5: 4, 6: 10, 7: 15, 8: 5
        }
        self.page_configs = {
            1: ["block_count", "block1_corner1", "block1_corner2", "block2_corner1",
                "block2_corner2", "block3_corner1", "block3_corner2", "bump_center"],
            2: ["angle_normal_kp", "angle_normal_ki", "angle_normal_kd", "integral_limitmax",
                "pwmout_limitmax", "high_angle_pwmout_limitmax", "low_angle_pwmout_limitmax",
                "A", "B"],
            3: ["gkd", "speed_fuse_ratio"],
            4: ["motor_control_T", "collect_dt", "plan_calculate_T", "uart_and_menu_T", "boost_time_threshold"],
            5: ["plan_arrive_threshold", "plan_point_transition_T"],
            6: ["min_start_v", "long_v_max", "short_v_max", "dead_zone_v", "transit_v",
                "orbit_v", "move_v_max", "scan_v_max"],
            7: ["servo_kp_x", "servo_kd_x", "servo_kp_y", "servo_kd_y", "servo_target_x",
                "servo_target_y_T", "servo_target_y_S", "servo_target_y_B", "min_rel_speed",
                "max_rel_speed", "finish_threshold_x", "finish_threshold_y", "servo_pwmout_limitmax"],
            8: ["radius_T", "radius_S", "radius_B"]
        }
        self.page_meta = {
            1: ("BLOCK", 1, 10), 2: ("PID", 1, 11), 3: ("COEF", 1, 4),
            4: ("TIME", 1, 7), 5: ("PATH", 1, 4), 6: ("SPEED", 1, 10),
            7: ("SERVO", 1, 15), 8: ("ORBIT", 1, 5)
        }

    def _is_param_line(self, line_num):
        page_map = self.page_line_map.get(self.change_page_to, {})
        return line_num in page_map.keys()

    def _is_save_line(self, line_num):
        return line_num == self.save_line_map.get(self.change_page_to, 0)

    def _format_param_display(self, config_key):
        short_name = self.param_short_name.get(config_key, config_key[:6])
        val = round(self.config[config_key], 3)
        return f"{short_name:<5} : {val:8.3f}"

    def menu_switch(self):
        self._show_page(self.change_page_to)

    def handle_key_from_interrupt(self, key):
        if not key:
            return

        if key == "press":
            if self._is_save_line(self.Current_line):
                self.save_data()
            else:
                self.toggle_param_select()
        elif key in ("up", "down"):
            self.move_arrow(key)
        elif key in ("left", "right"):
            if not self.detect_change_page(key):
                self.data_processing(key)
        if self.need_refresh:
            self.refresh_current_page()

