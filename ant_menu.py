from machine import *
from display import *
from smartcar import ticker,encoder
from user_main import lcd,encoder_l,encoder_r

# 闭环控制回调
def time_pit2_handler(time):
    # ant_key.button_scan() # 函数：按键扫描（后续要补）
    # ant_beep.Beep_Operate() # 函数：响应蜂鸣器操作(后续要补)
    ant_motor.encl_data, ant_motor.encr_data = encoder_l.get(), -encoder_r.get()
    # 这部分操作需结合后续其他文件情况！！！！

# 定时器初始化
def pit2_Start():
    pit2 = ticker(1)
    pit2.callback(time_pit2_handler)
    pit2.start(10)

# 当前菜单项
change_page_to = 0  # 将菜单定位到哪一页
Current_line = 0  # 当前行(公用)
Start_line, End_line = 0, 0 # 显示的起始行，结束行（公用）

# 显示箭头
def show_arrow():
    global Start_line,End_line,Current_line
    lcd.str16(0,16*19,"line={:<2d}".format(Current_line),0xFFFF)
    for i in range(Start_line, End_line + 1):
        if i == Current_line:
            lcd.str16(200,16*i,"<--",0xFFFF)
        else:
            lcd.str16(200,16*i,"   ",0xFFFF)

# 箭头上移
def arrow_up():
    global Start_line,End_line,Current_line
    # if ant_key.button_state["KEY_Up"]["state"] == ant_key.KeyState.SHORT:
    #     ant_key.clean_all_button_state()
    #     Current_line = End_line if Current_line <= Start_line else Current_line - 1
    #     show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

# 箭头下移
def arrow_down():
    global Start_line,End_line,Current_line
    # if hqu_key.button_state["KEY_Down"]["state"] == hqu_key.KeyState.SHORT:
    #    hqu_key.clearn_all_button_state()
    #    Current_line = Start_line if Current_line >= End_line else Current_line + 1
    #    show_arrow()
    # 判断按键状态，清除状态并且进行箭头的移动

# 箭头的移动,包含上移和下移
def move_arrow():
    arrow_up()
    arrow_down() 

# 监测指定的跳转页面行是否被按下，并指定目标页面
def detect_change_page(detect_line,target_page):
    global change_page_to
    # if ant_key.button_state["KEY_Enter"]["state"] == ant_key.KeyState.SHORT and Current_line == detect_line:
    #    ant_key.clearn_button_state("KEY_Enter")
    #    change_page_to = target_page
    return True
    # 预期将判断行的行数和目标页的页数设置为相等的

# 菜单Menu_First的按键左移和按键右移所对应的赋值操作（预期实现切换方案）
def MenuFirst_left_right_Operation():
    global current_scheme_index
    # 左移操作
    
    # 右移操作


#函数:菜单Menu_First（主菜单）
def Menu_First():
    global change_page_to
    global Start_line,End_line,Current_line
    Start_line,End_line,Current_line=0,6,0 # End_line不一定为6
    lcd.clear(0x0000)
    # lcd.str16(0,16 * 0,"Speed: < {:<3} > ".format(ant_config.scheme_profiles[current_scheme_index]["speed_normal"]), 0xFFFF)
    # lcd.str16(0,16 * 1,"Data",0xFFFF)
    # lcd.str16(0,16 * 2,"CCD",0xFFFF)
    # lcd.str16(0,16 * 3,"Ring",0xFFFF)
    # lcd.str16(0,16 * 4,"Color",0xFFFF)
    # lcd.str16(0,16 * 5,"Start",0xFFFF)
    # lcd.str16(0,16 * 6,"SAVE",0xFFFF)
    show_arrow()
    while True:
        # 数据显示

        # 按键操作
        move_arrow()
        MenuFirst_left_right_Operation()
        # 发车操作 ---> 第x行

        # 保存操作 ---> 第y行

        # 菜单切换
        if detect_change_page(detect_line=1,target_page=1):
            break
        if detect_change_page(detect_line=2,target_page=2):
            break
        if detect_change_page(detect_line=3,target_page=3):
            break
        if detect_change_page(detect_line=4,target_page=4):
            break

# 第一页菜单数据显示
def Menu_Page1_datashow():
    pass

#函数：第 1 页菜单显示
def Menu_Page1():
    global change_page_to
    global Start_line,End_line,Current_line
    Start_line,End_line,Current_line=0,16,0
    lcd.clear(0x0000)
    Menu_Page1_datashow()

    show_arrow()
    while True:
        move_arrow()
        Menu_Page1_datashow()
        # if detct_change_page(detect_line=18,target_page=0):
            #break

#函数：菜单选择与切换
def menu_switch():
    if(change_page_to == 0):
        Menu_First()
    elif(change_page_to == 1):
        Menu_Page1()