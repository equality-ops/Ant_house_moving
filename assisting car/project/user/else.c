#include "else.h"
#include "motor.h"
//button define
#define KEYSTROKE_UP 1
#define KEYSTROKE_DOWN 2
#define KEYSTROKE_COMFIRM 3
#define KEYSTROKE_CANCEL 4

//按键变量定义
uint8 keystroke_label = 0; // 按下的是哪个键
uint8 key_last_status[4] = {0};
uint8 key_status[4] = {0};
uint8 key_flag[4] = {0};

//菜单变量定义
int display_codename = 0; // 显示页面代号
uint8 cursor_row = 0;     // 光标所在行号
uint8 previous_cursor_row = 0;  // 上一次光标所在列号
uint8 menu_next_flag = 0; // 光标所指菜单进入标志位
float change_unit = 0;    // 单次修改的单位值
int change_unit_multiplier = 1; // 修改单位倍数
int keystroke_three_count = 0; // 定义一个全局变量记录KEYSTROKE_THREE的触发次数
uint8 menu_over_flag=0; //菜单操作完成标志

// 需要被修改的参数示例
extern PID_motor pid1;
extern PID_motor pid2;
extern PID_motor pid3;
extern PID_motor pid4;

uint8 date_buff[100]; // eeprom数据数组
uint8 eeprom_init_time = 0;
uint8 menu_count[]={
    2,// 主菜单有项数
    4,
    6};
// 将有菜单页面的代号填入该数组中，防止由箭头所在行号所决定进入不存在的菜单
int menu_have_sub[] = {
    0,
    10, 11,12, 13,14,
    20, 21,22,23, 24,25,26};
char *menu_sub_name[] = {// 菜单项名称
    "MENU",
    "max_12", "i_max1", "out_max1", "i_max2","out_max2",
    "motorpid","pid1_P","pid1_I", "pid1_D", "pid2_P","pid2_I","pid2_D"};
uint32 *menu_object_addr[] = {// 记录菜单项的对象地址，0表示无参数
    0,
    0, (uint32 *)&pid1.integral_max,(uint32 *)&pid1.output_max, (uint32 *)&pid2.integral_max,(uint32 *)&pid2.output_max,
    0, (uint32 *)&pid1.Kp, (uint32 *)&pid1.Ki, (uint32 *)&pid1.Kd,   (uint32 *)&pid2.Kp, (uint32 *)&pid2.Ki, (uint32 *)&pid2.Kd};
uint8 menu_object_type[] = {//record菜单项的对象类型，0表示无参数，1表示int32参数，2表示float参数,3表示特殊
    0,
    0, 2,2, 2,2,
    0, 2,2,2, 2,2,2}; // 0表示无参数，1表示int32参数，2表示float参数,3表示特殊

//蜂鸣器
void beep_once(int duration_ms) {
    gpio_set_level(BEEP_PIN, 0);
    system_delay_ms(duration_ms);
    gpio_set_level(BEEP_PIN, 1);
    system_delay_ms(duration_ms);
    gpio_set_level(BEEP_PIN, 0);
}
// 电压检测
void voltage_detect()
{
    float voltage;
    int adc_data= adc_convert(ADC1_CH0_P10);
    voltage =  (float)11 * 3.3 * adc_data / 4095 ;
    if (voltage>11.1) {
        while(1) {
            gpio_toggle_level(LED1);
            system_delay_ms(3000);
            printf("Battery voltage is normal: %.2f\r\n", voltage);
        }
    }
    else
    {
        while(1) {
            gpio_toggle_level(LED1);
            system_delay_ms(3000);
            printf("Battery voltage is too low: %.2f\r\n", voltage);
        }
    }
}

//=========================以下是EEPROM相关函数========================

void eeprom_init()
{
    iap_init(); // 初始化EEPROM;
    iap_read_buff(0x00, date_buff, 100); // 从EEPROM中读取数据
    eeprom_init_time = read_int(0); // eepeom没有被填充，则会读到垃圾值
    if (eeprom_init_time != 1) // 初次启动，eeprom_init_time为垃圾值，if成立
    {
        eeprom_flash(); // 填充源码变量初始化的值到eeprom
    }
    else // 非初次启动，读取eeprom用于赋值变量
    {
        int i = 1,buff_id=1;
        for (; i < sizeof(menu_have_sub)/sizeof(menu_have_sub[0]); i++) {
            if (menu_have_sub[i]%10 == 0) {
                continue;
            }
            if (menu_object_type[i] == 1) {
                *(int32 *)(menu_object_addr[i]) = read_int(buff_id);
            }
            else if (menu_object_type[i] == 2) {
                *(float *)(menu_object_addr[i]) = read_float(buff_id);
            }
            else if (menu_object_type[i] == 3) {
                *(uint32 *)(menu_object_addr[i]) = read_int(buff_id);
            }
            buff_id++;
        }
    }
}


void eeprom_flash()//将程序的数据输入到eeprom中
{
    uint8 verify_buff[100];
    uint8 i = 1,buff_id=1;
    save_int(1, 0); //表示eeprom是否被初始化过，1表示已初始化
    for (; i < sizeof(menu_have_sub)/sizeof(menu_have_sub[0]); i++)
    {
        if (menu_have_sub[i] % 10 == 0)
        {
            continue;
        }
        if (menu_object_type[i] == 1)
        {
            save_int(*(int32 *)(menu_object_addr[i]), buff_id);
        }
        else if (menu_object_type[i] == 2)
        {
            save_float(*(float *)(menu_object_addr[i]), buff_id);
        }
        else if (menu_object_type[i] == 3)
        {
            save_int(*(uint32 *)(menu_object_addr[i]), buff_id);
        }
        buff_id++;
    }
    iap_erase_page(0); // 擦除地址0所在的扇区数据，一共512个字节
    iap_write_buff(0x00, date_buff, 100);
    iap_read_buff(0x00, verify_buff, 100);
    if (memcmp(date_buff, verify_buff, 100) == 0) {
        printf("EEPROM flash successful and verified.\r\n");
    } else {
        printf("EEPROM flash failed or verification failed.\r\n");
    }
}
void save_int(int32 input, uint8 value_bit)//value_bit表示存储位置，保存int32到eeprom
{
    uint8 i;
    uint8 begin = value_bit * 4;
    uint8 *p = (uint8 *)&input;

    for (i = 0; i < 4; i++)
    {
        date_buff[begin++] = *(p + i);
    }
    
}

int32 read_int(uint8 value_bit)
{
    uint8 i;
    uint8 begin = value_bit * 4;
    int32 output;
    uint8 *p = (uint8 *)&output;
    for (i = 0; i < 4; i++)
    {
        *(p + i) = date_buff[begin++];
    }
    return output;
}

void save_float(float input, uint8 value_bit)
{
    uint8 i;
    uint8 begin = value_bit * 4;
    uint8 *p = (uint8 *)&input;
    for (i = 0; i < 4; i++)
    {
        date_buff[begin++] = *(p + i);
    }
}
void save_float_3(float input, uint8 value_bit)
{
    int32 i= (int32)(input*1000);
    save_int(i, value_bit);
}

float read_float(uint8 value_bit)
{
    uint8 i;
    uint8 begin = value_bit * 4;
    float output;
    uint8 *p = (uint8 *)&output;

    for (i = 0; i < 4; i++)
    {
        *(p + i) = date_buff[begin++];
    }
    return output;
}

float read_float_3(uint8 value_bit)
{
    int32 i = read_int(value_bit);
    return (float)i/1000;
}
//========================以下是菜单函数========================


void Keystroke_Scan(void)// 按键扫描,判断按键状态并更新 keystroke_label
{
    uint8 i = 0;
    keystroke_label = 0;

    // 保存按键状态
    for (i = 0; i < 4; i++)
    {
        key_last_status[i] = key_status[i];
    }
    key_status[0] = gpio_get_level(KEY1_PIN); // 按键按下则值为1
    key_status[1] = gpio_get_level(KEY2_PIN);
    key_status[2] = gpio_get_level(KEY3_PIN);
    key_status[3] = gpio_get_level(KEY4_PIN);

    for (i = 0; i < 4; i++)
    {
        if (key_status[i] && !key_last_status[i])
        {
            keystroke_label = i + 1;
            beep_once(10);
            break;  // 一次只响应一个按键，所以有一个按键响应则可以跳出循环
        }
    }
}

// 菜单箭头标识
void Cursor(uint8 rows_max)//更新光标显示，并且更新光标所在行号cursor_row
{
    menu_next_flag = 0;
    switch (keystroke_label)
    {
    case KEYSTROKE_UP:
        cursor_row = (cursor_row > ROWS_MIN) ? cursor_row - 1 : rows_max; // 光标行上移，如果cursorRow达到最上层，再上则回归最下层
        break;
    case KEYSTROKE_DOWN:
        cursor_row = (cursor_row < rows_max) ? cursor_row + 1 : ROWS_MIN; // 光标行下移，如果cursorRow达到最下层，再下则回归最上层
        break;
    case KEYSTROKE_COMFIRM:
        menu_next_flag = 1;
        break;
    case KEYSTROKE_CANCEL:
        menu_next_flag = -1;
        break;
    }
    ips200_show_string(0, cursor_row * line_space, ">"); // 在 cursor_row 对应位置打印箭头
    // 清除之前箭头位置的显示，避免残留
    if (previous_cursor_row != cursor_row)
    {
        ips200_show_string(0, previous_cursor_row * line_space, " ");  // 在 previous_cursor_row 对应位置打印空格
        previous_cursor_row = cursor_row;
    }
}

// 菜单上下级跳转
void Menu_Next_Back()
{
    switch (menu_next_flag)
    {
    case 0:
        break;
    case -1: // 返回上一级
        display_codename = display_codename / 10;
        cursor_row = ROWS_MIN;
        ips200_clear(RGB565_WHITE);										//清屏
        break;
    case 1: // 进入下一级
        if (display_codename/10 == 0 && Have_Sub_Menu(cursor_row*10)) // 如果当前在主菜单，进入下一级菜单时cursor_row不乘10
        {
            display_codename = cursor_row*10; // 进入下一级菜单，display_codename增加一位，个位为0
            cursor_row = ROWS_MIN;  
        }
        else if(Have_Sub_Menu(display_codename+ cursor_row))
        {
            display_codename = display_codename+ cursor_row; // 进入下一级菜单，改变display_codename的个位为cursor_row
        }
        ips200_clear(RGB565_WHITE);										//清屏
        break;
    }
    menu_next_flag = 0; // 切换完页面，标志位归0
}

// 检查本行是否存在子菜单
int Have_Sub_Menu(int menu_id)
{
    uint8 i = 0;
    // sizeof(menu_have_sub) / sizeof(menu_have_sub [0]) 计算数组长度
    for (i = 0; i < sizeof(menu_have_sub) / sizeof(menu_have_sub[0]); i++)
    {
        if (menu_have_sub[i] == menu_id)
        {
            return i;// 存在子菜单，返回其在menu_have_sub数组中的索引
        }
    }
    return 0;
}


void HandleKeystroke(int keystroke_label)// 判断是否按下KEYSTROKE_CANCEL返回，是否按下KEYSTROKE_COMFIRM修改参数单位
{
    switch (keystroke_label)
    {
    case KEYSTROKE_CANCEL:
        display_codename /= 10; // 返回上一页
        ips200_clear(RGB565_WHITE);										//清屏
        break;
    case KEYSTROKE_COMFIRM:
        keystroke_three_count++;
        switch (keystroke_three_count % 3)
        {
        case 0:
            change_unit_multiplier = 1;
            keystroke_three_count = 0;
            break;
        case 1:
            change_unit_multiplier = 10;
            break;
        case 2:
            change_unit_multiplier = 100;
            break;
        }
    break;
    }
}

// 整型参数修改
void Keystroke_int(int *parameter, int change_unit_MIN)
{
    int change_unit = change_unit_MIN * change_unit_multiplier;
    ips200_show_int32(15 * character_space, 0*line_space, change_unit, 3);
    Keystroke_Scan();
    HandleKeystroke(keystroke_label);

    switch (keystroke_label)
    {
    case KEYSTROKE_UP:
        *parameter += change_unit;
        break;
    case KEYSTROKE_DOWN:
        *parameter -= change_unit;
        break;
    }
}

// 浮点型参数修改
void Keystroke_float(float *parameter, float change_unit_MIN)
{
    float change_unit = change_unit_MIN * change_unit_multiplier;
    ips200_show_float(14 * character_space, 0 * line_space, change_unit, 2, 3);
    Keystroke_Scan();
    HandleKeystroke(keystroke_label);
    switch (keystroke_label)
    {
    case KEYSTROKE_UP:
        *parameter += change_unit;
        break;
    case KEYSTROKE_DOWN:
        *parameter -= change_unit;
        break;
    }
}

// 整型特值修改，-1或1
void Keystroke_Special_Value(int *parameter)
{
    Keystroke_Scan();
    HandleKeystroke(keystroke_label);
    switch (keystroke_label)
    {
    case KEYSTROKE_UP:
        *parameter = -1;
        break;
    case KEYSTROKE_DOWN:
        *parameter = 1;
        break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单目录
//  @param
//  @return     void
//  @note       启用while来显示目标页面  在每个页面按键按键后都会改变到对应页面，此函数用于更新屏幕显示
//             增删页的同时请记得同步修改menu_have_sub[]数组的值
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu(void)
{
    if (display_codename / 10 == 0) // 如果是主菜单
    {
        Keystroke_Menu_HOME();
    }
    else if (display_codename / 10 <= menu_count[0])// 如果是子菜单
    {
        Keystroke_Menu_sub();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      主菜单目录
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_HOME(void) // 0
{
    while (menu_next_flag == 0)
    {
        uint8 i=1,now_line=1;
        ips200_show_string(CENTER_COLUMN - 2 * character_space, line_space * 0, "MENU"); 			//显示字符串
        for (;i < sizeof(menu_have_sub) / sizeof(menu_have_sub[0]); i++)
        {
            if (menu_have_sub[i] % 10 == 0){// 根据display_codename来显示本页的菜单项
                ips200_show_string(1 * character_space, line_space * now_line, menu_sub_name[i]);
                now_line++;
            }
        }
        Keystroke_Scan();
        Cursor(menu_count[0]);
    }
    if (menu_next_flag == 1 && Have_Sub_Menu(cursor_row*10)) // 进入下一级菜单
    {
        display_codename = cursor_row*10; // 进入下一级菜单，display_codename增加一位，个位为0
        cursor_row = ROWS_MIN;
        ips200_clear(RGB565_WHITE);//清屏
    }
    else if (menu_next_flag == -1 && EEPROM_MODE == 1) // 在主菜单时按下回退键（按键4）来进行eeprom确认刷写
    {
        eeprom_flash();
        // 刷写完成提示
        ips200_clear(RGB565_WHITE);
        ips200_show_string(1*character_space, 3 * line_space, "EEPROM Save Success!");
        system_delay_ms(400);
        beep_once(100);
        menu_over_flag = 1; // 菜单操作完成标志位置1
        // ips200_clear(RGB565_WHITE);										//清屏
    }
    menu_next_flag = 0; // 切换完页面，标志位归0
}


/*///////////////////////////////////////
   子页面
*/
///////////////////////////////////////
/*
void Menu_ONE_Display(uint8 control_line) 
{
    
    ips200_show_string(0 * character_space, 0 * line_space, "<<STRAT");
    ips200_show_string(1 * character_space, 1 * line_space, "STRAT_FLAG");
    ips200_show_string(1 * character_space, 2 * line_space, "OUT_DIRECTION");
    ips200_show_int32(14 * character_space, 1 * line_space, start_flag, 3);    // “1” 应该与该函数被调用时control_line参数一致，才能正确显示&表示在调整的变量
    ips200_show_int32(14 * character_space, 2 * line_space, garage_out_direction, 3);
    if (control_line != -1)
        ips200_show_string(0 * character_space, control_line * line_space, "~"); //&标志提示
}
*/

void Keystroke_Menu_sub(void)
{
    if(display_codename%10 == 0) {
        while (menu_next_flag == 0)
        {
            Menu_Display_sub(-1, display_codename/10);
            Keystroke_Scan();
            Cursor(menu_count[display_codename/10]);   
        }
        Menu_Next_Back();
    }
    else{
        uint8 now_id = Have_Sub_Menu(display_codename);
        Menu_Display_sub(display_codename%10, display_codename/10);
        switch (menu_object_type[now_id]) {
            case 1:
                Keystroke_int((int *)menu_object_addr[now_id], 1);
                break;
            case 2:
                Keystroke_float((float *)menu_object_addr[now_id], 0.001);
                break;
            case 3:
                Keystroke_Special_Value((int *)menu_object_addr[now_id]);
                break;
            default:
                break;
        }
    }
}

void Menu_Display_sub(uint8 control_line,uint8 menu_id)
{
    uint8 i=1,now_line=1,now_idx=Have_Sub_Menu(menu_id*10);
    i=now_idx+1;// 从第一个子菜单开始显示
    ips200_show_string(0 * character_space, 0 * line_space, "<<");
    ips200_show_string(2 * character_space, 0 * line_space, menu_sub_name[now_idx]); 			//显示字符串
    for (;menu_have_sub[i] / 10 == menu_id; i++)
    {
        ips200_show_string(1 * character_space, line_space * now_line, menu_sub_name[i]);
        switch (menu_object_type[i]) {
            case 1:
                ips200_show_int32(20 * character_space, line_space * now_line, *(int *)menu_object_addr[i], 3);
                break;
            case 2:
                ips200_show_float(20 * character_space, line_space * now_line, *(float *)menu_object_addr[i], 4, 3);
                break;
            case 3:
                ips200_show_int32(20 * character_space, line_space * now_line, *(int *)menu_object_addr[i], 3);
                break;
            default:
                break;
        }
        now_line++;
    }
    if (control_line != -1)
        ips200_show_string(0 * character_space, control_line * line_space, "~"); //&标志提示
}
/*
void Keystroke_Menu_ONE(void) // 1 11 12
{
    switch (display_codename)
    {
    case 1:
        while (menu_next_flag == 0)
        {
            Menu_ONE_Display(-1);
            Keystroke_Scan();
            Cursor();   
        }
        Menu_Next_Back();
        break;
    case 11:
        Menu_ONE_Display(1);
        Keystroke_Special_Value(&start_flag);
        break;
    case 12:
        Menu_ONE_Display(2);
        Keystroke_Special_Value(&garage_out_direction);
        break;
    }

}

void Menu_TWO_Display(uint8 control_line)
{
    ips200_show_string(0 * character_space, 0 * line_space, "<<PID_SPEED");

    ips200_show_string(1 * character_space, 1 * line_space, "P");
    ips200_show_string(1 * character_space, 2 * line_space, "D");
    ips200_show_string(1 * character_space, 3 * line_space, "normal_speed");

    ips200_show_float(14 * character_space, 1 * line_space, PID_P, 2, 3);
    ips200_show_float(14 * character_space, 2 * line_space, PID_D, 2, 3);
    ips200_show_int32(14 * character_space, 3 * line_space, normal_speed, 3);
    if (control_line != -1)
        ips200_show_string(0 * character_space, control_line * line_space, "~"); //&标志提示
}

void Keystroke_Menu_TWO(void) // 2 21 22 23
{
    switch (display_codename)
    {
    case 2:
        while (menu_next_flag == 0)
        {
            Menu_TWO_Display(-1);
            Keystroke_Scan();
            Cursor();
        }
        Menu_Next_Back();
        break;

    case 21:
        Menu_TWO_Display(1);
        Keystroke_float(&PID_P, 0.001);
        break;
    case 22:
        Menu_TWO_Display(2);
        Keystroke_float(&PID_D, 0.001);
        break;
    case 23:
        Menu_TWO_Display(3);
        Keystroke_int(&normal_speed, 1);
        break;
    }
}
*/