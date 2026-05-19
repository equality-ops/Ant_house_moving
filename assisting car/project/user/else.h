#ifndef _ELSE_H_
#define _ELSE_H_
//这里放一些其他的函数声明或者宏定义等
#include "zf_common_headfile.h"
#include <stddef.h>
//LED define
#define LED1                        (IO_P52)
//PIT define
#define PIT1                         (TIM0_PIT)
#define PIT2                         (TIM1_PIT)
#define PIT3                         (TIM9_PIT)
#define PIT4                         (TIM3_PIT)
#define PIT5                         (TIM8_PIT)
//ADC define
#define ADC_CHANNEL1            ( ADC1_CH0_P10 )
//button define
#define KEY1_PIN        IO_PB2
#define KEY2_PIN        IO_PB3
#define KEY3_PIN        IO_PB4
#define KEY4_PIN        IO_P32
#define KEY_START       IO_P23
#define SWITCH1_PIN     IO_PB0
#define SWITCH2_PIN     IO_PB1

//beep define
#define BEEP_PIN (IO_P65)

//menu define
#define ROWS_MAX 7           // 光标在屏幕上可移动至的最大行数
#define ROWS_MIN 1           // 光标在屏幕上可移动至的最小行数
#define CENTER_COLUMN 10 * 8 // 中央列
#define EEPROM_MODE 1        // eeporm读写开启则为1
#define line_space 16           // 行间距
#define character_space 8         // 字符间距

#define area_HEIGHT 2400
#define area_WIDTH 3200
typedef enum {
    FALSE = 0,
    TRUE = 1
} BOOL;

extern uint8 date_buff[100]; //eeprom数据数组
extern uint8 keystroke_label;
extern uint8 menu_over_flag; //菜单操作完成标志

//eepom相关函数声明
void eeprom_init();
void eeprom_flash();
void save_int(int32 input, uint8 value_bit);
int32 read_int(uint8 value_bit);
void save_float_3(float input, uint8 value_bit);
float read_float_3(uint8 value_bit);
void save_float(float input, uint8 value_bit);
float read_float(uint8 value_bit);

void Keystroke_Scan(void);
void Menu_Next_Back(void);
int Have_Sub_Menu(int menu_id);
void Keystroke_Menu(void);
void Keystroke_Menu_HOME(void);
void Keystroke_Menu_sub(void);
void Menu_Display_sub(uint8 control_line,uint8 menu_id);
/*
void Keystroke_Menu_ONE(void);
void Keystroke_Menu_TWO(void);
*/

void beep_once(int duration_ms);
void voltage_detect(void);

float max(float a, float b);
float min(float a, float b);
#endif