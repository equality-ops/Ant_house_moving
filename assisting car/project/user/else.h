#ifndef _ELSE_H_
#define _ELSE_H_
//这里放一些其他的函数声明或者宏定义等
#include "zf_common_headfile.h"
//LED define
#define LED1                        (IO_P52)
//PIT define
#define PIT1                         (TIM0_PIT)
#define PIT2                         (TIM1_PIT)
//ADC define
#define ADC_CHANNEL1            ( ADC1_CH0_P10 )
//button define
#define KEY1_PIN        IO_PB2
#define KEY2_PIN        IO_PB3
#define KEY3_PIN        IO_PB4
#define KEY4_PIN        IO_P32
#define SWITCH1_PIN     IO_PB0
#define SWITCH2_PIN     IO_PB1
//beep define
#define BEEP_PIN IO_P65
void beep_once(void);
#endif