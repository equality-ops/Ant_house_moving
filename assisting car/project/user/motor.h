#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "zf_common_headfile.h"
// 最大pwm占空比
#define MAX_DUTY              ( 50 )                                                
// PWM引脚分配
#define PWM_A_1               ( PWMD_CH3_P52 )
#define PWM_A_2               ( PWMD_CH4_P53 )
#define PWM_B_1               ( PWMD_CH1_P50 )
#define PWM_B_2               ( PWMD_CH2_P51 )
#define PWM_C_1               ( PWMB_CH2_P75 )
#define PWM_C_2               ( PWMB_CH1_P74 )
#define PWM_D_1               ( PWMB_CH4_P77 )
#define PWM_D_2               ( PWMB_CH3_P76 )
// 编码器引脚分配
#define PIT_CH                          (TIM1_PIT )                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
//#define PIT_PRIORITY                    (TIM1_IRQn)               TIM1的中断优先级默认最低，不可修改，具体看手册。
#define ENCODER_QUAD_1                 	(PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_QUAD_1_CHA            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_QUAD_1_CHB              (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚
#define ENCODER_QUAD_2                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_2_CHA   		    (PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_QUAD_2_CHB       	    (PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

#define ENCODER_QUAD_3                 	(PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_QUAD_3_CHA            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_QUAD_3_CHB              (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚
#define ENCODER_QUAD_4                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_4_CHA   		    (PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_QUAD_4_CHB       	    (PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

extern int16 encoder_data_dir_1;
extern int16 encoder_data_dir_2;
extern int16 encoder_data_dir_3;
extern int16 encoder_data_dir_4;
// PID 结构体定义
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float integral;    // 积分累积
    float prev_error;  // 上次误差（用于微分）
    float integral_max; // 积分限幅
    float output_max;  // 输出限幅
    float target;     // 目标值
} PID_motor;
extern void set_motor_pwm(pwm_channel_enum channel1,pwm_channel_enum channel2, int duty);
extern PID_motor pid1,pid2,pid3,pid4;
extern void motor_PID_Init(PID_motor *p, float Kp, float Ki, float Kd,float integral_max, float output_max);
extern float motor_PID_Update(PID_motor *p, float measure);
#endif