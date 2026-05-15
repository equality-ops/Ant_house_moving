#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "zf_common_headfile.h"
#include "quaternion.h"
// 最大pwm占空比
#define MAX_DUTY              ( 7000 )                                                
// PWM引脚分配
#define DIR_1 (IO_P72)
#define PWM_1 (PWMD_CH3_P26)
#define DIR_3 (IO_P73)
#define PWM_3 (PWMD_CH4_P27)
#define DIR_4 (IO_P74)
#define PWM_4 (PWMF_CH3_PA5)
#define DIR_2 (IO_P75)
#define PWM_2 (PWMF_CH4_PA7)
// 编码器引脚分配
#define PIT_CH                          (TIM1_PIT )                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
//#define PIT_PRIORITY                    (TIM1_IRQn)               TIM1的中断优先级默认最低，不可修改，具体看手册。
#define ENCODER_QUAD_4 (PWMA_ENCODER)// 带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_4_CHA (PWMA_ENCODER_CH1P_P60)//PULSE对应的引脚
#define ENCODER_QUAD_4_CHB (PWMA_ENCODER_CH2P_P62)//DIR对应的引脚

#define ENCODER_QUAD_1 (PWMC_ENCODER)//带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_1_CHA (PWMC_ENCODER_CH1P_P40)//PULSE对应的引脚
#define ENCODER_QUAD_1_CHB (PWMC_ENCODER_CH2P_P42)//DIR对应的引脚

#define ENCODER_QUAD_3 (PWMB_ENCODER)
#define ENCODER_QUAD_3_CHA (PWMB_ENCODER_CH1_P00)//PULSE对应的引脚
#define ENCODER_QUAD_3_CHB (PWMB_ENCODER_CH2_P01) // DIR 对应的引脚

#define ENCODER_QUAD_2 (PWME_ENCODER)//带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_2_CHA (PWME_ENCODER_CH1P_P90)//PULSE对应的引脚
#define ENCODER_QUAD_2_CHB (PWME_ENCODER_CH2P_P92) // DIR 对应的引脚

#define carbody_h 100.0f//前后麦轮的距离
#define carbody_w 50.0f//左右麦轮的距离

extern int16 encoder_data_dir_1;
extern int16 encoder_data_dir_2;
extern int16 encoder_data_dir_3;
extern int16 encoder_data_dir_4;
extern int16 encoder_data_dir_1_prev;
extern int16 encoder_data_dir_2_prev;
extern int16 encoder_data_dir_3_prev;
extern int16 encoder_data_dir_4_prev;

// PID 结构体定义
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float integral;    // 积分累积
    float prev_error;  // 上次误差（用于微分）
    int integral_max; // 积分限幅
    int output_max;  // 输出限幅
    volatile float target;     // 目标值
} PID_motor;//电机pid速度环
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float integral;    // 积分累积
    float prev_error;  // 上次误差（用于微分）
    int integral_max; // 积分限幅
    int output_max;  // 输出限幅
    float target;     // 目标值   
} PID_angle;//角度pid角度环
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float integral;    // 积分累积
    float prev_error;  // 上次误差（用于微分）
    int integral_max; // 积分限幅
    int output_max;  // 输出限幅
    volatile float target;     // 目标值   
} PID_w;//角度pid角度环
// 位置 XY PID 控制参数（微分项为位置误差变化率）
typedef struct {
    float Kp, Kd;
    float prev_error;
    int   output_max;
    float target_x;
    float target_y;
} PID_xy;
typedef struct {
    volatile int encode1_data;
    volatile int encode2_data;
    volatile int encode3_data;
    volatile int encode4_data;
    volatile int encode1_data_prev_5ms;
    volatile int encode2_data_prev_5ms;
    volatile int encode3_data_prev_5ms;
    volatile int encode4_data_prev_5ms;
    volatile int encode1_delta_5ms;
    volatile int encode2_delta_5ms;
    volatile int encode3_delta_5ms;
    volatile int encode4_delta_5ms;
} ENCODER_DATA;
typedef struct {
    float x;          // X 坐标
    float y;          // Y 坐标
    float yaw;        // 航向角（弧度）
    float speed_x;    // X 方向速度
    float speed_y;    // Y 方向速度
    float speed_w;    // 角速度（弧度/秒）
} CAR_ATTITUDE;
// 目标控制量（包含位置、速度、各轮速度及模式）
typedef struct {
    float x, y, yaw;
    float speed_x, speed_y, speed_w;
    uint8 mode;     // 控制模式：0~6
    float v_wheel1, v_wheel2, v_wheel3, v_wheel4;
} TARGET_ATTITUDE;

extern float carbody_cR;           // 小车中心到轮子距离（由 carbody_h, carbody_w 计算）
extern ENCODER_DATA encoder_data;
extern PID_motor pid1, pid2, pid3, pid4;
extern PID_angle pid_angle;
extern PID_w pid_w;
extern PID_xy pid_xy;

void set_motor_pwm(uint8 wheel, int duty);

// 电机 PID 相关
void motor_PID_Init(PID_motor *p, float Kp, float Ki, float Kd, int integral_max, int output_max);
float motor_PID_Update(PID_motor *p, int measure);

// 角度 PID 相关
void angle_PID_Init(PID_angle *p, float Kp, float Ki, float Kd, int integral_max, int output_max);
float angle_PID_Update(PID_angle *p, float measure);

// 角速度 PID 相关
void w_PID_Init(PID_w *p, float Kp, float Ki, float Kd, int integral_max, int output_max);
float w_PID_Update(PID_w *p, float measure);

// 位置 XY PID 相关
void xy_PID_Init(PID_xy *p, float Kp, float Kd, int output_max);
void xy_PID_Update(PID_xy *p, CAR_ATTITUDE *car, CAR_ATTITUDE *target_speed);
// 小车姿态设置
void Set_CAR_ATTITUDE(CAR_ATTITUDE *car, float x, float y, float yaw, float speed_x, float speed_y, float speed_w);
void Set_TARGET_ATTITUDE(TARGET_ATTITUDE *target, float x, float y, float yaw, float speed_x, float speed_y, float speed_w, float v1, float v2, float v3, float v4, uint8 mode);

// 编码器数据处理
void Encoder_Init_Data(ENCODER_DATA *p);
void Encoder_Update_5ms(ENCODER_DATA *p);

// 运动学变换
void calculate_motortarget_by_vxy(CAR_ATTITUDE *target_speed, float *out);   // 输出数组长度为4
void calculate_vehicle_coordinate_by_encode(CAR_ATTITUDE *car, ENCODER_DATA *p, float kx, float ky);

// 导航控制（根据模式设置 PID 目标）
void set_nevigate_target(TARGET_ATTITUDE *target);  // 0:位移控制 1:角度控制 2:角度+位移控制 3:角速度控制 4:速度控制 5:控制指定轮 6:无控制

#endif