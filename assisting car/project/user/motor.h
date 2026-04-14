#ifndef _MOTOR_H_
#define _MOTOR_H_
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float integral;    // 积分累积
    float prev_error;  // 上次误差（用于微分）
    float integral_max; // 积分限幅
    float output_max;  // 输出限幅
} PID_motor;
PID_motor pid1,pid2,pid3,pid4;
void motor_PID_Init(PID_motor *p, float Kp, float Ki, float Kd,float integral_max, float output_max);
float motor_PID_Update(PID_motor *p, float setpoint, float measure);
#endif