#include "motor.h"


void motor_PID_Init(PID_motor *p, float Kp, float Ki, float Kd,
                float integral_max, float output_max) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->integral_max = integral_max;
    p->output_max = output_max;
}

float motor_PID_Update(PID_motor *p, float setpoint, float measure) {
    float error, output;
    // 计算误差
    error = setpoint - measure;

    // 积分累加 + 限幅（防积分饱和）
    p->integral += error;
    if (p->integral >  p->integral_max) p->integral =  p->integral_max;
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;

    // PID 输出
    output = p->Kp * error
           + p->Ki * p->integral
           + p->Kd * (error - p->prev_error);

    // 输出限幅
    if (output >  p->output_max) output =  p->output_max;
    if (output < -p->output_max) output = -p->output_max;

    // 保存误差
    p->prev_error = error;

    return output;
}