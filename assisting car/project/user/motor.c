#include "motor.h"
int16 encoder_data_dir_1 = 0;
int16 encoder_data_dir_2 = 0;
int16 encoder_data_dir_3 = 0;
int16 encoder_data_dir_4 = 0;
int16 encoder_data_dir_1_prev = 0;
int16 encoder_data_dir_2_prev = 0;
int16 encoder_data_dir_3_prev = 0;
int16 encoder_data_dir_4_prev = 0;
float carbody_cR=carbody_h*0.5+carbody_w*0.5;//小车中心到轮子距离
PID_motor pid1,pid2,pid3,pid4;
PID_angle pid_angle;
PID_w pid_w;
//电机pwm设置
void set_motor_pwm(pwm_channel_enum channel1,pwm_channel_enum channel2, int duty) {
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < -MAX_DUTY) duty = -MAX_DUTY;
    if (duty >= 0) {
        pwm_set_duty(channel1, duty); // 正转
        pwm_set_duty(channel2, 0); // 反转通道关闭
    } else {
        pwm_set_duty(channel1, 0); // 正转通道关闭
        pwm_set_duty(channel2, -duty); // 反转
    }
}

void motor_PID_Init(PID_motor *p, float Kp, float Ki, float Kd,int integral_max, int output_max) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->integral_max = integral_max;
    p->output_max = output_max;
    p->target = 0.0f;
}

float motor_PID_Update(PID_motor *p, float measure) {
    float error, output;
    // 计算误差
    error = p->target - measure;

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
void angle_PID_Init(PID_angle *p, float Kp, float Ki, float Kd,int integral_max, int output_max) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->integral_max = integral_max;
    p->output_max = output_max;
    p->target = 0.0f;
}

float angle_PID_Update(PID_angle *p, float measure) {
    float error, output;
    // 计算误差
    error = p->target - measure;

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

void w_PID_Init(PID_w *p, float Kp, float Ki, float Kd,int integral_max, int output_max) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->integral_max = integral_max;
    p->output_max = output_max;
    p->target = 0.0f;
}

float w_PID_Update(PID_w *p, float measure) {
    float error, output;
    // 计算误差
    error = p->target - measure;

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
void calculate_motor_target_xy(float target_x, float target_y, float target_w, float *out1, float *out2, float *out3, float *out4) {

    *out1 = target_x + target_y - target_w;
    *out2 = target_x - target_y - target_w;
    *out3 = target_x - target_y + target_w;
    *out4 = target_x + target_y + target_w;
}