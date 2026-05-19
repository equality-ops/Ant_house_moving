#include "motor.h"
#include "math.h"
#include "quaternion.h"
float carbody_cR = carbody_h * 0.5 + carbody_w * 0.5;  // 小车中心到轮子距离
ENCODER_DATA encoder_data;
PID_motor pid1, pid2, pid3, pid4;
PID_angle pid_angle;
PID_w pid_w;
PID_xy pid_xy;
pwm_channel_enum find_pwm_channel[4]={PWM_1,PWM_2,PWM_3,PWM_4};
gpio_pin_enum find_dir_channel[4]={DIR_1,DIR_2,DIR_3,DIR_4};
gpio_level_enum find_dir[4][2]={{GPIO_LOW,GPIO_HIGH},{GPIO_HIGH,GPIO_LOW},{GPIO_HIGH,GPIO_LOW},{GPIO_HIGH,GPIO_LOW}};
// 电机pwm设置
void set_motor_pwm(uint8 wheel, int duty) {
    pwm_channel_enum pwm=find_pwm_channel[wheel];
    gpio_pin_enum dir=find_dir_channel[wheel];
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < -MAX_DUTY) duty = -MAX_DUTY;
    if (duty >= 0) {
        pwm_set_duty(pwm, duty);   // 正转
        gpio_set_level(dir, find_dir[wheel][0]);
    } else {
        pwm_set_duty(pwm, -duty);  // 反转
        gpio_set_level(dir, find_dir[wheel][1]);
    }
}

void motor_PID_Init(PID_motor *p, float Kp, float Ki, float Kd, int integral_max, int output_max) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
    p->integral_max = integral_max;
    p->output_max = output_max;
    p->target = 0.0f;
}

float motor_PID_Update(PID_motor *p, int measure) {
    float error, output;
    // 计算误差
    error = p->target - measure;
    // 积分累加 + 限幅（防积分饱和）
    p->integral += error;
    if (p->integral > p->integral_max) p->integral = p->integral_max;
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;
    // PID 输出
    output = p->Kp * error
           + p->Ki * p->integral
           + p->Kd * (error - p->prev_error);
    // 输出限幅
    if (output > p->output_max) output = p->output_max;
    if (output < -p->output_max) output = -p->output_max;
    // 保存误差
    p->prev_error = error;
    return output;
}

void angle_PID_Init(PID_angle *p, float Kp, float Ki, float Kd, int integral_max, int output_max) {
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
    if (error > 180.0f) error -= 360.0f;  // 误差限制在 -180~180 度范围内
    if (error < -180.0f) error += 360.0f;
    // 积分累加 + 限幅（防积分饱和）
    p->integral += error;
    if (p->integral > p->integral_max) p->integral = p->integral_max;
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;
    // PID 输出
    output = p->Kp * error
           + p->Ki * p->integral
           + p->Kd * (error - p->prev_error);
    // 输出限幅
    if (output > p->output_max) output = p->output_max;
    if (output < -p->output_max) output = -p->output_max;
    // 保存误差
    p->prev_error = error;
    return output;
}

void w_PID_Init(PID_w *p, float Kp, float Ki, float Kd, int integral_max, int output_max) {
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
    if (p->integral > p->integral_max) p->integral = p->integral_max;
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;
    // PID 输出
    output = p->Kp * error
           + p->Ki * p->integral
           + p->Kd * (error - p->prev_error);
    // 输出限幅
    if (output > p->output_max) output = p->output_max;
    if (output < -p->output_max) output = -p->output_max;
    // 保存误差
    p->prev_error = error;
    return output;
}

void xy_PID_Init(PID_xy *p, float Kp, float Kd, int output_max) {
    p->Kp = Kp;
    p->Kd = Kd;
    p->prev_error = 0.0f;
    p->output_max = output_max;
    p->target_x = 0.0f;
    p->target_y = 0.0f;
}
void xy_PID_Update(PID_xy *p, CAR_ATTITUDE *car, CAR_ATTITUDE *target_speed) {
    float errorx, errory, error, speed, angle_to_target;
    // 计算误差
    errorx = p->target_x - car->x;
    errory = p->target_y - car->y;
    error = sqrt(errorx * errorx + errory * errory);
    // PID 输出
    speed = p->Kp * error + p->Kd * (error - p->prev_error);
    // 输出限幅
    if (speed > p->output_max) speed = p->output_max;
    if (speed < -p->output_max) speed = -p->output_max;
    // 保存误差
    p->prev_error = error;
    angle_to_target = my_atan2(errory, errorx) - car->yaw;  // 目标方向与小车当前航向的夹角
    target_speed->speed_x = speed * cos(angle_to_target);  // 前后
    target_speed->speed_y = speed * sin(angle_to_target);  // 左右
}
void xy_Update_by_target_v(PID_xy *p, CAR_ATTITUDE *car, CAR_ATTITUDE *target_speed,float speed) {
    float errorx, errory, error,angle_to_target;
    // 计算误差
    errorx = p->target_x - car->x;
    errory = p->target_y - car->y;
    error = sqrt(errorx * errorx + errory * errory);
    angle_to_target = my_atan2(errory, errorx) - car->yaw;  // 目标方向与小车当前航向的夹角
    target_speed->speed_x = speed * cos(angle_to_target);  // 前后
    target_speed->speed_y = speed * sin(angle_to_target);  // 左右
}
// 小车姿态初始化函数
void Set_CAR_ATTITUDE(CAR_ATTITUDE *car, float x, float y, float yaw, float speed_x, float speed_y, float speed_w) {
    car->x = x;
    car->y = y;
    car->yaw = yaw;
    car->speed_x = speed_x;
    car->speed_y = speed_y;
    car->speed_w = speed_w;
}

void Set_TARGET_ATTITUDE(TARGET_ATTITUDE *target, float x, float y, float yaw, float speed_x, float speed_y, float speed_w, float v1, float v2, float v3, float v4, uint8 mode) {
    //targetx,targety,targetyaw,targetspeedx,targetspeedy,v1,v2,v3,v4,mode
    target->x = x;
    target->y = y;
    target->yaw = yaw;
    target->speed_x = speed_x;
    target->speed_y = speed_y;
    target->speed_w = speed_w;
    target->mode = mode;
    target->v_wheel1 = v1;
    target->v_wheel2 = v2;
    target->v_wheel3 = v3;
    target->v_wheel4 = v4;
}

// 编码器数据初始化与更新函数
void Encoder_Init_Data(ENCODER_DATA *p) {
    p->encode1_data = encoder_get_count(ENCODER_QUAD_1);
    p->encode2_data = encoder_get_count(ENCODER_QUAD_2);
    p->encode3_data = -encoder_get_count(ENCODER_QUAD_3);
    p->encode4_data = encoder_get_count(ENCODER_QUAD_4);

    p->encode1_data_prev_5ms = p->encode1_data;
    p->encode2_data_prev_5ms = p->encode2_data;
    p->encode3_data_prev_5ms = p->encode3_data;
    p->encode4_data_prev_5ms = p->encode4_data;

    p->encode1_delta_5ms = 0;
    p->encode2_delta_5ms = 0;
    p->encode3_delta_5ms = 0;
    p->encode4_delta_5ms = 0;
}

void Encoder_Update_5ms(ENCODER_DATA *p) {
    int16 c1 = encoder_get_count(ENCODER_QUAD_1);
    int16 c2 = encoder_get_count(ENCODER_QUAD_2);
    int16 c3 = encoder_get_count(ENCODER_QUAD_3);
    int16 c4 = encoder_get_count(ENCODER_QUAD_4);

    encoder_clear_count(ENCODER_QUAD_1);
    encoder_clear_count(ENCODER_QUAD_2);
    encoder_clear_count(ENCODER_QUAD_3);
    encoder_clear_count(ENCODER_QUAD_4);

    p->encode1_delta_5ms = c1;
    p->encode2_delta_5ms = c2;
    p->encode3_delta_5ms = -c3;
    p->encode4_delta_5ms = c4;
}

void calculate_motortarget_by_vxy(CAR_ATTITUDE *target_speed, float *out) {  // 根据目标x,y,w计算四个轮子的目标速度
    out[0] = target_speed->speed_x + target_speed->speed_y + target_speed->speed_w;
    out[2] = target_speed->speed_x - target_speed->speed_y + target_speed->speed_w;
    out[1] = target_speed->speed_x - target_speed->speed_y - target_speed->speed_w;
    out[3] = target_speed->speed_x + target_speed->speed_y - target_speed->speed_w;
}

void calculate_vehicle_coordinate_by_encode(CAR_ATTITUDE *car, ENCODER_DATA *p, float kx, float ky) {  // 根据四个轮子的编码器数据计算小车在全球坐标系下的坐标
    float dx_vehicle = (p->encode1_delta_5ms + p->encode3_delta_5ms + p->encode2_delta_5ms + p->encode4_delta_5ms)* kx;
    float dy_vehicle = (p->encode1_delta_5ms - p->encode3_delta_5ms - p->encode2_delta_5ms + p->encode4_delta_5ms)* ky;
    car->x += (dx_vehicle * cos(car->yaw) - dy_vehicle * sin(car->yaw)) * 0.001f ;
    car->y += (dx_vehicle * sin(car->yaw) + dy_vehicle * cos(car->yaw)) * 0.001f ;
}

void set_nevigate_target(TARGET_ATTITUDE *target) {  // 0:位移控制 1:角度控制 2:角度+位移控制 3:角速度控制 4:速度控制 5:控制指定轮 6:无控制
    float distance = 0.0f;
    switch (target->mode) {
        case 0:  // 位移控制
            pid_xy.target_x = target->x;
            pid_xy.target_y = target->y;
            break;
        case 1:  // 角度控制
            pid_angle.target = target->yaw;  // 使用目标角度
            break;
        case 2:  // 角度+位移控制
            pid_angle.target = target->yaw;  // 使用目标角度
            pid_xy.target_x = target->x;
            pid_xy.target_y = target->y;
            break;
        case 3:  // 角速度控制
            pid_w.target = target->speed_w;  // 直接使用目标角速度
            break;
        case 4:  // 速度控制
        case 5:  // 控制指定轮
        case 6:  // 无控制
            break;
        default:
            break;
    }
}