#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "zf_common_headfile.h"
#include "quaternion.h"
#include "motor.h"
#define ROTATE_w_BOOM 10.0f // 旋转完成的角度误差阈值（度）
typedef enum
{
    CONTORL_NONE  = 0,
    CONTORL_WHEEL = 1,
    CONTORL_W = 2,
    CONTORL_ANGLE = 3,
    CONTORL_XY = 4,
    CONTORL_XY_SPEED = 5
}car_control_state_enum;
typedef enum
{
    ROTATE_START  = 0,
    ROTATE_RUNNING = 1,
    ROTATE_DONE = 2,
}car_rotate_state_enum;
typedef enum
{
    NEVIGATE_START  = 0,
    NEVIGATE_RUNNING = 1,
    NEVIGATE_DONE = 2,
}car_nevigate_state_enum;
extern car_control_state_enum Car_Control_State;
extern car_rotate_state_enum Rotate_State;
extern car_nevigate_state_enum Nevigate_State;
void rotate_to_yaw(float target_yaw,CAR_ATTITUDE *now_car, car_rotate_state_enum rotate_state);
void navigate_to_xy(float target_x, float target_y, CAR_ATTITUDE *now_car, car_nevigate_state_enum nevigate_state);
#endif