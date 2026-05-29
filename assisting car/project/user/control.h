#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "zf_common_headfile.h"
#include "quaternion.h"
#include "motor.h"
#include "else.h"
#include "uart_wireless.h"

#define ROTATE_w_BOOM 5.0f // 旋转完成的角度误差阈值（度）
#define NEVIGATE_V_LIMIT_HIGH 900.0f // 导航预热完成的速度阈值
#define NEVIGATE_V_LIMIT_MID 600.0f
#define NEVIGATE_V_LIMIT_LOW 400.0f
#define NEVIGATE_BOOM_level 0.95f//越小越快预热完成
#define OUTLINE_HIGHT 500.0f//出界高度
#define OUTLINE_WIDTH 800.0f//出界宽度
#define AVOID_DISTANCE 100.0f
typedef enum
{
    CONTORL_NONE  = 0,// 无控制
    CONTORL_WHEEL = 1,// 直接控制四个轮子的速度
    CONTORL_W = 2,// 直接控制角速度
    CONTORL_ANGLE = 3,// 直接控制角度
    CONTORL_NEVIGATE_TRACK_LINE = 4,// 直接控制x,y,同时控制角度保持当前角度
    CONTORL_SPEEDX_SPEEDY = 5,// 直接控制x,y速度
    CONTORL_NEVIGATE_SPEED_HIGH = 6,//直接控制指向目标点的速度
    CONTORL_NEVIGATE_SPEED_MID = 7,
    CONTORL_NEVIGATE_SPEED_LOW = 8
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
    NEVIGATE_BOOM = 1,
    NEVIGATE_RUNNING = 2,
    NEVIGATE_DONE = 3,
}car_nevigate_state_enum;
typedef enum
{
    FACING_UP=0,
    FACING_RIGHT=1,
    FACING_LEFT=3,
    FACING_DOWN=2,
}car_facing_direction_enum;
typedef enum{
    MENU_=0,
    INIT_=1,
    WAITING_=2,
    PLANNING_=3,
    TRACK_LINE_RUNNING=4,
    OUT_LINE_RUNNING=5,
    BACK_TO_LINE=6,
    END_=7
}CAR_STATE_ENUM;
typedef struct 
{
    float XY;
    float LENGTH;
}barrier;

extern CAR_STATE_ENUM WHOLE_STATE;
extern car_control_state_enum Car_Control_State;
extern car_rotate_state_enum Rotate_State;
extern car_nevigate_state_enum Nevigate_State;
extern float Controled_Nevigate_V;
extern BOOL RUNNING_IF_CLOCKWISE;
extern BOOL TRACKING_LINE;
extern car_facing_direction_enum car_direction;
extern float line_base;
extern float yaw_offset;
void rotate_to_yaw(float target_yaw,CAR_ATTITUDE *now_car, car_rotate_state_enum rotate_state);
void navigate_to_xy(float target_x, float target_y, CAR_ATTITUDE *now_car, car_nevigate_state_enum nevigate_state);
void nevigate(float target_yaw,float target_x,float target_y,CAR_ATTITUDE *car);
int waiting(BOOL* wireless_analyze_state);
int planning(SIDE_ENUM target_side,SIDE_ENUM now_side,CAR_ATTITUDE *car,uint16 target_x_or_y);
int tracking_line_running(CAR_ATTITUDE *car);
int out_line_running(CAR_ATTITUDE *car,SIDE_ENUM *now_side,SIDE_ENUM target_side,uint16 target_x_or_y);
int back_to_line(CAR_ATTITUDE *car,uint16 target_x_or_y);
float w_tracking_UPDATE(car_facing_direction_enum direction,float *offset,int erro,int erro2,CAR_ATTITUDE *car);
BOOL yorx_trackline_UPDATE(car_facing_direction_enum direction,float base,CAR_ATTITUDE *car,int erro,CAR_ATTITUDE *Target_Speed);
void tracking_and_nevigate(car_facing_direction_enum direction,float target_x,float target_y,CAR_ATTITUDE *car);
void track_and_avoid(car_facing_direction_enum direction,float target_x,float target_y,CAR_ATTITUDE *car);
void set_barr(SIDE_ENUM barr_side,float xy,float length);
#endif