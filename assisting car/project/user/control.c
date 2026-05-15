#include "control.h"
car_control_state_enum Car_Control_State=CONTORL_NONE;
car_rotate_state_enum Rotate_State = ROTATE_START;
car_nevigate_state_enum Nevigate_State = NEVIGATE_START;
void rotate_to_yaw(float target_yaw,CAR_ATTITUDE *now_car, car_rotate_state_enum rotate_state){
    float yaw_error = target_yaw - now_car->yaw;// 计算误差
    if (rotate_state == ROTATE_START) {
        // 设置角度 PID 的目标值为误差
        pid_angle.target = target_yaw;
        Rotate_State = ROTATE_RUNNING;
    }
    // 将误差限制在 -180~180 度范围内
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;
    if (Rotate_State == ROTATE_RUNNING) {
        if (fabs(yaw_error) < 1.5f && now_car->speed_w < 10) { // 误差小于 1 度且速度小于 10 认为旋转完成
            Rotate_State = ROTATE_DONE;
            pid_w.target = 0.0f; // 停止旋转
        }
    }
}
void navigate_to_xy(float target_x, float target_y, CAR_ATTITUDE *now_car, car_nevigate_state_enum nevigate_state){
    float distance = sqrt((target_x - now_car->x) * (target_x - now_car->x) + (target_y - now_car->y) * (target_y - now_car->y));
    if (nevigate_state == NEVIGATE_START) {
        pid_xy.target_x = target_x;
        pid_xy.target_y = target_y;
        Nevigate_State = NEVIGATE_RUNNING;
    }
    if (Nevigate_State == NEVIGATE_RUNNING) {
        if (distance < 3.0f && fabs(now_car->speed_x) < 30 && fabs(now_car->speed_y) < 30) { // 距离小于 20 且速度小于 10 认为导航完成
            Nevigate_State = NEVIGATE_DONE;
        }
    }
}