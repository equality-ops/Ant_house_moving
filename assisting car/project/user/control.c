#include "control.h"
car_control_state_enum Car_Control_State=CONTORL_NONE;
car_rotate_state_enum Rotate_State = ROTATE_START;
car_nevigate_state_enum Nevigate_State = NEVIGATE_START;
float Controled_Nevigate_V=0;
CAR_STATE_ENUM WHOLE_STATE=MENU_;
BOOL RUNNING_IF_CLOCKWISE=TRUE;
BOOL RUNNING_TOWARD_OUTLINE=FALSE;
int planning_points_num=0;
int planning_outline_points_num=0;
CAR_ATTITUDE PLANNING_POINTS[5];
CAR_ATTITUDE PLANNING_OUTLINE_POINTS[4];
CAR_ATTITUDE PLANNING_BACK_POINT;
//                           st    ld       l                    lu                  u                          ru                         r                          rd                d             ed
float TURNING_POINT[10][2]={{0,0},{0,0},{area_HEIGHT/2.0f,0},{area_HEIGHT,0},{area_HEIGHT,area_WIDTH/2.0f},{area_HEIGHT,area_WIDTH},{area_HEIGHT/2.0f,area_WIDTH},{0,area_WIDTH},{0,area_WIDTH/2.0f},{0,0}};
void rotate_to_yaw(float target_yaw,CAR_ATTITUDE *now_car, car_rotate_state_enum rotate_state){
    float yaw_error = target_yaw - now_car->yaw*57.29578f;// 计算误差
    Car_Control_State=CONTORL_ANGLE;
    if (rotate_state == ROTATE_START) {
        // 设置角度 PID 的目标值为误差
        pid_angle.target = target_yaw;
        Rotate_State = ROTATE_RUNNING;
    }
    // 将误差限制在 -180~180 度范围内
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;
    if (Rotate_State == ROTATE_RUNNING) {
        if (fabs(yaw_error) < 0.8f && now_car->speed_w < 10.0f) { // 误差小于 1 度且速度小于 15 认为旋转完成
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
        Controled_Nevigate_V=1;
        /*
        if (distance > 250.0f){
            Nevigate_State = NEVIGATE_BOOM; // 距离过大先预热启动
            Controled_Nevigate_V_num=0;
        }
        else 
        */
        Nevigate_State = NEVIGATE_RUNNING; // 距离小于250认为直接进入运行状态
    }
    /*
    if (Nevigate_State == NEVIGATE_BOOM) {
        Car_Control_State=CONTORL_NEVIGATE_SPEED;
        if (Controled_Nevigate_V_num > 13 || distance < 250.0f) { // 速度大于 30 认为预热完成
            Nevigate_State = NEVIGATE_RUNNING;
        }
    }*/
    if (Nevigate_State == NEVIGATE_RUNNING) {
        Car_Control_State=CONTORL_NEVIGATE_SPEED;
        if (distance < 3.0f && fabs(now_car->speed_x) < 30 && fabs(now_car->speed_y) < 30) { // 距离小于 3 且速度小于 30 认为导航完成
            Nevigate_State = NEVIGATE_DONE;
            pid_xy.output_max=NEVIGATE_V_LIMIT;//恢复正常速度限制
        }
    }
}
void nevigate(float target_yaw,float target_x,float target_y,CAR_ATTITUDE *car){
    if (fabs(target_yaw-car->yaw*57.29578f)>2.0f){
        Rotate_State=ROTATE_START;
        while (Rotate_State!=ROTATE_DONE){
            system_delay_ms(30);
            rotate_to_yaw(target_yaw, car,Rotate_State);
        }
        beep_once(100);
    }
    Nevigate_State=NEVIGATE_START;
    while (Nevigate_State!=NEVIGATE_DONE){
        navigate_to_xy(target_x, target_y, car,Nevigate_State);
        system_delay_ms(30);
    }
    beep_once(100);
    system_delay_ms(100);
}
int waiting(BOOL* wireless_analyze_state){
    planning_points_num=0;
    planning_outline_points_num=0;
     // 等待无线数据，直到收到数据后返回 1 进入下一状态
    if (*wireless_analyze_state==TRUE) {
        *wireless_analyze_state=FALSE;
        return 1; // 收到无线数据，进入下一状态
    }
    return 0; // 继续等待
}
float calculate_location(SIDE_ENUM side,int target_x_or_y){
    if (side==SIDE_START || side==SIDE_END)return 0;
    switch (side){
        case SIDE_LEFT:
            return target_x_or_y;
        case SIDE_UP:
            return area_HEIGHT+target_x_or_y;
        case SIDE_RIGHT:
            return area_HEIGHT*2+area_WIDTH-target_x_or_y;
        case SIDE_DOWN:
            return area_HEIGHT*2+area_WIDTH*2-target_x_or_y;
    }
    return 0;
}
void calculate_target_point(float target_x_or_y,SIDE_ENUM side,CAR_ATTITUDE *out0,CAR_ATTITUDE *out1,CAR_ATTITUDE *out2,CAR_ATTITUDE *back){
    switch (side){
        case SIDE_LEFT:
            out2->x=target_x_or_y;
            out2->y=-OUTLINE_HIGHT;
            out2->yaw=-90;
            if(RUNNING_IF_CLOCKWISE){
                out1->x=max(target_x_or_y-OUTLINE_WIDTH/2.0f,0);
                out1->y=-OUTLINE_HIGHT;
                out1->yaw=180;
            }
            else {
                out1->x=min(target_x_or_y+OUTLINE_WIDTH/2.0f,area_HEIGHT);
                out1->y=-OUTLINE_HIGHT;
                out1->yaw=180;
            }
            out0->x=out1->x;
            out0->y=0;
            out0->yaw=180;
            back->x=out2->x;
            back->y=0;
            back->yaw=180;
            break;
        case SIDE_UP:
            out2->x=area_HEIGHT+OUTLINE_HIGHT;
            out2->y=target_x_or_y;
            out2->yaw=0;
            if(RUNNING_IF_CLOCKWISE){
                out1->x=area_HEIGHT+OUTLINE_HIGHT;
                out1->y=max(target_x_or_y-OUTLINE_WIDTH/2.0f,0);
                out1->yaw=-90;
            }
            else {
                out1->x=area_HEIGHT+OUTLINE_HIGHT;
                out1->y=min(target_x_or_y+OUTLINE_HIGHT/2.0f,area_WIDTH);
                out1->yaw=-90;
            }
            out0->x=area_HEIGHT;
            out0->y=out1->y;
            out0->yaw=-90;
            back->x=area_HEIGHT;
            back->y=out2->y;
            back->yaw=-90;
            break;
        case SIDE_RIGHT:
            out2->x=target_x_or_y;
            out2->y=area_WIDTH+OUTLINE_HIGHT;
            out2->yaw=90;
            if(RUNNING_IF_CLOCKWISE){
                out1->x=min(target_x_or_y+OUTLINE_WIDTH/2.0f,area_HEIGHT);
                out1->y=area_WIDTH+OUTLINE_HIGHT;
                out1->yaw=0;
            }
            else {
                out1->x=max(target_x_or_y-OUTLINE_WIDTH/2.0f,0);
                out1->y=area_WIDTH+OUTLINE_HIGHT;
                out1->yaw=0;
            }
            out0->x=out1->x;
            out0->y=area_WIDTH;
            out0->yaw=0;
            back->x=out2->x;
            back->y=area_WIDTH;
            back->yaw=0;
            break;
        case SIDE_DOWN:
            out2->x=-OUTLINE_HIGHT;
            out2->y=target_x_or_y;
            out2->yaw=180;
            if(RUNNING_IF_CLOCKWISE){
                out1->x=-OUTLINE_HIGHT;
                out1->y=min(target_x_or_y+OUTLINE_HIGHT/2.0f,area_WIDTH);
                out1->yaw=90;
            }
            else {
                out1->x=-OUTLINE_HIGHT;
                out1->y=max(target_x_or_y-OUTLINE_HIGHT/2.0f,0);
                out1->yaw=90;
            }
            out0->x=0;
            out0->y=out1->y;
            out0->yaw=90;
            back->x=0;
            back->y=out2->y;
            back->yaw=90;
            break;
    }

}
BOOL planning_direction(SIDE_ENUM target_side,SIDE_ENUM now_side,CAR_ATTITUDE *car,uint16 target_x_or_y){
    uint16 now_location,target_location,target_now_x_or_y;
    if (now_side==SIDE_LEFT||now_side==SIDE_RIGHT)
        target_now_x_or_y = (int)car->x;
    else
        target_now_x_or_y = (int)car->y;
    if (target_x_or_y==6660) 
        if (target_side==SIDE_LEFT||target_side==SIDE_RIGHT)
            target_x_or_y=area_HEIGHT/2;
        else
            target_x_or_y=area_WIDTH/2;
    target_location = calculate_location(target_side,target_x_or_y);
    now_location = calculate_location(now_side, target_now_x_or_y);
    if (target_location >= now_location) {
        if(target_location-now_location>area_HEIGHT+area_WIDTH) return FALSE;//应该逆时针
        else return TRUE;//应该顺时针
    }
    else if (target_location < now_location) {
        if(now_location-target_location>area_HEIGHT+area_WIDTH) return TRUE;//应该顺时针
        else return FALSE;//应该逆时针
    }
    return TRUE;//其他情况认为方向错误
}
/*
BOOL if_near_turning_point(SIDE_ENUM target_side,uint16 target_x_or_y){//判断目标点是否在转弯点附近
    SIDE_ENUM turning_point_index;
    if (RUNNING_IF_CLOCKWISE) turning_point_index=target_side-1;
    else turning_point_index=target_side+1;
    if (target_side==SIDE_LEFT||target_side==SIDE_RIGHT)
        if (abs(TURNING_POINT[turning_point_index][0]-target_x_or_y)<OUTLINE_WIDTH/2.0f) return TRUE;
        else return FALSE;
    else
        if (abs(TURNING_POINT[turning_point_index][1]-target_x_or_y)<OUTLINE_HIGHT/2.0f) return TRUE;
        else return FALSE;
}
*/
int planning(SIDE_ENUM target_side,SIDE_ENUM now_side,CAR_ATTITUDE *car,uint16 target_x_or_y){//planning会规划小车行驶方向(RUNNING_IF_CLOCKWISE顺或逆)和途径点(PLANNING_POINTS和PLANNING_OUTLINE_POINTS)
    SIDE_ENUM i=now_side;
    if (planning_direction(target_side, now_side, car, target_x_or_y)!=RUNNING_IF_CLOCKWISE) {
        if (RUNNING_IF_CLOCKWISE) {
            RUNNING_IF_CLOCKWISE=FALSE;
        }
        else {
            RUNNING_IF_CLOCKWISE=TRUE;
        }
    }
    if (target_side == now_side)return 3; // 已经在目标边，进入
    if(i==SIDE_START&&RUNNING_IF_CLOCKWISE)i+=2;
    while (target_side != i){
        if (i%2==1){
            PLANNING_POINTS[planning_points_num].x=TURNING_POINT[i][0];
            PLANNING_POINTS[planning_points_num].y=TURNING_POINT[i][1];
            printf("planning point: %f,%f\r\n",PLANNING_POINTS[planning_points_num].x,PLANNING_POINTS[planning_points_num].y);
            if (RUNNING_IF_CLOCKWISE) {
                switch (i){
                    case POINT_LEFT_DOWN:
                        PLANNING_POINTS[planning_points_num].yaw=90;
                        break;
                    case POINT_LEFT_UP:
                        PLANNING_POINTS[planning_points_num].yaw=180;
                        break;
                    case POINT_RIGHT_DOWN:
                        PLANNING_POINTS[planning_points_num].yaw=0;
                        break;
                    case POINT_RIGHT_UP:
                        PLANNING_POINTS[planning_points_num].yaw=-90;
                        break;
                }
            }
            else {
                switch (i){
                    case POINT_LEFT_DOWN:
                        PLANNING_POINTS[planning_points_num].yaw=180;
                        break;
                    case POINT_LEFT_UP:
                        PLANNING_POINTS[planning_points_num].yaw=-90.0f;
                        break;
                    case POINT_RIGHT_DOWN:
                        PLANNING_POINTS[planning_points_num].yaw=90;
                        break;
                    case POINT_RIGHT_UP:
                        PLANNING_POINTS[planning_points_num].yaw=0;
                        break;
                }
            }
            planning_points_num++;
        }
        if(RUNNING_IF_CLOCKWISE)i=(i+1)%9;
        else i=(i+8)%9;
    }
    /*
    calculate_target_point(target_x_or_y, target_side, &PLANNING_OUTLINE_POINTS[0], &PLANNING_OUTLINE_POINTS[1], &PLANNING_OUTLINE_POINTS[2], &PLANNING_BACK_POINT);
    planning_outline_points_num=3;
    if (PLANNING_POINTS[planning_points_num-1].x==PLANNING_OUTLINE_POINTS[0].x && PLANNING_POINTS[planning_points_num-1].y==PLANNING_OUTLINE_POINTS[0].y){
        RUNNING_TOWARD_OUTLINE=TRUE;
        PLANNING_OUTLINE_POINTS[0].yaw=PLANNING_POINTS[planning_points_num-1].yaw;
        PLANNING_OUTLINE_POINTS[1].yaw=PLANNING_POINTS[planning_points_num-1].yaw;
    }
    */
    return 1;
}
int planning_outline(SIDE_ENUM target_side,uint16 target_x_or_y){
    if (target_x_or_y==6660)return 0;//666代表只规划了边
    calculate_target_point(target_x_or_y, target_side, &PLANNING_OUTLINE_POINTS[0], &PLANNING_OUTLINE_POINTS[1], &PLANNING_OUTLINE_POINTS[2], &PLANNING_BACK_POINT);
    planning_outline_points_num=3;
    if (PLANNING_POINTS[planning_points_num-1].x==PLANNING_OUTLINE_POINTS[0].x && PLANNING_POINTS[planning_points_num-1].y==PLANNING_OUTLINE_POINTS[0].y){
        RUNNING_TOWARD_OUTLINE=TRUE;
        PLANNING_OUTLINE_POINTS[0].yaw=PLANNING_POINTS[planning_points_num-1].yaw;
        PLANNING_OUTLINE_POINTS[1].yaw=PLANNING_POINTS[planning_points_num-1].yaw;
    }
    return 1;
}
int tracking_line_running(CAR_ATTITUDE *car){
    int i;
    for (i = 0; i < planning_points_num; i++) {
        nevigate(PLANNING_POINTS[i].yaw, PLANNING_POINTS[i].x, PLANNING_POINTS[i].y,car);
        beep_once(100);
    }
    return 1;
}
int out_line_running(CAR_ATTITUDE *car,SIDE_ENUM *now_side,SIDE_ENUM target_side,uint16 target_x_or_y){
    int i;
    if (!planning_outline(target_side, target_x_or_y)) return 0;//规划轮廓失败不进入
    for (i = 0; i < planning_outline_points_num; i++) {
        nevigate(PLANNING_OUTLINE_POINTS[i].yaw, PLANNING_OUTLINE_POINTS[i].x, PLANNING_OUTLINE_POINTS[i].y,car);
        beep_once(100);
    }
    *now_side=target_side;
    return 1;
}
int back_to_line(CAR_ATTITUDE *car,uint16 target_x_or_y){
    char str[20];
    if (!wireless_analyze_state) return 0;//未收到信号不返回
    func_int_to_str(str,target_x_or_y);
    wireless_uart_send_string(str);
    wireless_analyze_state=FALSE;
    if (target_x_or_y!=8880) return 0;//收到信号但数据错误不返回 
    nevigate(PLANNING_BACK_POINT.yaw, PLANNING_BACK_POINT.x, PLANNING_BACK_POINT.y,car);
    beep_once(100);
    return 1;
}