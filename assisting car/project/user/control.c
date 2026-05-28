#include "control.h"
//#include "uart_wireless.h"
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
BOOL TRACKING_LINE=FALSE;
float line_base=0;
int line_angle_count=0;
car_facing_direction_enum car_direction=FACING_UP;
float yaw_offset=0;
//                           st    ld       l                    lu                  u                          ru                         r                          rd                d             ed
float TURNING_POINT[10][2]={{0,0},{0,0},{area_HEIGHT/2.0f,0},{area_HEIGHT,0},{area_HEIGHT,area_WIDTH/2.0f},{area_HEIGHT,area_WIDTH},{area_HEIGHT/2.0f,area_WIDTH},{0,area_WIDTH},{0,area_WIDTH/2.0f},{0,0}};
//-------------------------------------------底层旋转惯导函数--------------------------------------------

void rotate_to_yaw(float target_yaw,CAR_ATTITUDE *now_car, car_rotate_state_enum rotate_state){
    float yaw_error = target_yaw - (now_car->yaw+yaw_offset)*Rad_to_Deg;
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
        if (fabs(yaw_error) < 0.8f && now_car->speed_w < 10.0f){
            Rotate_State = ROTATE_DONE;
            pid_w.target = 0.0f;
        }
    }
}
void navigate_to_xy(float target_x, float target_y, CAR_ATTITUDE *now_car, car_nevigate_state_enum nevigate_state){
    
    float distance = sqrt((target_x - now_car->x) * (target_x - now_car->x) + (target_y - now_car->y) * (target_y - now_car->y));
    car_nevigate_state_enum NEVIGATE_STATE_=CONTORL_NEVIGATE_SPEED_LOW;
    if (nevigate_state == NEVIGATE_START) {
        pid_xy.target_x = target_x;
        pid_xy.target_y = target_y;
        Controled_Nevigate_V=1;
        if (distance >= 600.0f)
            NEVIGATE_STATE_=CONTORL_NEVIGATE_SPEED_HIGH;
        else if (distance >= 250.0f)
            NEVIGATE_STATE_=CONTORL_NEVIGATE_SPEED_MID;
        Car_Control_State=NEVIGATE_STATE_;
        Nevigate_State = NEVIGATE_RUNNING;
    }
    if (Nevigate_State == NEVIGATE_RUNNING) {
        if (distance < 3.0f && fabs(now_car->speed_x) < 30 && fabs(now_car->speed_y) < 30) {
            Nevigate_State = NEVIGATE_DONE;
            pid_xy.output_max=NEVIGATE_V_LIMIT_HIGH;
        }
    }
}

//-------------------------------------------惯导函数--------------------------------------------

void nevigate(float target_yaw,float target_x,float target_y,CAR_ATTITUDE *car){//惯导
    if(fabs(target_yaw-car->yaw*Rad_to_Deg)>2.0f){
        Rotate_State=ROTATE_START;
        while (Rotate_State!=ROTATE_DONE){
            system_delay_ms(30);
            rotate_to_yaw(target_yaw, car,Rotate_State);
        }
        beep_once(50);
    }
    Nevigate_State=NEVIGATE_START;
    while (Nevigate_State!=NEVIGATE_DONE){
        navigate_to_xy(target_x, target_y, car,Nevigate_State);
        system_delay_ms(30);
    }
    beep_once(50);
    system_delay_ms(100);
}

void tracking_and_nevigate(car_facing_direction_enum direction,float target_x,float target_y,CAR_ATTITUDE *car){//惯导,会自动改变方向，并在途中巡线用于纠正坐标
    float target_yaw=0;
    int k=0;
    switch (direction)
    {
        case FACING_UP:
            line_base=target_y;
            break;
        case FACING_DOWN:
            target_yaw=180;
            line_base=target_y;
            break;
        case FACING_RIGHT:
            target_yaw=90;
            line_base=target_x;
            break;
        case FACING_LEFT:
            target_yaw=-90;
            line_base=target_x;
            break;
        default:
            break;
    }
    if(fabs(target_yaw-car->yaw*Rad_to_Deg)>2.0f){
        Rotate_State=ROTATE_START;
        while (Rotate_State!=ROTATE_DONE){
            system_delay_ms(30);
            rotate_to_yaw(target_yaw,car,Rotate_State);
        }
        beep_once(50);
        car_direction=direction;
    }
    TRACKING_LINE=TRUE;
    Nevigate_State=NEVIGATE_START;
    while (Nevigate_State!=NEVIGATE_DONE){
        navigate_to_xy(target_x, target_y, car,Nevigate_State);
        system_delay_ms(30);
    }
    TRACKING_LINE=FALSE;
    beep_once(50);
    system_delay_ms(100);
}


//-------------------------------------------路径规划相关函数--------------------------------------------


float calculate_location(SIDE_ENUM side,int target_x_or_y){//将二维坐标转化为一维环线上的一点
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
void calculate_target_point(float target_x_or_y,SIDE_ENUM side,CAR_ATTITUDE *out0,CAR_ATTITUDE *out1,CAR_ATTITUDE *out2,CAR_ATTITUDE *back){//计算在最终边的目标点位姿
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
BOOL planning_direction(SIDE_ENUM target_side,SIDE_ENUM now_side,CAR_ATTITUDE *car,uint16 target_x_or_y){//输入目标边、位置，现在的边、位置，输出应该顺/逆时针行进
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

//-------------------------------------------状态机相关函数--------------------------------------------
//WAITING状态机
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
//PLANNING状态机
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
    return 1;
}
//TRACK_LINE_RUNNING状态机
int tracking_line_running(CAR_ATTITUDE *car){
    int i;
    for (i = 0; i < planning_points_num; i++) {
        car_facing_direction_enum direction = ((int)(PLANNING_POINTS[i].yaw+360)/90)%4;
        tracking_and_nevigate(direction,PLANNING_POINTS[i].x,PLANNING_POINTS[i].y,car);
        //nevigate(PLANNING_POINTS[i].yaw, PLANNING_POINTS[i].x, PLANNING_POINTS[i].y,car);
    }
    return 1;
}

//OUT_LINE_RUNNING状态机
int planning_outline(SIDE_ENUM target_side,uint16 target_x_or_y){//规划线外惯导到达点，当收到666时返回0，持续等待
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
int out_line_running(CAR_ATTITUDE *car,SIDE_ENUM *now_side,SIDE_ENUM target_side,uint16 target_x_or_y){
    int i;
    if (!planning_outline(target_side, target_x_or_y)) return 0;//规划轮廓失败不进入
    if (planning_outline_points_num>0){
        car_facing_direction_enum direction = ((int)(PLANNING_OUTLINE_POINTS[0].yaw+360)/90)%4;
        tracking_and_nevigate(direction,PLANNING_OUTLINE_POINTS[0].x,PLANNING_OUTLINE_POINTS[0].y,car);
    }
    for (i = 1; i < planning_outline_points_num; i++) {
        car_facing_direction_enum direction = ((int)(PLANNING_OUTLINE_POINTS[0].yaw+360)/90)%4;
        car_direction=direction;
        switch (direction)
        {
            case FACING_UP:
                line_base=PLANNING_OUTLINE_POINTS[i].y;
                break;
            case FACING_DOWN:
                line_base=PLANNING_OUTLINE_POINTS[i].y;
                break;
            case FACING_RIGHT:
                line_base=PLANNING_OUTLINE_POINTS[i].x;
                break;
            case FACING_LEFT:
                line_base=PLANNING_OUTLINE_POINTS[i].x;
                break;
            default:
                break;
        }
        nevigate(PLANNING_OUTLINE_POINTS[i].yaw, PLANNING_OUTLINE_POINTS[i].x, PLANNING_OUTLINE_POINTS[i].y,car);
    }
    *now_side=target_side;
    return 1;
}

//BACK_TO_LINE状态机
int back_to_line(CAR_ATTITUDE *car,uint16 target_x_or_y){
    char str[20];
    car_facing_direction_enum direction = ((int)(PLANNING_BACK_POINT.yaw+360)/90)%4;
    if (!wireless_analyze_state) return 0;//未收到信号不返回
    func_int_to_str(str,target_x_or_y);
    wireless_uart_send_string(str);
    wireless_analyze_state=FALSE;
    if (target_x_or_y!=8880) return 0;//收到信号但数据错误不返回 
    car_direction=direction;//纠正车体角度
    switch (direction)
    {
        case FACING_UP:
            line_base=PLANNING_BACK_POINT.y;
            break;
        case FACING_DOWN:
            line_base=PLANNING_BACK_POINT.y;
            break;
        case FACING_RIGHT:
            line_base=PLANNING_BACK_POINT.x;
            break;
        case FACING_LEFT:
            line_base=PLANNING_BACK_POINT.x;
            break;
        default:
            break;
    }
    nevigate(PLANNING_BACK_POINT.yaw, PLANNING_BACK_POINT.x, PLANNING_BACK_POINT.y,car);
    return 1;
}

//-------------------------------------------巡线视觉纠正相关函数--------------------------------------------
BOOL yorx_trackline_UPDATE(car_facing_direction_enum direction,float base,CAR_ATTITUDE *car,int erro,CAR_ATTITUDE *Target_Speed){//通过控制车体y轴速度来巡线，在视觉误差小时纠正车体坐标
    float *x_or_y,error_=erro*0.4736;
    int output=0;
    if(direction==FACING_UP||direction==FACING_DOWN){
        if(direction==FACING_UP&&car->x>area_HEIGHT-280)
            return TRUE;
        if(direction==FACING_DOWN&&car->x<250)
            return TRUE;
        x_or_y=&car->y;
    }
    else{
        if(direction==FACING_RIGHT&&car->y>area_WIDTH-280)
            return TRUE;
        if(direction==FACING_LEFT&&car->y<280)
            return TRUE;
        x_or_y=&car->x;
    }
    if(direction==FACING_UP||direction==FACING_LEFT)
        error_=-error_;
    //printf("%d\n",erro);
    output=min(max(-550,erro*1.4),550);
    if(abs(erro)<15){
        *x_or_y=base;
        return FALSE;
    }
    *x_or_y=base+error_;
    Target_Speed->speed_y=output;
    return FALSE;
}
float w_tracking_UPDATE(car_facing_direction_enum direction,float *offset,int erro,int erro2,CAR_ATTITUDE *car){//巡线时当误差小时将角度用offset纠正，误差大时返回目标角速度辅助锁定正确方向
    float base=(direction+1)%4*1.570796-1.570796;//将目标方向转化为目标角度
    if(abs(erro)<10&&abs(erro2)<700){
        line_angle_count++;
        if (line_angle_count>4){
        beep_once(100);
        *offset=(base-car->yaw);
        line_angle_count=0;
        }
    }
    else
        line_angle_count=0;
    return erro/2;
}