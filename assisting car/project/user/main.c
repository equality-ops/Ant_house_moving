#include "zf_common_headfile.h"
#include "quaternion.h"
#include "uart_wireless.h"
#include "else.h"
#include "motor.h"
//初始化变量
uint8 key1_status = 1;//按键状态
uint8 key_start_status = 1; //run按钮状态
// IMU相关变量
static Quat q;//全局四元数
static Vec3 forward_body;
static float roll_angle = 0.0;
static float pitch_angle = 0.0;
//static float yaw_angle = 0.0;
float gyro_bias_x = 0.0, gyro_bias_y = 0.0, gyro_bias_z = 0.0;
//目标
int16 camera_erro=0;
uint8 target_side = 2; // 0:左，1:中，2:右
uint16 target_x_or_y = 0; // 目标距离
int counter=0;

//小车姿态(包含坐标航向角，速度)
CAR_ATTITUDE car;
CAR_ATTITUDE Target_Speed;
TARGET_ATTITUDE Nevigate_Target;//导航模式 0:位移控制 1:角度控制 2:角度+位移控制 3:角速度控制 4:速度控制
//中断处理函数

void uart_rx_interrupt_handler (uint8 dat)// UART 接收中断处理函数
{
    uart_query_byte(UART_INDEX, &dat);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &dat, 1);                           // 将数据写入 fifo 中
}

void uart_handler (void)// UART 定时数据处理函数
{
    fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
    if(fifo_data_count != 0)                                                // 读取到数据了
    {
        // 为了防止在读取FIFO的时候，又写入FIFO，这里关闭总中断。
        interrupt_global_disable();
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
        interrupt_global_enable();
        rb_write_q(&uart_rx_buffer, fifo_get_data, fifo_data_count); // 将读出的数据写入环形缓冲区
        analyze_uart_data(&uart_rx_buffer,&camera_erro,&uart_analyze_flag); // 分析数据
    }
}
void wireless_handler(void) {// Wireless 定时数据处理函数
    data_len = wireless_uart_read_buffer(data_buffer, 32);                    		// 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
    if(data_len != 0)                                                       		// 收到了消息 读取函数会返回实际读取到的数据个数
    {
        rb_write_q(&wireless_rx_buffer, data_buffer, data_len); // 将收到的数据写入环形缓冲区
        analyze_wireless_data(&wireless_rx_buffer,&target_side,&target_x_or_y,&wireless_analyze_state); // 分析数据
    }
}

void imu_handler(void) {//IMU中断
    float gx_raw, gy_raw, gz_raw;
    float gx_dps, gy_dps, gz_dps;
    float gx_f, gy_f, gz_f;//滤波后的陀螺仪数据
    float wx, wy, wz;
    Quat dq;
    Quat temp;
    Vec3 direction_pure;
    imu660rb_get_acc(); 
    imu660rb_get_gyro();
    gx_raw = (float)imu660rb_gyro_x;
    gy_raw = (float)imu660rb_gyro_y;
    gz_raw = (float)imu660rb_gyro_z;

    // 卡尔曼滤波
    gx_f = Kalman1D_Update(&kf_gx, gx_raw);
    gy_f = Kalman1D_Update(&kf_gy, gy_raw);
    gz_f = Kalman1D_Update(&kf_gz, gz_raw);

    // 转换为 dps
    gx_dps = (gx_f - gyro_bias_x) / (-GYRO_LSB_PER_DPS)*1.1457297;
    gy_dps = (gy_f - gyro_bias_y) / (-GYRO_LSB_PER_DPS)*1.1457297;
    gz_dps = (gz_f - gyro_bias_z) / (-GYRO_LSB_PER_DPS)*1.1457297;

    car.speed_w = gz_dps;//陀螺仪角速度乘以比例系数得到小车的角速度位姿存入car.speed_w

    wx = gx_dps * RAD_PER_DEG;
    wy = gy_dps * RAD_PER_DEG;
    wz = gz_dps * RAD_PER_DEG;
    // 计算 dq
    omega_to_dq(wx, wy, wz, DT, &dq);
    
    // q = q * dq
    quat_mul(&q, &dq, &temp);
    //printf("%f,%f,%f,%f\r\n",q.w,q.x,q.y,q.z);
    // 拷贝回去
    q = temp;
    
    // 归一化
    quat_normalize(&q);
    
    // 欧拉角
    quat_to_euler(&q,&roll_angle,&pitch_angle,&car.yaw);//通过陀螺仪累加得到小车航向角位姿存入car.yaw
    rotate_vector_by_quat(&q,&forward_body, &direction_pure);
    if (Nevigate_Target.mode<=4)
        Target_Speed.speed_w=w_PID_Update(&pid_w,car.speed_w);//角速度环，输出目标角速度
    /*
    if (counter==10){
        printf("%f,%f,%f,%f\r\n",car.yaw,gx_f,gy_f,gz_f);
        counter=0;
    }
    */
    counter++;
}
void xy_angle_handler(void){
    switch(Nevigate_Target.mode){
        case 0:
            xy_PID_Update(&pid_xy,&car,&Target_Speed);//pid，将目标速度输出赋予Target_Speed
            pid_w.target=angle_PID_Update(&pid_angle,car.yaw*57.32484);//角度环，输出目标角速度
            break;
        case 1:
            pid_w.target=angle_PID_Update(&pid_angle,car.yaw*57.32484);//角度环，输出目标角速度
            break;
        case 2:
            xy_PID_Update(&pid_xy,&car,&Target_Speed);//pid，将目标速度输出赋予Target_Speed
            pid_w.target=angle_PID_Update(&pid_angle,car.yaw*57.32484);//角度环，输出目标角速度      
            break;
        case 3://角速度控制
        case 4://纯速度控制
        case 5://轮速控制
        case 6:
            break;
    }
}
void motor_handler(void) {//电机控制定时器中断
    //int pid1_result, pid2_result, pid3_result, pid4_result;
    float xy_target[4];
    Encoder_Update_5ms(&encoder_data);//更新编码器数据
    calculate_vehicle_coordinate_by_encode(&car,&encoder_data,1.0f,1.0f);//根据编码器数据计算小车在全局坐标系下的坐标,改变小车的位姿car
    if (Nevigate_Target.mode==5){
        xy_target[0]=Nevigate_Target.v_wheel1;
        xy_target[1]=Nevigate_Target.v_wheel2;
        xy_target[2]=Nevigate_Target.v_wheel3;
        xy_target[3]=Nevigate_Target.v_wheel4;
    }
    else
        calculate_motortarget_by_vxy(&Target_Speed,xy_target);//Target_Speed包含目标x,y,w，根据目标x,y,w计算四个轮子的目标速度存入xy_target
    
    //pid电机控制
    
    pid1.target= xy_target[0];
    pid2.target= xy_target[1];
    pid3.target= xy_target[2];
    pid4.target= xy_target[3];

    /*
    pid1.target=600;
    pid2.target=600;
    pid3.target=600;
    pid4.target=600;
    */
    
    set_motor_pwm(0,(int)motor_PID_Update(&pid1,encoder_data.encode1_delta_5ms));
    set_motor_pwm(1,(int)motor_PID_Update(&pid2,encoder_data.encode2_delta_5ms));
    set_motor_pwm(2,(int)motor_PID_Update(&pid3,encoder_data.encode3_delta_5ms));
    set_motor_pwm(3,(int)motor_PID_Update(&pid4,encoder_data.encode4_delta_5ms));

}
void calibrate_gyro(int num_samples,int dt) {
    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int i;
    i=0;
    for (; i < num_samples; i++) {
        imu660rb_get_gyro();
        sum_x += (float)imu660rb_gyro_x;
        sum_y += (float)imu660rb_gyro_y;
        sum_z += (float)imu660rb_gyro_z;
        printf("%d\n\r",imu660rb_gyro_z);
        system_delay_ms(dt);
    }
    gyro_bias_x = sum_x / num_samples;
    gyro_bias_y = sum_y / num_samples;
    gyro_bias_z = sum_z / num_samples;
}


void all_init(void) {//初始化
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    //初始化按钮
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY_START, GPI, 1, GPI_PULL_UP);
    gpio_init(SWITCH1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(SWITCH2_PIN, GPI, 1, GPI_PULL_UP);

    //LED初始化
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    //蜂鸣器初始化
    gpio_init(BEEP_PIN, GPO, 0, GPO_PUSH_PULL);
    //kalman滤波器初始化
    Kalman1D_Init(&kf_gx, 0.02, 0.1, 0.0, 1.0);
    Kalman1D_Init(&kf_gy, 0.02, 0.1, 0.0, 1.0);
    Kalman1D_Init(&kf_gz, 0.02, 0.1, 0.0, 1.0);
    // UART 初始化
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    uart_write_string(UART_INDEX, "REDAY");                                // 输出测试信息
    uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    uart_write_byte(UART_INDEX, '\n');                                          // 输出换行
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE, uart_rx_interrupt_handler);        // 开启 UART_INDEX 的接收中断
    rb_init(&uart_rx_buffer, 64); // 初始化 UART 接收环形缓冲区
    if(wireless_uart_init())                                          // 判断是否通过初始化
    {
        while(1)                                                      // 初始化失败就在这进入死循环
        {
            beep_once(50);                                                // 发出提示音
            system_delay_ms(500);                                     
        }
    }
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("wireless ready\r\n");    // 初始化正常 输出测试信息
    rb_init(&wireless_rx_buffer, 64);
    //电机PWM 初始化
    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_1, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_2, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_3, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_4, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    //编码器初始化
    encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUAD_3, ENCODER_QUAD_3_CHA, ENCODER_QUAD_3_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUAD_4, ENCODER_QUAD_4_CHA, ENCODER_QUAD_4_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    Encoder_Init_Data(&encoder_data);
    // PID 初始化
    motor_PID_Init(&pid1, 10.0f, 2, 0.1f, 3000, 8000);
    motor_PID_Init(&pid2, 10.0f, 2, 0.1f, 3000, 8000);
    motor_PID_Init(&pid3, 10.0f, 2, 0.1f, 3000, 8000);
    motor_PID_Init(&pid4, 10.0f, 2, 0.1f, 3000, 8000);
    //ips200 初始化
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init();
    ips200_clear(RGB565_WHITE);
    // IMU 初始化
    while(1) {
        if(imu660rb_init())
            printf("\r\nimu660rb init error.");
        else
            gpio_set_level(BEEP_PIN, 0);
            break;
        gpio_toggle_level(LED1);
        system_delay_ms(300);
    }
    quat_identity(&q);
    Set_CAR_ATTITUDE(&car,0.0f, 0.0f, 0.0f,0.0f,0.0f,0.0f);
    Set_CAR_ATTITUDE(&Target_Speed,0.0f, 0.0f, 0.0f,0.0f,0.0f,0.0f);
    w_PID_Init(&pid_w,3,0,1,250,500);
    angle_PID_Init(&pid_angle,8,0.1,5,80,200);
    // EEPROM 初始化
    eeprom_init();
}

void main(void) {
    all_init();
    system_delay_ms(500);
    beep_once(100);
    // 菜单操作
    /*
    while (1) { 
        Keystroke_Menu();
        if (menu_over_flag == 1) { // 菜单操作完成
            break; // 退出菜单循环
        }
    }
    */
    //等待按下发车建
    key_start_status = gpio_get_level(KEY_START);
    while (key_start_status == 1) {
        key_start_status = gpio_get_level(KEY_START);
        system_delay_ms(50);
    }
    beep_once(100);
    calibrate_gyro(200,5);
    printf("%f\n",gyro_bias_z);
    beep_once(100);
    /*
    while (1) {
        system_delay_ms(100);
        printf("%f,%f,%f,%f,%f\r\n", pid1.Kp, pid1.Ki, pid1.Kd, pid1.output_max, pid1.integral_max);
    }
    //读取按钮1状态，进行电压检测
    
    key1_status = gpio_get_level(KEY1_PIN);
    if (key1_status ==0) {
        adc_init(ADC1_CH0_P10, ADC_12BIT);
        system_delay_ms(100);
        beep_once();
        system_delay_ms(200);
        beep_once();
        key1_status = 1;
        voltage_detect();
    }
    */
    pit_ms_init(PIT3, 5, imu_handler);//内环IMU中断
    pit_ms_init(PIT1, 100, wireless_handler);//wireless中断
    pit_ms_init(PIT2, 50, uart_handler);//uart中断
    
    pit_ms_init(PIT4, 5, motor_handler);//内环motor中断
    pit_ms_init(PIT5, 10, xy_angle_handler);//外环位置角度中断
    Set_TARGET_ATTITUDE(&Nevigate_Target,0,0,0,0,0,0,0,0,0,0,1);//targetx,targety,targetyaw,targetspeedx,targetspeedy,v1,v2,v3,v4,mode
    set_nevigate_target(&Nevigate_Target);
    while (1){
        system_delay_ms(50);
        //printf("%d,%d,%d,%d\r\n",encoder_data.encode1_delta_5ms,encoder_data.encode2_delta_5ms,encoder_data.encode3_delta_5ms,encoder_data.encode4_delta_5ms);
        printf("%f,%f,%f\r\n",car.yaw,car.speed_w,pid_w.target);
    }
    /*
    while (1) {
        system_delay_ms(100);
        //printf("%d,%d,%d,%d\r\n", encoder_data_dir_1,encoder_data_dir_2, encoder_data_dir_3, encoder_data_dir_4);
        if (uart_analyze_flag==1){
            uart_analyze_flag=0;
            printf("C_err: %d\r\n", camera_erro);
            uart_send_int16_to_chr(camera_erro);
            //beep_once(50);
        }
        if (wireless_analyze_state==1) {
            wireless_analyze_state=0;
            printf("Target side: %d, Target x or y: %d\r\n", target_side, target_x_or_y);
            wireless_uart_send_buffer((uint8 *)&target_side, 1);
            wireless_send_uint16_to_chr(target_x_or_y, 3);
            wireless_uart_send_string("\r\n");
            beep_once(50);
        }
    }
    */
    /*
    while(1) {
        if(imu660rb_init())
            printf("\r\nimu660rb init error.");
        else
            break;
        gpio_toggle_level(LED1);
        system_delay_ms(300);
    }
    init_attitude();
    calibrate_gyro();
    printf("\r\ngyro bias: x=%.2f, y=%.2f, z=%.2f", gyro_bias_x, gyro_bias_y, gyro_bias_z);
    pit_ms_init(PIT, 10, pit_hanlder);
    while(1) {
        printf("\r\nroll: %6.2f, pitch: %6.2f, yaw: %6.2f", roll_angle, pitch_angle, yaw_angle);
        gpio_toggle_level(LED1);
        system_delay_ms(300);
    }
    */

}