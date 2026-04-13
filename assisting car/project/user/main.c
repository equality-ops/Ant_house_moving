#include "zf_common_headfile.h"
#include "quaternion.h"
#include "uart_wireless.h"
//初始化变量
uint8 key1_status = 1;//按键状态

// IMU相关变量
static Quat q;//全局四元数
static Vec3 forward_body;
static int tick_count = 0;
static float roll_angle = 0.0;
static float pitch_angle = 0.0;
static float yaw_angle = 0.0;
float gyro_bias_x = 0.0, gyro_bias_y = 0.0, gyro_bias_z = 0.0;
Kalman1D kf_gx, kf_gy, kf_gz;//陀螺仪卡尔曼滤波器

//uart相关变量
ring_buffer uart_rx_buffer;//uart接收环形缓冲区
uint8       uart_get_data[64] = {0};                        // 串口接收数据缓冲区
uint8       fifo_get_data[64] = {0};                        // fifo 输出读出缓冲区
uint8       last_rx_data_count = 0;
uint32      fifo_data_count = 0;                            // fifo 数据个数
fifo_struct uart_data_fifo = {0};
uint8 uart_analyze_flag = 0;//uart分析标志

//wireless相关变量
ring_buffer wireless_rx_buffer;//wireless接收环形缓冲区
uint8 data_buffer[32];
uint8 last_length = 0;
uint8 data_len;
uint8 count = 0;
uint8 wireless_analyze_state=0;//

//目标
int16 camera_erro=0;
uint8 target_side = 2; // 0:左，1:中，2:右
uint16 target_x_or_y = 0; // 目标距离


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
void update_attitude(void) {
    float gx_raw, gy_raw, gz_raw;
    float gx_f, gy_f, gz_f;
    float gx_dps, gy_dps, gz_dps;
    float wx, wy, wz;
    Quat dq;
    Quat temp;
    Vec3 direction_pure;
    imu660rb_get_acc(); 
    imu660rb_get_gyro();
    gx_raw = (float)imu660rb_gyro_x;
    gy_raw = (float)imu660rb_gyro_y;
    gz_raw = (float)imu660rb_gyro_z;

    // 转换为 dps
    gx_dps = (gx_raw - gyro_bias_x) / (-GYRO_LSB_PER_DPS);
    gy_dps = (gy_raw - gyro_bias_y) / (-GYRO_LSB_PER_DPS);
    gz_dps = (gz_raw - gyro_bias_z) / (-GYRO_LSB_PER_DPS);

    // 卡尔曼滤波
    gx_f = Kalman1D_Update(&kf_gx, gx_raw);
    gy_f = Kalman1D_Update(&kf_gy, gy_raw);
    gz_f = Kalman1D_Update(&kf_gz, gz_raw);
    wx = gx_dps * RAD_PER_DEG;
    wy = gy_dps * RAD_PER_DEG;
    wz = gz_dps * RAD_PER_DEG;
    // 计算 dq
    omega_to_dq(wx, wy, wz, DT, &dq);
    // q = q * dq
    quat_mul(&q, &dq, &temp);
    
    // 拷贝回去
    q = temp;
    // 归一化
    quat_normalize(&q);
    // 欧拉角
    quat_to_euler(&q,&roll_angle, &pitch_angle, &yaw_angle);
    rotate_vector_by_quat(&q,&forward_body, &direction_pure);
    tick_count++;
}
void pit_hanlder(void) {//IMU中断
    update_attitude();
}
// 电压检测
void voltage_detect()
{
    float voltage;
    int adc_data= adc_convert(ADC1_CH0_P10);
    voltage =  (float)11 * 3.3 * adc_data / 4095 ;
    if (voltage>11.1) {
        while(1) {
            gpio_toggle_level(LED1);
            system_delay_ms(3000);
            printf("Battery voltage is normal: %.2f\r\n", voltage);
        }
    }
    else
    {
        while(1) {
            gpio_toggle_level(LED1);
            system_delay_ms(3000);
            printf("Battery voltage is too low: %.2f\r\n", voltage);
        }
    }
}

void calibrate_gyro(void) {
    const int num_samples = 100;
    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int i;
    i=0;
    for (; i < num_samples; i++) {
        imu660rb_get_gyro();
        sum_x += (float)imu660rb_gyro_x;
        sum_y += (float)imu660rb_gyro_y;
        sum_z += (float)imu660rb_gyro_z;
        system_delay_ms(10);
    }
    gyro_bias_x = sum_x / num_samples;
    gyro_bias_y = sum_y / num_samples;
    gyro_bias_z = sum_z / num_samples;
}

void beep_once(void) {
    gpio_set_level(BEEP_PIN, 0);
    system_delay_ms(50);
    gpio_set_level(BEEP_PIN, 1);
    system_delay_ms(50);
    gpio_set_level(BEEP_PIN, 0);
}

void uart_send(const uint8 *dat,uint8 length){
    uint8 n_buffer[64];   //用于存储读取到的数据的临时缓冲区
    memcpy(n_buffer, dat, length);
    printf("%s\r\n", n_buffer);              
    uart_write_buffer(UART_INDEX, n_buffer, length); 
    uart_write_string(UART_INDEX, "\r\n");                 
}
void uart_send_int16_to_chr(int16 dat) {
    char buffer[6];  // int16 范围 -32768 ~ 32767，最多6字符（含负号和结尾\0）
    int16 num = dat;
    uint8 idx = 0;
    
    if (num < 0) {
        uart_write_byte(UART_INDEX, '-');
        num = -num;
    }
    
    // 生成数字字符（反向）
    do {
        buffer[idx++] = (num % 10) + '0';
        num /= 10;
    } while (num > 0);
    
    // 反向发送
    while (idx > 0) {
        uart_write_byte(UART_INDEX, buffer[--idx]);
    }
    uart_write_string(UART_INDEX, "\r\n");
}
void all_init(void) {//初始化
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    //初始化按钮
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(SWITCH1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(SWITCH2_PIN, GPI, 1, GPI_PULL_UP);

    //LED初始化
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    //蜂鸣器初始化
    gpio_init(BEEP_PIN, GPO, 1, GPO_PUSH_PULL);
    //kalman滤波器初始化
    Kalman1D_Init(&kf_gx, 0.01, 0.1, 0.0, 1.0);
    Kalman1D_Init(&kf_gy, 0.01, 0.1, 0.0, 1.0);
    Kalman1D_Init(&kf_gz, 0.01, 0.1, 0.0, 1.0);
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
            beep_once();                                                // 发出提示音
            system_delay_ms(1000);                                     
        }
    }
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("wireless ready\r\n");    // 初始化正常 输出测试信息
    rb_init(&wireless_rx_buffer, 64);
    //rb_init(&wireless_rx_buffer); // 初始化无线接收环形缓冲区
}
void main(void) {
    all_init();
    beep_once();
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
    pit_ms_init(PIT1, 100, wireless_handler);//wireless中断
    pit_ms_init(PIT2, 50, uart_handler);//uart中断
    while (1) {
        system_delay_ms(100);
        if (uart_analyze_flag==1){
            uart_analyze_flag=0;
            printf("C_err: %d\r\n", camera_erro);
            uart_send_int16_to_chr(camera_erro);
            beep_once();
        }
        if (wireless_analyze_state==1) {
            wireless_analyze_state=0;
            printf("Target side: %d, Target x or y: %d\r\n", target_side, target_x_or_y);
            wireless_uart_send_buffer((uint8 *)&target_side, 1);
            wireless_send_uint16_to_chr(target_x_or_y, 3);
            wireless_uart_send_string("\r\n");
            beep_once();
        }
    }
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