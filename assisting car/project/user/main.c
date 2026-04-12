#include "zf_common_headfile.h"
#include "quaternion.h"
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
typedef struct {
    uint8 buffer[MAX_BUF_SIZE];
    volatile uint8 head; 
    volatile uint8 tail;
    uint8 length;
} ring_buffer;



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
uint8 wirelesst_analyze_state=0;//wireless数据分析状态0:等待帧头 1:等待字母 2:等待数据 3:等待帧尾

//目标
uint16 camera_erro=0;
uint8 new_erro1=0;
uint8 new_erro2=0;
uint8 target_side = -1; // 0:左，1:中，2:右
uint16 target_x_or_y = 0; // 目标距离
//循环队列函数
void rb_init(ring_buffer *rb,uint8 length)
{
    rb->head = 0;
    rb->tail = 0;
    if (length > MAX_BUF_SIZE) {
        length = MAX_BUF_SIZE; // 如果长度超过缓冲区大小，取最大值
    }
    rb->length = length;// 设置环形缓冲区长度为2的次幂，方便取模运算
}
void rb_write_one(ring_buffer *rb, int8 dat)
{
    rb->buffer[rb->head] = dat;
    rb->head = (rb->head + 1) & (rb->length - 1);
    if (rb->head == rb->tail) {
        rb->tail = (rb->tail + 1) & (rb->length - 1); // 覆盖旧数据
    }
}
uint8 rb_read_one(ring_buffer *rb, uint8 *dat)
{
    if (rb->head == rb->tail) {
        return 0; // 缓冲区空，没有数据可读
    }
    *dat = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) & (rb->length - 1);
    return 1; // 成功读取一个字节
}
uint8 rb_idx_to_head_length(const ring_buffer *rb,uint8 idx) {
    return (rb->head - idx) & (rb->length - 1);
}
uint8 rb_move(const ring_buffer *rb, uint8 idx,uint8 l) {//idx移动l个位置
    return (idx + l) & (rb->length - 1);
}
void rb_write(ring_buffer *rb, uint8 *dat, uint8 length)
{
    uint8 i=0;
    uint8 l=(rb->head - rb->tail) & (rb->length - 1);
    if (length > rb->length-l) {// 如果写入长度超过缓冲区大小，取最后一段
        length = rb->length-l;
        i=length-rb->length;
    }
    for (; i < length; i++) {
        rb_write_one(rb, dat[i]);
    }
}
void rb_write_q(ring_buffer *rb, uint8 *dat, uint8 length){
    uint8 head = rb->head;
    uint8 tail = rb->tail;
    // 计算剩余空间
    uint8 free_space = (tail - head - 1) & (rb->length - 1);
    if (length > free_space)
        length = free_space;

    // 第一段：head 到 buffer末尾
    uint8 first_part = rb->length - head;
    if (first_part > length)
        first_part = length;
    memcpy(&rb->buffer[head], dat, first_part);
    // 第二段：从0开始
    memcpy(&rb->buffer[0], dat + first_part, length - first_part);
    rb->head = (head + length) & (rb->length - 1);
}
void analyze_uart_data(ring_buffer *rb) {
    while (rb_idx_to_head_length(rb, rb->tail)>3) {// 确保至少有4个字节可读（帧头+数据1+数据2+帧尾）
        uint8 dat;
        uint8 i=rb->tail; 
        rb_read_one(rb, &dat); // 读取一个字节
        if (dat != (uint8)0x23) { // 寻找帧头
            continue;
        }
        else {   // 找到帧头，检查帧尾
            i=rb_move(rb, i, 3); // 将移动到帧尾索引
            if (rb->buffer[i] == (uint8)0x21)
            { 
                new_erro1=rb->buffer[rb_move(rb, i, -2)]; // 读取数据1
                new_erro2=rb->buffer[rb_move(rb, i, -1)]; // 读取数据2
                if (new_erro1&0x01==0 && new_erro2&0x01==0) {
                    uint16 erro=0;
                    uint16 abs14 = ((uint16)(new_erro1 >> 1) << 7) | (new_erro2 >> 1);
                    camera_erro = (new_erro1 & 0x80) ? -(int16)abs14 : (int16)abs14;
                    rb->tail = rb_move(rb, i, 1); // 更新环形缓冲区的尾部索引，丢弃已处理数据（将tail移动到帧尾后一位）
                    uart_analyze_flag = 1; // 设置分析完成标志
                }
            }
        }
    }
}
void analyze_wireless_data(ring_buffer *rb) {//信号形式为 *A123!
    while (rb_idx_to_head_length(rb, rb->tail)>5) {// 确保至少有6个字节可读（帧头+字母+数据1+数据2+数据3+帧尾）
        uint8 dat;
        uint8 i=rb->tail;
        rb_read_one(rb, &dat); // 读取一个字节
        if (dat != (uint8)0x2A) { // 寻找帧头 '*'
            continue;
        }
        else {   // 找到帧头，检查帧尾
            i=rb_move(rb, i, 5); // 移动到帧尾索引
            if (rb->buffer[i] == (uint8)0x21)
            { 
                uint8 new_side=rb->buffer[rb_move(rb, i, -4)]-0x41; // 读取字母,A对应0左，B对应1前，C对应2右
                int16 dat1=0;
                int16 dat2=0;
                int16 dat3=0;
                uint16 xy;
                dat1=rb->buffer[rb_move(rb, i, -1)]-0x30;
                dat2=rb->buffer[rb_move(rb, i, -2)]-0x30;
                dat3=rb->buffer[rb_move(rb, i, -3)]-0x30;
                if (dat1>=0 && dat1<=9 && dat2>=0 && dat2<=9 && dat3>=0 && dat3<=9) {
                    target_side = new_side;
                    xy=dat1;
                    xy+=dat2*10;
                    xy+=dat3*100;
                    target_x_or_y=xy;
                    rb->tail = rb_move(rb, i, 1); // 更新环形缓冲区的尾部索引，丢弃已处理数据（将tail移动到帧尾后一位）
                    wirelesst_analyze_state=1; // 设置分析完成标志
                }
            }
        }
    }
}

//中断处理函数

// UART 接收中断处理函数
void uart_rx_interrupt_handler (uint8 dat)
{
    uart_query_byte(UART_INDEX, &dat);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &dat, 1);                           // 将数据写入 fifo 中
}
void uart_handler (void)
{
    fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
    if(fifo_data_count != 0)                                                // 读取到数据了
    {
        // 为了防止在读取FIFO的时候，又写入FIFO，这里关闭总中断。
        interrupt_global_disable();
        fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
        interrupt_global_enable();
        rb_write_q(&uart_rx_buffer, fifo_get_data, fifo_data_count); // 将读出的数据写入环形缓冲区
        analyze_uart_data(&uart_rx_buffer); // 分析数据
    }
}
void wireless_handler(void) {
    data_len = wireless_uart_read_buffer(data_buffer, 32);                    		// 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
    if(data_len != 0)                                                       		// 收到了消息 读取函数会返回实际读取到的数据个数
    {
        rb_write_q(&wireless_rx_buffer, data_buffer, data_len); // 将收到的数据写入环形缓冲区
        analyze_wireless_data(&wireless_rx_buffer); // 分析数据
    }
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
void wireless_send_uint16_to_chr(uint16 *dat,uint8 length){//
    char buff[length];
    while (length>0){
        *dat%10+0x30;
    }
    wireless_uart_send_buffer(dat, length); 
    wireless_uart_send_string("\r\n");
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
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");    // 初始化正常 输出测试信息
    //rb_init(&wireless_rx_buffer); // 初始化无线接收环形缓冲区
}
void main(void) {
    kalman_filter_init();
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
    pit_ms_init(PIT, 80, uart_handler);//uart中断
    pit_ms_init(PIT, 80, wireless_handler);//wireless中断
    while (1) {
        system_delay_ms(100);
        if (uart_analyze_flag==1){
            uart_analyze_flag=0;
            printf("Camera error: %d\r\n", camera_erro);
            uart_send((uint8 *)&camera_erro,2);
            beep_once();
        }
        if (wirelesst_analyze_state==1) {
            wirelesst_analyze_state=0;
            printf("Target side: %d, Target x or y: %d\r\n", target_side, target_x_or_y);
            char send_buffer[3];
            wireless_sendstr(send_buffer, 3);
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