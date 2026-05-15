#include "uart_wireless.h"
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
uint8 wireless_analyze_state=0;
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
    uint8 first_part;
    // 计算剩余空间
    uint8 free_space = (tail - head - 1) & (rb->length - 1);
    if (length > free_space)
        length = free_space;

    // 第一段：head 到 buffer末尾
    first_part = rb->length - head;
    if (first_part > length)
        first_part = length;
    memcpy(&rb->buffer[head], dat, first_part);
    // 第二段：从0开始
    memcpy(&rb->buffer[0], dat + first_part, length - first_part);
    rb->head = (head + length) & (rb->length - 1);
}
//uart函数
void analyze_uart_data(ring_buffer *rb,int16* camera_erro,uint8* uart_analyze_flag) {
    while (rb_idx_to_head_length(rb, rb->tail)>3) {// 确保至少有4个字节可读（帧头+数据1+数据2+帧尾）
        uint8 dat;
        uint8 i=rb->tail; 
        rb_read_one(rb, &dat); // 读取一个字节
        if (dat != (uint8)0x23) { // 寻找帧头#
            continue;
        }
        else {   // 找到帧头，检查帧尾!
            i=rb_move(rb, i, 3); // 将移动到帧尾索引
            if (rb->buffer[i] == (uint8)0x21)
            { 
                uint8 new_erro1=rb->buffer[rb_move(rb, i, -2)]; // 读取数据1
                uint8 new_erro2=rb->buffer[rb_move(rb, i, -1)]; // 读取数据2
                if ((new_erro1&0x01)==0 && (new_erro2&0x01)==0) {
                    uint16 abs14 = ((uint16)((new_erro1&0x7F) >> 1) << 7) | (new_erro2 >> 1);
                    *camera_erro = (new_erro1 & 0x80) ? -(int16)abs14 : (int16)abs14;
                    rb->tail = rb_move(rb, i, 1); // 更新环形缓冲区的尾部索引，丢弃已处理数据（将tail移动到帧尾后一位）
                    *uart_analyze_flag = 1; // 设置分析完成标志
                }
            }
        }
    }
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
//wireless函数
void analyze_wireless_data(ring_buffer *rb,uint8* target_side,uint16* target_x_or_y,uint8* wireless_analyze_state ) {
    //信号形式为 *A123!
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
                if (dat1>=0 && dat1<=9 && dat2>=0 && dat2<=9 && dat3>=0 && dat3<=9 && new_side<=2 && new_side>=0) {
                    *target_side = new_side;
                    xy=dat1;
                    xy+=dat2*10;
                    xy+=dat3*100;
                    *target_x_or_y=xy;
                    rb->tail = rb_move(rb, i, 1); // 更新环形缓冲区的尾部索引，丢弃已处理数据（将tail移动到帧尾后一位）
                    *wireless_analyze_state=1; // 设置分析完成标志
                }
            }
        }
    }
}
void wireless_send_uint16_to_chr(uint16 dat, uint8 length) {
    char buff[7];
    uint8 start = 0;
    uint8 end = length;  // 记录实际占用长度
    uint8 i;
    i = end;
    while (i > start) {
        i--;
        buff[i] = (char)(dat % 10 + 0x30);
        dat /= 10;
    }
    wireless_uart_send_buffer(buff, end);
    wireless_uart_send_string("\r\n");
}
void vofa_data_analyze(ring_buffer *rb, float *channel1, float *channel2,float *channel3, float *channel4,float *channel5, float *channel6)
{
    uint8 used = (rb->head - rb->tail + MAX_BUF_SIZE) & (MAX_BUF_SIZE - 1);
    uint8 i   = rb->tail;
    uint8 end = rb->head;
    uint8 ch_idx, ch, j;
    float float_value;
    if(used < 4) return;
    while(i != end)
    {
        uint8 neg     = 0;
        int   value   = 0;
        uint8 valid   = 0;
        uint8 has_dig = 0;

        if(rb->buffer[i] != 'P')
        {
            i = (i + 1) & (MAX_BUF_SIZE - 1);
            continue;
        }

        ch_idx = (i + 1) & (MAX_BUF_SIZE - 1);
        if(ch_idx == end) break;

        ch = rb->buffer[ch_idx];
        if(ch < 'A' || ch > 'D')
        {
            i = (i + 1) & (MAX_BUF_SIZE - 1);
            continue;
        }

        j = (ch_idx + 1) & (MAX_BUF_SIZE - 1);

        while(j != end)
        {
            uint8 c = rb->buffer[j];

            if(c == '!')
            {
                if(has_dig) valid = 1;
                break;
            }
            else if(c == '-' && j == ((ch_idx + 1) & (MAX_BUF_SIZE - 1)))
            {
                neg = 1;
            }
            else if(c >= '0' && c <= '9')
            {
                value   = value * 10 + (c - '0');
                has_dig = 1;
            }
            else
            {
                break;
            }

            j = (j + 1) & (MAX_BUF_SIZE - 1);
        }

        if(!valid)
        {
            i = (i + 1) & (MAX_BUF_SIZE - 1);
            continue;
        }

        if(neg) value = -value;
        float_value = (float)value / 10.0f;

        switch(ch)
        {
            case 'A': *channel1 = float_value; break;
            case 'B': *channel2 = float_value; break;
            case 'C': *channel3 = float_value; break;
            case 'D': *channel4 = float_value; break;
            case 'E': *channel5 = float_value; break;
            case 'F': *channel6 = float_value; break;
            default:  break;
        }

        rb->tail = (j + 1) & (MAX_BUF_SIZE - 1);
        i        = rb->tail;
        end      = rb->head;
    }
}