#ifndef UART_WIRELESS_H
#define UART_WIRELESS_H
#include "zf_common_headfile.h"
// UART 和 Wireless 相关函数声明

//uart define
#define UART_INDEX              ( UART_5   )                // 默认 UART_5
#define UART_BAUDRATE           ( 115200 )                  // 默认 115200
#define UART_TX_PIN             ( UART5_TX_P05 )            // 默认 UART5_TX_P05
#define UART_RX_PIN             ( UART5_RX_P04 )            // 默认 UART5_RX_P04
#define MAX_BUF_SIZE 128
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
uint8 wireless_analyze_state=0;

//环形缓冲区函数
void rb_init(ring_buffer *rb,uint8 length);
void rb_write_one(ring_buffer *rb, int8 dat);
uint8 rb_read_one(ring_buffer *rb, uint8 *dat);
uint8 rb_idx_to_head_length(const ring_buffer *rb,uint8 idx);
uint8 rb_move(const ring_buffer *rb, uint8 idx,uint8 l);
void rb_write(ring_buffer *rb, uint8 *dat, uint8 length);
void rb_write_q(ring_buffer *rb, uint8 *dat, uint8 length);
//uart define
void analyze_uart_data(ring_buffer *rb,int16* camera_erro,uint8* uart_analyze_flag);
void uart_send(const uint8 *dat,uint8 length);
void uart_send_int16_to_chr(int16 dat);
//wireless define
void wireless_send_uint16_to_chr(uint16 dat, uint8 length);
void analyze_wireless_data(ring_buffer *rb,uint8* target_side,uint16* target_x_or_y,uint8* wireless_analyze_state );

#endif