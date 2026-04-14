#include "vofa.h"
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       解析 VOFA 无线数据包，格式为 PA123! PB-45! 等
// 参数说明       rb        循环缓冲区指针
//               channel1~4 各通道输出指针（A/B/C/D 对应 1/2/3/4）
// 返回参数       void
// 使用示例       vofa_data_analyze(&rb, &ch1, &ch2, &ch3, &ch4);
// 备注           格式：P + 通道字母(A-D) + 整数数值 + '!'
//               例：PA123!  PB-45!  PC0!
//-------------------------------------------------------------------------------------------------------------------
void vofa_data_analyze(ring_buffer *rb, float *channel1, float *channel2,
                       float *channel3, float *channel4)
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
            default:  break;
        }

        rb->tail = (j + 1) & (MAX_BUF_SIZE - 1);
        i        = rb->tail;
        end      = rb->head;
    }
}