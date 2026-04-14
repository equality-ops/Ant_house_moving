#ifndef VOFA_H
#define VOFA_H
#include "uart_wireless.h"
extern void vofa_data_analyze(ring_buffer *rb, float *channel1, float *channel2,float *channel3, float *channel4);
#endif