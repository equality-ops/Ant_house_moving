#ifndef _QUATERNION_H_
#define _QUATERNION_H_
#define LED1                        (IO_P52)
#define PIT                         (TIM0_PIT)
#define GYRO_LSB_PER_DPS   16.384f
#define RAD_PER_DEG        0.017453f// 3.1415926f/180.0f
#define DT                 0.01f//IMU中断时间
//ADC define
#define ADC_CHANNEL1            ( ADC1_CH0_P10 )
//button define
#define KEY1_PIN        IO_PB2
#define KEY2_PIN        IO_PB3
#define KEY3_PIN        IO_PB4
#define KEY4_PIN        IO_P32

#define SWITCH1_PIN     IO_PB0
#define SWITCH2_PIN     IO_PB1
//beep define
#define BEEP_PIN IO_P65
//uart define
#define UART_INDEX              ( UART_5   )                // 默认 UART_5
#define UART_BAUDRATE           ( 115200 )                  // 默认 115200
#define UART_TX_PIN             ( UART5_TX_P05 )            // 默认 UART5_TX_P05
#define UART_RX_PIN             ( UART5_RX_P04 )            // 默认 UART5_RX_P04
#define MAX_BUF_SIZE 128
typedef struct {
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差
    float x;      // 状态估计值
    float P;      // 估计误差协方差
} Kalman1D;

// 初始化卡尔曼滤波器
void Kalman1D_Init(Kalman1D *k, float Q, float R, float x0, float P0);

// 更新滤波器，输入测量值 z，返回滤波后的估计值
float Kalman1D_Update(Kalman1D *k, float z);


typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quat;

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

// 初始化
void quat_identity(Quat *q);

// dq计算
void omega_to_dq(float wx, float wy, float wz, float dt, Quat *dq);

// 四元数乘法
void quat_mul(const Quat *q1, const Quat *q2, Quat *out);

// 归一化
void quat_normalize(Quat *q);

// 转欧拉角
void quat_to_euler(const Quat *q, float *roll, float *pitch, float *yaw);

// 向量旋转
void rotate_vector_by_quat(const Quat *q, const Vec3 *v, Vec3 *out);

#endif