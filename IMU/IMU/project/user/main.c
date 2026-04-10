#include "zf_common_headfile.h"
#include "quaternion.h"

#define LED1                        (IO_P52)
#define PIT                         (TIM0_PIT)
#define GYRO_LSB_PER_DPS   (32768.0f / 2000.0f)
#define RAD_PER_DEG        (3.1415f / 180.0f)
#define DT                 0.01f

static Quat q;
static Vec3 forward_body;
static int tick_count = 0;
static float roll_angle = 0.0f;
static float pitch_angle = 0.0f;
static float yaw_angle = 0.0f;
static float yaw_offset = 0.0f;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

void init_attitude(void) {
    q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
    forward_body.x = 1.0f; forward_body.y = 0.0f; forward_body.z = 0.0f;
}

void update_attitude(void) {
    float gx_raw, gy_raw, gz_raw;
    float gx_dps, gy_dps, gz_dps;
    float wx, wy, wz;
    Quat dq;
    Vec3 direction_pure;
    Quat temp_q; // 用于暂存四元数乘法结果，更安全

    imu660rb_get_gyro();
    gx_raw = (float)imu660rb_gyro_x;
    gy_raw = (float)imu660rb_gyro_y;
    gz_raw = (float)imu660rb_gyro_z;

    gx_dps = (gx_raw - gyro_bias_x) / (-GYRO_LSB_PER_DPS);
    gy_dps = (gy_raw - gyro_bias_y) / (-GYRO_LSB_PER_DPS);
    gz_dps = (gz_raw - gyro_bias_z) / (-GYRO_LSB_PER_DPS);

    wx = gx_dps * RAD_PER_DEG;
    wy = gy_dps * RAD_PER_DEG;
    wz = gz_dps * RAD_PER_DEG;

    // 使用指针传参更新四元数
    omega_to_dq(&dq, wx, wy, wz, DT);
    
    // 计算结果存放到 temp_q，再赋回给 q
    quat_mul(&temp_q, &q, &dq);
    q = temp_q;
    
    quat_normalize(&q, &q); // 我们在底层做了保护，这里原地修改是安全的

    rotate_vector_by_quat(&direction_pure, &q, &forward_body);  // 可选用
    quat_to_euler(&q, &roll_angle, &pitch_angle, &yaw_angle);
    yaw_angle += yaw_offset;
    tick_count++;
}

void pit_hanlder(void) {
    update_attitude();
}

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    while(1) {
        if(imu660rb_init())
            printf("\r\nimu660rb init error.");
        else
            break;
        gpio_toggle_level(LED1);
        system_delay_ms(300);
    }

    init_attitude();
    pit_ms_init(PIT, 10, pit_hanlder);

    while(1) {
        printf("\r\nroll: %6.2f, pitch: %6.2f, yaw: %6.2f", roll_angle, pitch_angle, yaw_angle);
        gpio_toggle_level(LED1);
        system_delay_ms(300);
    }
}