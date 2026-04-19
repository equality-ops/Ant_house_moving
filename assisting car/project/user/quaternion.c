#include "quaternion.h"
#include <math.h>
#define M_PI 3.1415926f
Kalman1D kf_gx, kf_gy, kf_gz;
void Kalman1D_Init(Kalman1D *k, float Q, float R, float x0, float P0) {
    k->Q = Q;
    k->R = R;
    k->x = x0;
    k->P = P0;
}

float Kalman1D_Update(Kalman1D *k, float z) {
    float K;
    // 预测
    k->P = k->P + k->Q;
    // 计算卡尔曼增益
    K = k->P / (k->P + k->R);
    // 更新估计值
    k->x = k->x + K * (z - k->x);
    // 更新协方差
    k->P = (1.0f - K) * k->P;
    return k->x;
}

void init_attitude(void) {
    Quat q;
    quat_identity(&q);
}
// 单位四元数
void quat_identity(Quat *q)
{
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}
// 角速度 → dq
void omega_to_dq(float wx, float wy, float wz, float dt, Quat *dq)
{
    float half_dt;

    half_dt = 0.5f * dt;

    dq->w = 1.0f;
    dq->x = wx * half_dt;
    dq->y = wy * half_dt;
    dq->z = wz * half_dt;
}

// 四元数乘法
void quat_mul(const Quat *q1, const Quat *q2, Quat *out)
{
    float w, x, y, z;
    w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    out->w = w;
    out->x = x;
    out->y = y;
    out->z = z;
}

// 归一化
void quat_normalize(Quat *q)
{
    float norm;

    norm = q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z;

    if(norm > 0.0f)
    {
        norm = 1.0f / sqrt(norm);

        q->w *= norm;
        q->x *= norm;
        q->y *= norm;
        q->z *= norm;
    }
}
#include <math.h>

float my_atan2(float y, float x) {
    if (x > 0) {
        return atan(y / x);
    } 
    else if (x < 0) {
        if (y >= 0)
            return atan(y / x) + M_PI;   // 第二象限
        else
            return atan(y / x) - M_PI;   // 第三象限
    } 
    else { // x == 0
        if (y > 0)
            return M_PI / 2;
        else if (y < 0)
            return -M_PI / 2;
        else
            return 0;  // 未定义，按需返回 0
    }
}
// 转欧拉角（已简化）
void quat_to_euler(const Quat *q, float *roll, float *pitch, float *yaw)
{
    float sinr, cosr;
    float sinp;
    float siny, cosy;

    // roll
    sinr = 2.0f * (q->w * q->x + q->y * q->z);
    cosr = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    if (cosr == 0.0f && sinr > 0.0f)
        *roll = 90.0f;
    else if (cosr == 0.0f && sinr < 0.0f)
        *roll = -90.0f;
    else

        
    *roll = my_atan2(sinr, cosr);

    // pitch
    sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asin(sinp);

    // yaw
    siny = 2.0f * (q->w * q->z + q->x * q->y);
    cosy = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = my_atan2(siny, cosy);
}

// 向量旋转（可选）
void rotate_vector_by_quat(const Quat *q, const Vec3 *v, Vec3 *out)
{
    float tx, ty, tz;

    tx = 2.0f * (q->y * v->z - q->z * v->y);
    ty = 2.0f * (q->z * v->x - q->x * v->z);
    tz = 2.0f * (q->x * v->y - q->y * v->x);

    out->x = v->x + q->w * tx + (q->y * tz - q->z * ty);
    out->y = v->y + q->w * ty + (q->z * tx - q->x * tz);
    out->z = v->z + q->w * tz + (q->x * ty - q->y * tx);
}