#include "quaternion.h"

void quat_mul(Quat *result, Quat *q1, Quat *q2) {
    // 提前读取，防止 q1/q2 和 result 是同一个指针导致的数据覆盖
    float w1 = q1->w, x1 = q1->x, y1 = q1->y, z1 = q1->z;
    float w2 = q2->w, x2 = q2->x, y2 = q2->y, z2 = q2->z;

    // 拆解乘法，避免单行表达式过长
    float t_w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    float t_x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    float t_y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    float t_z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    result->w = t_w;
    result->x = t_x;
    result->y = t_y;
    result->z = t_z;
}

void quat_conj(Quat *result, Quat *q) {
    result->w = q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;
}

float quat_norm(Quat *q) {
    // 拆解平方和计算
    float ww = q->w * q->w;
    float xx = q->x * q->x;
    float yy = q->y * q->y;
    float zz = q->z * q->z;
    return sqrt(ww + xx + yy + zz);
}

void quat_normalize(Quat *result, Quat *q) {
    float n = quat_norm(q);
    float inv;
    float t_w, t_x, t_y, t_z;

    if (n < 1e-9f) {
        result->w = 1.0f;
        result->x = 0.0f;
        result->y = 0.0f;
        result->z = 0.0f;
        return;
    }
    
    inv = 1.0f / n;
    
    // 暂存结果，允许原地修改 (如 quat_normalize(&q, &q))
    t_w = q->w * inv;
    t_x = q->x * inv;
    t_y = q->y * inv;
    t_z = q->z * inv;

    result->w = t_w;
    result->x = t_x;
    result->y = t_y;
    result->z = t_z;
}

void omega_to_dq(Quat *result, float wx, float wy, float wz, float dt) {
    float ww = wx*wx;
    float yy = wy*wy;
    float zz = wz*wz;
    float omega = sqrt(ww + yy + zz);
    float half_dt, theta, half, s;

    if (omega < 1e-9f) {
        half_dt = 0.5f * dt;
        result->w = 1.0f;
        result->x = wx * half_dt;
        result->y = wy * half_dt;
        result->z = wz * half_dt;
        quat_normalize(result, result);
        return;
    }
    
    theta = omega * dt;
    half = 0.5f * theta;
    s = sin(half) / omega;
    
    result->w = cos(half);
    result->x = wx * s;
    result->y = wy * s;
    result->z = wz * s;
    quat_normalize(result, result);
}

void rotate_vector_by_quat(Vec3 *result, Quat *q, Vec3 *v) {
    Quat vq;
    Quat q_conj;
    Quat temp;
    Quat rotated;
    
    vq.w = 0.0f;
    vq.x = v->x;
    vq.y = v->y;
    vq.z = v->z;
    
    quat_conj(&q_conj, q);
    quat_mul(&temp, q, &vq);
    quat_mul(&rotated, &temp, &q_conj);
    
    result->x = rotated.x;
    result->y = rotated.y;
    result->z = rotated.z;
}

void quat_to_euler(Quat *q, float *roll, float *pitch, float *yaw) {
    float w = q->w, x = q->x, y = q->y, z = q->z;
    float roll_rad, pitch_rad, yaw_rad;
    float roll_num, roll_den;
    float pitch_val;
    float yaw_num, yaw_den;

    // 彻底拆解复杂的三角函数和浮点表达式
    roll_num = 2.0f * (w*x + y*z);
    roll_den = 1.0f - 2.0f * (x*x + y*y);
    roll_rad = atan2(roll_num, roll_den);
    
    pitch_val = 2.0f * (w*y - z*x);
    // 限幅保护：浮点误差可能导致 pitch_val 略大于 1，引发 asin 崩溃
    if (pitch_val > 1.0f) pitch_val = 1.0f;
    else if (pitch_val < -1.0f) pitch_val = -1.0f;
    pitch_rad = asin(pitch_val);
    
    yaw_num = 2.0f * (w*z + x*y);
    yaw_den = 1.0f - 2.0f * (y*y + z*z);
    yaw_rad = atan2(yaw_num, yaw_den);
    
    *roll = roll_rad * 180.0f / 3.1415f;
    *pitch = pitch_rad * 180.0f / 3.1415f;
    *yaw = yaw_rad * 180.0f / 3.1415f;
}