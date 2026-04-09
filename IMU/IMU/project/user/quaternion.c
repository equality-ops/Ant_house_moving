#include "quaternion.h"

Quat quat_mul(Quat q1, Quat q2) {
    Quat result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

Quat quat_conj(Quat q) {
    Quat result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

float quat_norm(Quat q) {
    return sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

Quat quat_normalize(Quat q) {
    float n;
    Quat result;
    float inv;
    n = quat_norm(q);
    if (n == 0.0f) {
        result.w = 1.0f;
        result.x = 0.0f;
        result.y = 0.0f;
        result.z = 0.0f;
        return result;
    }
    inv = 1.0f / n;
    result.w = q.w * inv;
    result.x = q.x * inv;
    result.y = q.y * inv;
    result.z = q.z * inv;
    return result;
}

Quat omega_to_dq(float wx, float wy, float wz, float dt) {
    float omega;
    Quat result;
    float half_dt;
    float theta, half, s;
    omega = sqrt(wx*wx + wy*wy + wz*wz);
    if (omega < 1e-9f) {
        half_dt = 0.5f * dt;
        result.w = 1.0f;
        result.x = wx * half_dt;
        result.y = wy * half_dt;
        result.z = wz * half_dt;
        return quat_normalize(result);
    }
    theta = omega * dt;
    half = 0.5f * theta;
    s = sin(half) / omega;
    result.w = cos(half);
    result.x = wx * s;
    result.y = wy * s;
    result.z = wz * s;
    return quat_normalize(result);
}

Vec3 rotate_vector_by_quat(Quat q, Vec3 v) {
    Quat vq;
    Quat rotated;
    Vec3 result;
    vq.w = 0.0f;
    vq.x = v.x;
    vq.y = v.y;
    vq.z = v.z;
    rotated = quat_mul(quat_mul(q, vq), quat_conj(q));
    result.x = rotated.x;
    result.y = rotated.y;
    result.z = rotated.z;
    return result;
}

void quat_to_euler(Quat q, float *roll, float *pitch, float *yaw) {
    float w, x, y, z;
    float roll_rad, pitch_rad, yaw_rad;
    w = q.w; x = q.x; y = q.y; z = q.z;
    roll_rad = atan2(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    pitch_rad = asin(2.0f*(w*y - z*x));
    yaw_rad = atan2(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    *roll = roll_rad * 180.0f / 3.1415f;
    *pitch = pitch_rad * 180.0f / 3.1415f;
    *yaw = yaw_rad * 180.0f / 3.1415f;
}