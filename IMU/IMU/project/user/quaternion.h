#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

typedef struct {
    float w, x, y, z;
} Quat;

typedef struct {
    float x, y, z;
} Vec3;

// 全部改为指针传参
void quat_mul(Quat *result, Quat *q1, Quat *q2);
void quat_conj(Quat *result, Quat *q);
float quat_norm(Quat *q);
void quat_normalize(Quat *result, Quat *q);
void omega_to_dq(Quat *result, float wx, float wy, float wz, float dt);
void rotate_vector_by_quat(Vec3 *result, Quat *q, Vec3 *v);
void quat_to_euler(Quat *q, float *roll, float *pitch, float *yaw);

#endif