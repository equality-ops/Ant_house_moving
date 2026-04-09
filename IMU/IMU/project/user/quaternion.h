#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

typedef struct {
    float w, x, y, z;
} Quat;

typedef struct {
    float x, y, z;
} Vec3;

Quat quat_mul(Quat q1, Quat q2);
Quat quat_conj(Quat q);
float quat_norm(Quat q);
Quat quat_normalize(Quat q);
Quat omega_to_dq(float wx, float wy, float wz, float dt);
Vec3 rotate_vector_by_quat(Quat q, Vec3 v);
void quat_to_euler(Quat q, float *roll, float *pitch, float *yaw);

#endif