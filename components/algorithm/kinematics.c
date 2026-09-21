#include "kinematics.h"
#include "arm_param.h"
#include "matrix.h"
#include "math.h"

void forward_kinematics(float theta[6], int8_t joint, float T[16])
{
    for (int8_t i = 0; i < 16; i++)
        T[i] = (i % 5 == 0) ? 1 : 0;

    float T_temp[16];

    for (int i = 0; i < joint; i++)
    {
        float ct = cosf(arm_param.arm_theta[i]);
        float st = sinf(arm_param.arm_theta[i]);
        float ca = arm_param.arm_ca[i];
        float sa = arm_param.arm_sa[i];

        float T_transform[16] = {
            ct,      -st,      0.0f,    arm_param.arm_a[i],
            st * ca,  ct * ca, -sa,    -arm_param.arm_d[i] * sa,
            st * sa,  ct * sa,  ca,     arm_param.arm_d[i] * ca,
            0.0f,     0.0f,     0.0f,   1.0f
        };

        matrix_mul44(T, T_transform, T_temp);

        for (uint8_t i = 0; i < 16; i++)
            T[i] = T_temp[i];
    }
}