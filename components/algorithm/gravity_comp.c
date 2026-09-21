#include "gravity_comp.h"
#include "arm_param.h"
#include "kinematics.h"
#include "matrix.h"
#include "ZDT/ZDT_X42_V2.h"
#include "stm32f4xx_hal.h"

uint8_t gravity_compensation(float theta[6], float torque[6])
{
    float T[6][16];
    for (int i = 5; i >= 0; i--)
    {
        forward_kinematics(theta, i, T[i]);
        
        float p[6][3];//关节坐标
        float z[6][3];//z轴单位向量
        float cv[6][3];//质心坐标
        
        p[i][0] = T[i][3];
        p[i][1] = T[i][7];
        p[i][2] = T[i][11];

        z[i][0] = T[i][2];
        z[i][1] = T[i][6];
        z[i][2] = T[i][10];

        for (uint8_t j = 0; j < 6; j++)
            transform_point(T[j], arm_param.link_center[j], cv[j]);

        for (uint8_t j = i; j < 6; j++)
        {
            float f[3] = {0, 0, -arm_param.link_mass[i] * GRAVITY};
            float r[3];
            float m[3];
            vec_sub(cv[j], p[j], r);
            vec_cross(f, r, m);
            torque[i] += vec_dot(m, z[j]); 
        }
    }

    return 1;
}
