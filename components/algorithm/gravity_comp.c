#include "gravity_comp.h"
#include "arm_param.h"
#include "kinematics.h"
#include "matrix.h"
#include "ZDT/ZDT_X42_V2.h"
#include "stm32f4xx_hal.h"

uint8_t gravity_compensation(float theta[6], float torque[6])
{
    float T[6][16];
    float p[6][3];//关节坐标
    float z[6][3];//z轴单位向量
    float cv[6][3];//质心坐标

    /* 每次调用必须先把输出清零：
       调用方传进来的是 arm_param.joint_torque（不是每轮清零的临时变量），
       下面又是 torque[i] += ...，不清零的话每 20ms 就往上累加一次，
       几秒钟力矩就滚到溢出，电机上会看到力矩单调发散。 */
    for (uint8_t i = 0; i < 6; i++) torque[i] = 0.0f;

    for (int i = 5; i >= 0; i--)
    {
        forward_kinematics(theta, i, T[i]);
        
        p[i][0] = T[i][3];
        p[i][1] = T[i][7];
        p[i][2] = T[i][11];

        z[i][0] = T[i][2];
        z[i][1] = T[i][6];
        z[i][2] = T[i][10];


    }

    for (uint8_t i = 0; i < 6; i++)
        transform_point(T[i], arm_param.link_center[i], cv[i]);

    for (int i = 0; i < 6; i++)
    {
        for (uint8_t j = i; j < 6; j++)
        {
            float f[3] = {0, 0, -arm_param.link_mass[j] * GRAVITY};
            float r[3];
            float m[3];
            vec_sub(cv[j], p[i], r);
            vec_cross(f, r, m);
            torque[i] += vec_dot(m, z[i]); 
        }
    }
    return 1;
}
