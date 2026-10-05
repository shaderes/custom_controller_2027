#include "kinematics.h"
#include "arm_param.h"
#include "matrix.h"
#include "math.h"

/**
  * @brief    正运动学：算出前 joint 个关节累计的 4x4 变换矩阵（行主序）
  * @param    theta ：关节角数组（单位 rad），第 i 个元素是第 i 个关节角
  * @param    joint ：要累计的关节个数，joint = 0 时输出单位阵
  * @param    T     ：输出，16 个 float 的 4x4 行主序矩阵，平移在 T[3]/T[7]/T[11]
  * @retval   none
  * @note     原来这里用的是全局 arm_param.arm_theta[i]，函数传进来的 theta 被无视了：
  *           一旦有人换一组角度调用就会算错，所以改成用参数 theta。
  *           另外内层拷贝 T 的循环原来也叫 i（遮蔽外层 i，虽然结果对但容易看错），改名为 k。
  */
void forward_kinematics(float theta[6], int8_t joint, float T[16])
{
    int   i;
    int   k;
    float T_temp[16];

    for (i = 0; i < 16; i++)
        T[i] = (i % 5 == 0) ? 1.0f : 0.0f;

    for (i = 0; i < joint; i++)
    {
        float ct = cosf(theta[i]);
        float st = sinf(theta[i]);
        float ca = arm_param.arm_ca[i];
        float sa = arm_param.arm_sa[i];

        float T_transform[16] = {
            ct,      -st,      0.0f,    arm_param.arm_a[i],
            st * ca,  ct * ca, -sa,    -arm_param.arm_d[i] * sa,
            st * sa,  ct * sa,  ca,     arm_param.arm_d[i] * ca,
            0.0f,     0.0f,     0.0f,   1.0f
        };

        matrix_mul44(T, T_transform, T_temp);

        for (k = 0; k < 16; k++)
            T[k] = T_temp[k];
    }
}
