#ifndef ARM_PARAM
#define ARM_PARAM

#include "gravity_comp.h"
#include "kinematics.h"
#include "main.h"

/* 机械臂参数的数值都在 arm_param.c。
   头文件里只能放 extern 声明：一旦写成定义，每个包含本头文件的 .c
   （main.c / kinematics.c / gravity_comp.c ...）都会生成一份，
   链接时报 L6200E: Symbol ... multiply defined。 */
extern float arm_a_i[6];
extern float arm_alpha_i[6];
extern float arm_d_i[6];
extern float arm_theta_i[6];

extern float arm_ca_i[6];
extern float arm_sa_i[6];

extern float link_mass_i[6];        // x28电机质量未知
extern float link_center_i[6][3];

typedef struct
{
    float arm_a[6];
    float arm_alpha[6];
    float arm_d[6];
    float arm_theta[6];

    float arm_ca[6];
    float arm_sa[6];

    float link_mass[6];
    float link_center[6][3];

    float joint_torque[6];
    float joint_current[6];
    float joint_speed[6];
} arm_param_t;

/* 参数本体定义在 arm_param.c，这里只声明 */
extern arm_param_t arm_param;

#endif
