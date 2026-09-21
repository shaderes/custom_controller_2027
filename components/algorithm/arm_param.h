#ifndef ARM_PARAM
#define ARM_PARAM

#include "gravity_comp.h"
#include "kinematics.h"
#include "main.h"

// float arm_a_i[6] = {0, 0, 130, 0, 0, 0};
// float arm_alpha_i[6] = {0, -HPI, 0, -HPI, HPI, -HPI};
// float arm_d_i[6] = {72, 0, 0, 114.5, 0, 0};
// float arm_theta_i[6] = {0, 0, 0, 0, 0, 0};    

// float arm_ca_i[6] = {1, 0, 1, 0, 0, 0};
// float arm_sa_i[6] = {0, -1, 0, -1, 1, -1};

// float link_mass_i[6] = {0, 0.2764, 0.1633, 0, 0, 0};//x28电机未知
// float link_center_i[6][3] = {
//                 {0, 0, 0},
//                 {0, 0, 0.13},
//                 {0, 0, 0.0685},
//                 {0, 0, 0},
//                 {0, 0, 0.00505},
//                 {0, 0, 0}};

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
} arm_param_t;

/* 参数本体定义在 arm_param.c，这里只声明：
   放在头文件里定义的话，每个包含本头文件的 .c 都会生成一份，链接报 L6200E */
extern arm_param_t arm_param;

#endif
