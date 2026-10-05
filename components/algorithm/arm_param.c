#include "arm_param.h"

/* ============================================================================
 *  机械臂参数定义（头文件 arm_param.h 里只有 extern 声明）
 *
 *  为什么单独放一个 .c：
 *      原来这些数组和 arm_param 都写在 arm_param.h 里，而 main.c、kinematics.c、
 *      gravity_comp.c 都会包含这个头文件，于是每个 .o 里都有一份定义，
 *      链接直接报 L6200E: Symbol ... multiply defined。
 * ==========================================================================*/

float arm_a_i[6]     = {0, 0, 130, 0, 0, 0};
float arm_alpha_i[6] = {0, -HPI, 0, -HPI, HPI, -HPI};
float arm_d_i[6]     = {138.451f, 0, 0, 114.5f, 0, 0};
float arm_theta_i[6] = {0, 0, 0, 0, 0, 0};

float arm_ca_i[6]    = {1, 0, 1, 0, 0, 0};
float arm_sa_i[6]    = {0, -1, 0, -1, 1, -1};

float link_mass_i[6] = {0, 0.2764f, 0.1633f, 0, 0, 0};        /* x28电机质量未知 */
float link_center_i[6][3] = {
    {0, 0, 0},
    {0, 0, 0.13f},
    {0, 0, 0.0685f},
    {0, 0, 0},
    {0, 0, 0.00505f},
    {0, 0, 0}};

/* 参数本体：重力补偿实际用的就是这一份，用上面的标定值初始化。
 *
 *   arm_theta / joint_speed / joint_torque / joint_current 是运行时状态，保持 0 即可
 *   （分别由 main.c 的关节角 theta[]、电机 0x35 实测转速换算的关节角速度、
 *     gravity_compensation() 算的力矩、力矩→电流换算写入）。
 *   下标统一是"关节号 - 1"；本工程只有 3 台步进电机驱动关节 3/4/5，
 *   所以只有下标 2/3/4 会被真正填上，关节 1/2/6 那几项保持 0。
 *
 *   想临时关掉全部补偿力矩：把下面的初始化去掉、只留
 *       arm_param_t arm_param;
 *   这样力矩恒为 0，电机不会因为补偿而受力。 */
arm_param_t arm_param =
{
    .arm_a         = {0, 0, 130, 0, 0, 0},
    .arm_alpha     = {0, -HPI, 0, -HPI, HPI, -HPI},
    .arm_d         = {138.451f, 0, 0, 114.5f, 0, 0},
    .arm_theta     = {0, 0, 0, 0, 0, 0},
    .arm_ca        = {1, 0, 1, 0, 0, 0},
    .arm_sa        = {0, -1, 0, -1, 1, -1},
    .link_mass     = {0, 0.2764f, 0.1633f, 0, 0, 0},
    .link_center   = {{0, 0, 0},
                      {0, 0, 0.13f},
                      {0, 0, 0.0685f},
                      {0, 0, 0},
                      {0, 0, 0.00505f},
                      {0, 0, 0}},
    .joint_torque  = {0, 0, 0, 0, 0, 0},
    .joint_current = {0, 0, 0, 0, 0, 0},
    .joint_speed = {0, 0, 0, 0, 0, 0},
};
