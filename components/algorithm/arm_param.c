#include "arm_param.h"

/* 机械臂参数本体。
 *
 * arm_param.h 里只放 extern 声明，定义放在这里：
 * 头文件里写 `arm_param_t arm_param;` 的话，每个包含它的 .c（main.c、
 * kinematics.c、gravity_comp.c ...）都会生成一份定义，链接时报
 *   L6200E: Symbol arm_param multiply defined
 *
 * 目前保持全 0，即重力补偿算出的力矩为 0（不向电机施加力矩）。
 * 参数确认后在这里初始化，例如：
 *     {0, 0, 130, 0, 0, 0},            // arm_a
 *     {0, -HPI, 0, -HPI, HPI, -HPI},   // arm_alpha
 *     ...
 */
arm_param_t arm_param;
