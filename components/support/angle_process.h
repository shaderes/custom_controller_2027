/**
 * @brief 角度处理函数库
 */
#ifndef __ANGLE_PROCESS__
#define __ANGLE_PROCESS__

#include "main.h"
#include "struct_typedef.h"

#define PI 3.14159f

#define NORMALIZE_TO_2PI(angle) (angle+PI) // 范围-PI~PI转化为0~2PI

fp32 ecd_to_angle(int16_t ecd,int16_t max_ecd, int32_t offset_ecd , fp32 offset_angle);
fp32 angle_normalize(fp32 angle,fp32 offset);
fp32 angle_limit(fp32 angle,fp32 max_angle,fp32 min_angle);
uint8_t is_angle_around(fp32 target_angle,fp32 cur_angle,fp32 delta);

#endif
