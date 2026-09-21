#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "stm32f4xx_hal.h"

void forward_kinematics(float theta[6], int8_t joint, float T[16]);

#endif
