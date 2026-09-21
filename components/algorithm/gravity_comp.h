#ifndef __GRAVITY_COMP_H
#define __GRAVITY_COMP_H

#include "stm32f4xx_hal.h"

uint8_t gravity_compensation(float theta[6], float torque[6]);

#endif
