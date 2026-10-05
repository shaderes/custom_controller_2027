#include "stm32f4xx_hal.h"
#include "mit.h"

void mit_init(mit_t *mit, float kp, float kd)
{
    mit->kp = kp;
    mit->kd = kd;
}

void mit_calc(mit_t *mit, float target_angle, float target_speed, float ff_torque)
{
    mit->set_angle = target_angle;
    mit->set_speed = target_speed;
    mit->torque = mit->kp * (target_angle - mit->ref_angle) + mit->kd * (target_speed - mit->ref_speed) + ff_torque;
}
