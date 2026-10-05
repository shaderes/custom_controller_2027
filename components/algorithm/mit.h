#ifndef __MIT_H
#define __MIT_H

typedef struct 
{
    float kp;
    float kd;
    float current;
    float torque;
    float ref_angle;
    float ref_speed;
    float set_angle;
    float set_speed;
} mit_t;


void mit_init(mit_t *mit, float kp, float kd);
void mit_calc(mit_t *mit, float target_angle, float target_speed, float ff_torque);

#endif
