#ifndef __SERVO_H__
#define __SERVO_H__
#include "main.h"

typedef enum{
  PWM1=0,
  PWM2,
  PWM_PORT_COUNT
} PWM_OUTPUT_PORT;

typedef struct{
  uint16_t max;
  uint16_t min;
  uint16_t base;

  int16_t offset;

  uint8_t enable;
  uint16_t duty;
} Servo_ctrl_t;

void servo_init(uint8_t index,uint16_t duty_max_lim,uint16_t duty_min_lim,uint16_t duty_base);
void servo_set_offset(uint8_t index,int16_t offset);
void servo_add_offset(uint8_t index,int16_t offset);
void servo_enable(uint8_t index);
void servo_disable(uint8_t index);
void __SERVO_CTRL_HOOK(void);

#endif
