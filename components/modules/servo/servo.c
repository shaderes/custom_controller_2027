#include "servo.h"
#include "bsp_pwm.h"

Servo_ctrl_t servo_list[2];
#define SERVO(INDEX) (servo_list[INDEX])

void servo_init(uint8_t index, uint16_t duty_max_lim, uint16_t duty_min_lim, uint16_t duty_base)
{
  servo_list[index].max = duty_max_lim;
  servo_list[index].min = duty_min_lim;
  servo_list[index].base = duty_base;
  servo_list[index].offset = 0;
  servo_list[index].enable = 0;
}

void servo_enable(uint8_t index)
{
  servo_list[index].enable = 1;
}

void servo_disable(uint8_t index)
{
  servo_list[index].enable = 0;
}

void servo_set_offset(uint8_t index, int16_t offset)
{
  servo_list[index].offset = offset;
}

void servo_add_offset(uint8_t index, int16_t offset)
{
  servo_list[index].offset = servo_list[index].offset + offset;
}

void __SERVO_CTRL_HOOK(void)
{
  uint8_t index;
  for (index = 0; index < PWM_PORT_COUNT; index++)
  {
    if (servo_list[index].enable)
    {
      //由于机械安装原因，假设min = 30，base = 60,max = 70此时base对应水平位置
      //通过操控offset 来实现输出duty
      servo_list[index].duty = servo_list[index].base + servo_list[index].offset;
      if (servo_list[index].duty > servo_list[index].max)
      {
        servo_list[index].duty = servo_list[index].max;
        // servo_list[index].duty = servo_list[index].base + servo_list[index].offset;逆推确定offset
        servo_list[index].offset = servo_list[index].max - servo_list[index].base; // = duty(max)-base
      }
      else if (servo_list[index].duty < servo_list[index].min)
      {
        servo_list[index].duty = servo_list[index].min;
        servo_list[index].offset = servo_list[index].base - servo_list[index].min;
      }
    }
    else
    {
      servo_list[index].duty = 0;
    }
  }
  set_pwm1_duty(servo_list[PWM2].duty);
  set_pwm2_duty(servo_list[PWM1].duty);
}
