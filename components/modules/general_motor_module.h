#ifndef __GERNERAL_MOTOR_TYPEDEF__
#define __GERNERAL_MOTOR_TYPEDEF__

#include "main.h"

// 电机控制模式,由电机控制函数设置
typedef enum
{
  NON_FORCE = 0,
  SPEED_LOOP,
  POS_LOOP,
  GIVING_CURRENT,
  LOCK,
  OFFLINE,
  STAll,
} Motor_Ctrl_mode_e;

typedef enum
{
  NON_MOTOR = 0x00,
  DJI_MOTOR,
  M8010_MOTOR,
  M4310_MOTOR,
  AK_MOTOR,
} Motor_Type_e;

#define GENERAL_MOTOR_GET_FEEDBACK(instance_ptr, motor_type, current_ptr, speed_ptr, angle_ptr) \
  {                                                                                             \
    switch (motor_type)                                                                         \
    {                                                                                           \
    case DJI_MOTOR:                                                                             \
      DJI_Motor_get_feedback(                                                                   \
          (DJI_Motor_Ctrl_t *)instance_ptr,                                                     \
          (current_ptr),                                                                        \
          (speed_ptr),                                                                          \
          (angle_ptr));                                                                         \
      break;                                                                                    \
    case M8010_MOTOR:                                                                           \
      *current_ptr = ((M8010_motor_t *)instance_ptr)->recv_data.T;                              \
      *speed_ptr = ((M8010_motor_t *)instance_ptr)->recv_data.W;                                \
      *angle_ptr = ((M8010_motor_t *)instance_ptr)->recv_data.Pos;                              \
      break;                                                                                    \
    case M4310_MOTOR:                                                                           \
      *current_ptr = ((Joint_Motor_t *)instance_ptr)->para.tor;                                 \
      *speed_ptr = ((Joint_Motor_t *)instance_ptr)->para.vel;                                   \
      *angle_ptr = ((Joint_Motor_t *)instance_ptr)->para.pos;                                   \
      break;                                                                                    \
    case AK_MOTOR:                                                                              \
      *current_ptr = ((AK_Joint_Motor_t *)instance_ptr)->current;                               \
      *speed_ptr = ((AK_Joint_Motor_t *)instance_ptr)->spd;                                     \
      *angle_ptr = ((AK_Joint_Motor_t *)instance_ptr)->pos;                                     \
      break;                                                                                    \
    default:                                                                                    \
      break;                                                                                    \
    }                                                                                           \
  }

#define GENERAL_MOTOR_SET_OUTPUT(instance_ptr, motor_type, ctrl_state, current, speed, angle) \
  {                                                                                           \
    switch (motor_type)                                                                       \
    {                                                                                         \
    case DJI_MOTOR:                                                                           \
      switch (ctrl_state)                                                                     \
      {                                                                                       \
      case SPEED_LOOP:                                                                        \
        DJI_Motor_set_speed((DJI_Motor_Ctrl_t *)instance_ptr, speed);                         \
        break;                                                                                \
      case POS_LOOP:                                                                          \
        DJI_Motor_set_angle((DJI_Motor_Ctrl_t *)instance_ptr, angle);                         \
        break;                                                                                \
      case GIVING_CURRENT:                                                                    \
        DJI_Motor_set_current((DJI_Motor_Ctrl_t *)instance_ptr, current);                     \
        break;                                                                                \
      case LOCK:                                                                              \
        DJI_Motor_lockup((DJI_Motor_Ctrl_t *)instance_ptr);                                   \
        break;                                                                                \
      case OFFLINE:                                                                           \
        DJI_Motor_set_offline((DJI_Motor_Ctrl_t *)instance_ptr);                              \
      case NON_FORCE:                                                                         \
        break;                                                                                \
      default:                                                                                \
        DJI_Motor_set_nonforce((DJI_Motor_Ctrl_t *)instance_ptr);                             \
        break;                                                                                \
      }                                                                                       \
      break;                                                                                  \
    case M8010_MOTOR:                                                                         \
      switch (ctrl_state)                                                                     \
      {                                                                                       \
      case POS_LOOP:                                                                          \
        M8010_motor_set_angle((M8010_motor_t *)instance_ptr, angle);                          \
        break;                                                                                \
      case LOCK:                                                                              \
        M8010_motor_lock((M8010_motor_t *)instance_ptr);                                      \
        break;                                                                                \
      case NON_FORCE:                                                                         \
        M8010_motor_nonforce((M8010_motor_t *)instance_ptr);                                  \
        break;                                                                                \
      case OFFLINE:                                                                           \
      default:                                                                                \
        M8010_motor_nonforce((M8010_motor_t *)instance_ptr);                                  \
        break;                                                                                \
      }                                                                                       \
      break;                                                                                  \
    case M4310_MOTOR:                                                                         \
      switch (ctrl_state)                                                                     \
      {                                                                                       \
      case POS_LOOP:                                                                          \
        dm_set_pos((Joint_Motor_t *)instance_ptr, angle);                                     \
        break;                                                                                \
      case LOCK:                                                                              \
        pos_speed_ctrl((Joint_Motor_t *)instance_ptr, angle, 0.0001);                         \
        break;                                                                                \
      case NON_FORCE:                                                                         \
        pos_speed_ctrl((Joint_Motor_t *)instance_ptr, angle, 0.0001);                         \
        break;                                                                                \
      case OFFLINE:                                                                           \
        DM_Motor_Offline_Handler((Joint_Motor_t *)instance_ptr);                              \
        break;                                                                                \
      default:                                                                                \
        pos_speed_ctrl((Joint_Motor_t *)instance_ptr, angle, 0.0001);                         \
        break;                                                                                \
      }                                                                                       \
      break;                                                                                  \
    case AK_MOTOR:                                                                            \
      switch (ctrl_state)                                                                     \
      {                                                                                       \
      case POS_LOOP:                                                                          \
        AK_joint_motor_pos_speed_ctrl((AK_Joint_Motor_t *)instance_ptr, angle, 24000, 28000); \
        break;                                                                                \
      case LOCK:                                                                              \
        AK_joint_motor_lock_ctrl((AK_Joint_Motor_t *)instance_ptr);                           \
        break;                                                                                \
      case NON_FORCE:                                                                         \
        AK_joint_motor_nonforce_ctrl((AK_Joint_Motor_t *)instance_ptr);                       \
        break;                                                                                \
      case OFFLINE:                                                                           \
        break;                                                                                \
      default:                                                                                \
        AK_joint_motor_nonforce_ctrl((AK_Joint_Motor_t *)instance_ptr);                       \
        break;                                                                                \
      }                                                                                       \
      break;                                                                                  \
    default:                                                                                  \
      break;                                                                                  \
    }                                                                                         \
  }

#include "DJI_motor.h"
#include "dm4310_drv.h"
#include "M8010_motor.h"
#include "AK_series.h"

#endif
