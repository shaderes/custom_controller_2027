/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"
#include "stddef.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
    // fp32 v_set; //输入信号
    // fp32 x1,x2; //过渡信号
    // fp32 y_out; //平滑信号
    // fp32 timing; //采样周期
    // fp32 speeding; //速度因子
void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout,fp32 timing,fp32 speeding)
{  
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->ref[0] = pid->ref[1] = pid->ref[2] = 0.0f;    
    
    
    pid->v_set = 0.0;
    pid->x1 = 0.0;
    pid->x2 = 0.0;
    pid->y_out = 0.0;
    pid->timing = timing; //采样周期为0.01秒
    pid->speeding = speeding; //速度因子为10.0
}

//(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if(set-ref<pid->dead_band&&set-ref>-pid->dead_band)
      ref=set;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
		pid->ref[2] = pid->ref[1];
		pid->ref[1] = pid->ref[0];		
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		pid->ref[0] = ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);

		 }	
			else if(pid->mode == PID_POSITIPON_DELAY)
	  	{

			  pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->ref[0] - pid->ref[1]);
				pid->LastDout = pid->Dout; 
        pid->Dout = pid->Kd * (1.0f-0.1f) * pid->Dbuf[0] + 0.1f * pid->LastDout;
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
			  pid->v_set = pid->out; //将PID输出作为输入信号
        pid->x1 = pid->x1 + pid->timing * pid->x2; //计算过渡信号
        pid->x2 = pid->x2 - pid->timing * pid->speeding * pid->speeding * (pid->x1 - pid->v_set); //计算过渡信号
        pid->y_out = pid->x1 + pid->speeding * (pid->v_set - pid->x1); //计算平滑信号
        return pid->y_out; //返回平滑信号			
			}		
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
    
}

void PID_Set_Deadbound(PidTypeDef *pid,fp32 dead_band)
{
  pid->dead_band=dead_band;
}


static void abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

//static void pid_param_init(
//    pid_t*   pid,
//    uint32_t mode,
//    uint32_t maxout,
//    uint32_t intergral_limit,
//    float    kp,
//    float    ki,
//    float    kd)
//{
//  pid->integral_limit = intergral_limit;
//  pid->max_out        = maxout;
//  pid->pid_mode       = mode;
//
//  pid->p = kp;
//  pid->i = ki;
//  pid->d = kd;
//}
//
///**
//  * @brief     modify pid parameter when code running
//  * @param[in] pid: control pid struct
//  * @param[in] p/i/d: pid parameter
//  * @retval    none
//  */
//static void pid_reset(pid_t *pid, float kp, float ki, float kd)
//{
//  pid->p = kp;
//  pid->i = ki;
//  pid->d = kd;
//
//  pid->pout = 0;
//  pid->iout = 0;
//  pid->dout = 0;
//  pid->out  = 0;
//}
//
//
//
///**
//  * @brief     calculate delta PID and position PID
//  * @param[in] pid: control pid struct
//  * @param[in] get: measure feedback value
//  * @param[in] set: target value
//  * @retval    pid calculate output 
//  */
//float pid_calc(pid_t *pid,int16_t set,int16_t get)
//{
//  pid->err[NOW] = set - get;
//
//  if (pid->pid_mode == POSITION_PID) //position PID
//  {
//      pid->pout = pid->p * pid->err[NOW];
//      pid->iout += pid->i * pid->err[NOW];
//      pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
//    
//      abs_limit(&(pid->iout), pid->integral_limit);
//      pid->out = pid->pout + pid->iout + pid->dout;
//      abs_limit(&(pid->out), pid->max_out);
//  }
//
//  else if (pid->pid_mode == DELTA_PID) //delta PID
//  {
//      pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//      pid->iout = pid->i * pid->err[NOW];
//      pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
//      pid->out += pid->pout + pid->iout + pid->dout;
//      abs_limit(&(pid->out), pid->max_out);
//  }
//  pid->err[LLAST] = pid->err[LAST];
//  pid->err[LAST]  = pid->err[NOW];
//
//  return pid->out;
//}
//
///**
//  * @brief     initialize pid parameter
//  * @retval    none
//  */
//void PID_struct_init(
//    pid_t*   pid,
//    uint32_t mode,
//    uint32_t maxout,
//
//    uint32_t intergral_limit,
//
//    float kp,
//    float ki,
//    float kd)
//{
//  pid->f_param_init = pid_param_init;
//  pid->f_pid_reset  = pid_reset;
//
//  pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
//  pid->f_pid_reset(pid, kp, ki, kd);
//}
