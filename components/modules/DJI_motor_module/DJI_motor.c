#include "DJI_motor.h"
#include "CAN_receive.h"
#include "angle_process.h"
#include <string.h>
#include "bsp_buzzer.h"

// vofa测试
// fp32 vofa_output_speed = 0.0f;

// ABS
#define ABS(X) ((X) > 0 ? (X) : (-X))

// 电流输出数组宏函数
#define IS_OUTPUT_ID_200H(id) ((id >= 0x201) && (id <= 0x204))
#define IS_OUTPUT_ID_1FFH(id) ((id >= 0x205) && (id <= 0x208))
#define GET_OUTPUT_CURRENT_INDEX(id) ((id - 0x201) % 4)

// Motor_Ctrl宏函数
#define __DJI_Motor_Ctrl_get_reverse_current(motor_ptr, current) (motor_ptr->reverse_flag ? -current : current)
#define __DJI_Motor_Ctrl_get_torque(motor_ptr) ((motor_ptr->recv_pack).torque)
#define __DJI_Motor_Ctrl_get_speed(motor_ptr) ((motor_ptr->recv_pack).speed_rpm)
#define __DJI_Motor_Ctrl_get_ecd(motor_ptr) ((motor_ptr->recv_pack).ecd)
#define __DJI_Motor_Ctrl_get_ecd_angle(motor_ptr) (motor_ptr->ecd_angle)
#define __DJI_Motor_Ctrl_get_angle(motor_ptr) (motor_ptr->angle_sum_flag ? motor_ptr->angle_sum : (motor_ptr->circle_count) * PI * 2 + *(motor_ptr->ref_ptr))
#define __DJI_Motor_Ctrl_get_init_state(motor_ptr, state_flag) ((motor_ptr->init_state) & (state_flag))
#define __DJI_Motor_Ctrl_set_init_state(motor_ptr, state_flag) ((motor_ptr->init_state) |= (state_flag))

#define __DJI_Motor_Ctrl_write_current(motor_ptr, current)                                                                                            \
  if (IS_OUTPUT_ID_200H(motor_ptr->id))                                                                                                               \
    (motor_ptr->mounted_bus->output_current200H)[GET_OUTPUT_CURRENT_INDEX(motor_ptr->id)] = __DJI_Motor_Ctrl_get_reverse_current(motor_ptr, current); \
  if (IS_OUTPUT_ID_1FFH(motor_ptr->id))                                                                                                               \
    (motor_ptr->mounted_bus->output_current1FFH)[GET_OUTPUT_CURRENT_INDEX(motor_ptr->id)] = __DJI_Motor_Ctrl_get_reverse_current(motor_ptr, current);

// 功能性代码(只会出现在一个位置,仅为方便修改集中在此)
/*靠近角度边界且被监测值(速度，电流)仍向边界行去时触发*/
#define __DJI_Motor_Ctrl_boundary_ctrl(motor_ptr, detected_value)                               \
  {                                                                                             \
    if (__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_ANGLE_LIMIT_INIT))                         \
    {                                                                                           \
      if ((motor->max_angle - motor->braking_angle) > __DJI_Motor_Ctrl_get_angle(motor) &&      \
          detected_value > 0)                                                                   \
        DJI_Motor_set_angle(motor, motor->max_angle);                                           \
      else if ((motor->min_angle + motor->braking_angle) < __DJI_Motor_Ctrl_get_angle(motor) && \
               detected_value < 0)                                                              \
        DJI_Motor_set_angle(motor, motor->min_angle);                                           \
    }                                                                                           \
  }

//if (current_speed < 20.0f && current_current > motor->stall_detect_threshold)目标速度
/*堵转检测*/ // 稍后完善
#define __DJI_Motor_Ctrl_stall_detect(motor_ptr)                            \
                                                                            \
  if (__DJI_Motor_Ctrl_get_init_state(motor_ptr, MOTOR_STALL_DETECT_INIT))  \
  {                                                                         \
    if (motor_ptr->stall_loop_count == motor_ptr->stall_loop_count_compare) \
    {                                                                       \
      motor_ptr->stall_flag = 1;                                            \
    }                                                                       \
    else                                                                    \
    {                                                                       \
      motor_ptr->stall_loop_count++;                                        \
    }                                                                       \
    if (motor_ptr->stall_flag == 1)                                         \
    {                                                                       \
      __DJI_Motor_warning();                                                \
      DJI_Motor_set_nonforce(motor_ptr);                                    \
    }                                                                       \
  }

// 使用应用层的detect模块中已完成下方两宏函数功能(保留)，若没有detect模块时就设置检测一段时间未收到反馈来设置掉线标志位
/*电机掉线检测*/
#define __DJI_Motor_offline_detect(motor_ptr)                                          \
  if (__DJI_Motor_Ctrl_get_init_state(motor_ptr, MOTOR_OFFLINE_DETECT_INIT))           \
  {                                                                                    \
    if (motor_ptr->non_feedback_loop_count == motor_ptr->non_feedback_counter_compare) \
    {                                                                                  \
      motor_ptr->offline_flag = 1;                                                     \
    }                                                                                  \
    else                                                                               \
    {                                                                                  \
      motor_ptr->non_feedback_loop_count++;                                            \
    }                                                                                  \
    if (motor_ptr->offline_flag == 1)                                                  \
    {                                                                                  \
      __DJI_Motor_warning();                                                           \
      DJI_Motor_set_nonforce(motor_ptr);                                               \
    }                                                                                  \
  }
/*电机掉线恢复*/
#define __DJI_Motor_offline_counter(motor_ptr)                               \
  if (__DJI_Motor_Ctrl_get_init_state(motor_ptr, MOTOR_OFFLINE_DETECT_INIT)) \
  {                                                                          \
    if (motor_ptr->offline_flag == 1)                                        \
    {                                                                        \
      if (motor_ptr->offline_recover)                                        \
      {                                                                      \
        motor_ptr->non_feedback_loop_count--;                                \
        if (motor_ptr->non_feedback_loop_count == 0)                         \
          motor_ptr->offline_flag = 0;                                       \
      }                                                                      \
    }                                                                        \
    else                                                                     \
    {                                                                        \
      motor_ptr->non_feedback_loop_count = 0;                                \
    }                                                                        \
  }

/**
 * @brief 电机初始化
 * @param[out] motor 电机控制句柄
 * @param[in] bus 电机搭载can总线句柄
 * @param[in] id 电机can总线id(0x201~0x208)
 */
// 在应用层小心使用再次初始化！！！，如果一直错误判断会让他一直初始化一直循环创造一个新的大疆电机结构体导致他溢栈，最会进入硬错误
void DJI_Motor_init(DJI_Motor_Ctrl_t *motor, DJI_Motor_Bus_t *bus, DJI_Motor_Type_e motor_type, uint16_t id)
{
  memset((void *)motor, 0x0, sizeof(DJI_Motor_Ctrl_t));
  motor->type = motor_type;
  motor->id = id;
  motor->mounted_bus = bus;
  motor->ref_ptr = &(motor->ecd_angle);
  DJI_Motor_set_offline_detect(motor, 10, 0);

  DJI_CANBus_add_motor(bus, motor);
  DJI_Motor_set_nonforce(motor);
}

/**
 * @brief 角度限制设置(opt config)
 * @param[in,out] motor 电机控制句柄
 * @param[in] max_angle 最大角度
 * @param[in] min_angle 最小角度
 * @param[in] braking_angle 边界刹车角度
 * @note
 * 1.本函数在pid输入前作用，对pid目标值(设定值进行限制)
 * 2.力矩(电流)与速度控制时，当角度在
 * max_angle-braking_angle ~ max_angle
 * 或min_angle ~ min_angle+braking_angle
 * 的范围时，设置为最值的位置环控制
 */
// 暂未使用
void DJI_Motor_set_angle_limit(DJI_Motor_Ctrl_t *motor, fp32 max_angle, fp32 min_angle, fp32 braking_angle)
{
  if (motor == NULL)
    return;
  motor->max_angle = max_angle;
  motor->min_angle = min_angle;
  motor->braking_angle = braking_angle;
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_ANGLE_LIMIT_INIT);
}

void DJI_Motor_set_speed_limit(DJI_Motor_Ctrl_t *motor, fp32 max_speed)
{
  motor->speed_limit = max_speed;
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_SPEED_LIMIT_INIT);
}

void DJI_Motor_set_current_limit(DJI_Motor_Ctrl_t *motor, uint16_t max_current)
{
  motor->current_limit = max_current;
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_CURRENT_LIMIT_INIT);
}

void inline DJI_Motor_set_reverse(DJI_Motor_Ctrl_t *motor)
{
  motor->reverse_flag = 1;
}

/**
 * @brief 设置外部角度反馈
 * @param[in,out] motor 电机控制句柄
 * @param[in] feedback_angle 角度反馈值指针
 * @note 原pid默认使用电机自带编码器作为pid角度反馈
 */
void DJI_Motor_set_angle_feedback(DJI_Motor_Ctrl_t *motor, fp32 *feedback_angle)
{
  motor->ref_ptr = feedback_angle;
  __DJI_Motor_Ctrl_set_init_state(motor, EXTERN_ENCODER_INIT);
}

/**
 * @brief 电机位置环PID初始化
 */
void DJI_Motor_Pos_PID_init(DJI_Motor_Ctrl_t *motor, enum PID_MODE pid_mode,
                            fp32 Kp, fp32 Ki, fp32 Kd,
                            fp32 max_out, fp32 max_iout)
{
  fp32 pid[3] = {Kp, Ki, Kd};
  PID_Init(&(motor->pid_pos_loop), pid_mode, pid, max_out, max_iout, 0.01, 10);
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_POS_PID_INIT);
}

/**
 * @brief 电机速度环PID初始化
 */
void DJI_Motor_Speed_PID_init(DJI_Motor_Ctrl_t *motor, enum PID_MODE pid_mode,
                              fp32 Kp, fp32 Ki, fp32 Kd,
                              fp32 max_out, fp32 max_iout)
{
  fp32 pid[3] = {Kp, Ki, Kd};
  PID_Init(&(motor->pid_speed_loop), pid_mode, pid, max_out, max_iout, 0.01, 10);
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_SPEED_PID_INIT);
}

void DJI_Motor_set_multiple_circle_angle(DJI_Motor_Ctrl_t *motor)
{
  motor->circle_count_flag = 1;
  motor->angle_sum_flag = 0;
}

void DJI_Motor_set_sum_angle(DJI_Motor_Ctrl_t *motor)
{
  motor->circle_count_flag = 0;
  motor->angle_sum_flag = 1;
}

void DJI_Motor_PID_set_deadband(DJI_Motor_Ctrl_t *motor, fp32 deadband)
{
}

inline void DJI_Motor_set_stall_detect(DJI_Motor_Ctrl_t *motor)
{
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_STALL_DETECT_INIT);
}

// 检测若长时间未接收到反馈
void DJI_Motor_set_offline_detect(DJI_Motor_Ctrl_t *motor, uint16_t counter, uint8_t recoverable)
{
  motor->non_feedback_counter_compare = counter;
  motor->non_feedback_loop_count = 0;
  motor->offline_flag = 0;
  motor->offline_recover = recoverable;
  __DJI_Motor_Ctrl_set_init_state(motor, MOTOR_OFFLINE_DETECT_INIT);
}

void DJI_Motor_set_angle(DJI_Motor_Ctrl_t *motor, fp32 angle)
{
  DJI_Motor_set_online(motor);

  if (!__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_SPEED_PID_INIT | MOTOR_POS_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor, NON_FORCE);
    return;
  }

  if (__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_ANGLE_LIMIT_INIT))
  {
    if (motor->max_angle < angle)
      angle = motor->max_angle;
    else if (motor->min_angle > angle)
      angle = motor->min_angle;
  }

  motor->mode = POS_LOOP;
  motor->set_angle = angle;
}

void DJI_Motor_set_speed(DJI_Motor_Ctrl_t *motor, fp32 speed_rpm)
{
  DJI_Motor_set_online(motor);

  if (!__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor, NON_FORCE);
    return;
  }

  if (__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_SPEED_LIMIT_INIT))
  {
    if (speed_rpm > motor->speed_limit)
      speed_rpm = motor->speed_limit;
    else if (speed_rpm < -motor->speed_limit)
      speed_rpm = -motor->speed_limit;
  }

  motor->mode = SPEED_LOOP;
  motor->set_speed = speed_rpm;
}

void DJI_Motor_set_current(DJI_Motor_Ctrl_t *motor, int16_t current)
{
  DJI_Motor_set_online(motor);

  if (__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_CURRENT_LIMIT_INIT))
  {
    if (current > motor->current_limit)
      current = motor->current_limit;
    else if (current < -motor->current_limit)
      current = -motor->current_limit;
  }

  motor->mode = GIVING_CURRENT;
  motor->set_current = current;
}

void DJI_Motor_set_nonforce(DJI_Motor_Ctrl_t *motor)
{
  motor->mode = NON_FORCE;
  motor->set_current = 0x00;
}

void DJI_Motor_lockup(DJI_Motor_Ctrl_t *motor)
{
  if (!__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_POS_PID_INIT | MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor, NON_FORCE);
    return;
  }

  motor->mode = LOCK;
  if (motor->set_lock_angle == 0.0f)
    motor->set_lock_angle = __DJI_Motor_Ctrl_get_ecd_angle(motor);
}

// 使用时若没有detect模块，将motor->mode = OFFLINE改为检测未更新数据的判断
void DJI_Motor_set_offline(DJI_Motor_Ctrl_t *motor)
{
  if (motor->mode == OFFLINE)
    __DJI_Motor_offline_detect(motor); // 若一段时间内offline则设置flag

  // 蜂鸣器报警
}

void DJI_Motor_set_online(DJI_Motor_Ctrl_t *motor)
{
  motor->offline_recover = 1;
  __DJI_Motor_offline_counter(motor);
}
/**
 * @brief 获取反馈数值
 * @param[in]  motor  电机控制句柄
 * @param[out] torque 力矩/电流
 * @param[out] speed  速度(rpm)
 * @param[out] angle  角度(rad)
 */
void DJI_Motor_get_feedback(DJI_Motor_Ctrl_t *motor, fp32 *torque, fp32 *speed, fp32 *angle)
{//整数向浮点数转化属于隐式转换，不会丢失精度
  if (torque != NULL)
    *torque = __DJI_Motor_Ctrl_get_torque(motor);
  if (speed != NULL)
    *speed = __DJI_Motor_Ctrl_get_speed(motor);
  if (angle != NULL)
    *angle = __DJI_Motor_Ctrl_get_angle(motor);
  return;
}

uint8_t DJI_Motor_get_stall_flag(DJI_Motor_Ctrl_t *motor)
{
  return motor->stall_flag;
}

uint8_t DJI_Motor_get_offline_flag(DJI_Motor_Ctrl_t *motor)
{
  return motor->offline_flag;
}

void DJI_Motor_clear_offline_flag(DJI_Motor_Ctrl_t *motor)
{
  motor->offline_flag = 0;
}

void DJI_Motor_clear_circle_count(DJI_Motor_Ctrl_t *motor)
{
  motor->circle_count = 0;
}

/**
 * @brief 电机控制钩子函数
 * @note 被定时调用
 * @see __DJI_CANBus_ctrl_loop
 */
void __DJI_Motor_ctrl_loop(DJI_Motor_Ctrl_t *motor)
{
  // __DJI_Motor_offline_detect(motor);
  switch (motor->mode)
  {
  case SPEED_LOOP:
    __DJI_Motor_speed_ctrl_loop(motor);
    break;

  case LOCK:
  case POS_LOOP:
    __DJI_Motor_pos_ctrl_loop(motor);
    break;

  default:
  case NON_FORCE:
  case GIVING_CURRENT:
    __DJI_Motor_current_ctrl_loop(motor);
    break;
  }
}

/**
 * @brief 速度控制钩子
 * @note 被__DJI_Motor_ctrl_loop调用
 */
void __DJI_Motor_speed_ctrl_loop(DJI_Motor_Ctrl_t *motor)
{
  if (!__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_SPEED_PID_INIT))
  {
    __DJI_Motor_Ctrl_set_init_state(motor, NON_FORCE);
    return;
  }

  __DJI_Motor_Ctrl_boundary_ctrl(motor, motor->set_speed);
  __DJI_Motor_Ctrl_write_current(motor, __DJI_Motor_speed_loop_calc(motor));
}

void __DJI_Motor_pos_ctrl_loop(DJI_Motor_Ctrl_t *motor)
{
  if ((!__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_SPEED_PID_INIT)) ||
      (!__DJI_Motor_Ctrl_get_init_state(motor, MOTOR_POS_PID_INIT)))
  {
    __DJI_Motor_Ctrl_set_init_state(motor, NON_FORCE);
    return;
  }

  __DJI_Motor_Ctrl_write_current(motor, __DJI_Motor_angle_loop_calc(motor));
}

/**
 * @brief 电机电流控制循环
 * @details 用于在定时器中刷新电流输出
 */
void __DJI_Motor_current_ctrl_loop(DJI_Motor_Ctrl_t *motor)
{
  if (motor->mode != LOCK)
    motor->set_lock_angle = 0.0f;
  else
    motor->set_angle = motor->set_lock_angle;

  __DJI_Motor_Ctrl_boundary_ctrl(motor, motor->set_current);
  __DJI_Motor_Ctrl_write_current(motor, motor->set_current);
}

/**
 * @brief 反馈回调函数
 * @note  获取到反馈时触发
 */
void __DJI_Motor_get_feedback(DJI_Motor_Ctrl_t *motor, uint8_t *rx_msg)
{
  motor->last_ecd = motor->recv_pack.ecd;
  // memcpy((void*)&(motor->recv_pack),rx_msg,sizeof(uint8_t)*8);//报文高低8位方向相反，不能直接memcpy
  //  报文赋值
  motor->recv_pack.ecd = rx_msg[0] << 8;
  motor->recv_pack.ecd |= rx_msg[1];
  motor->recv_pack.speed_rpm = rx_msg[2] << 8;
  motor->recv_pack.speed_rpm |= rx_msg[3];
  motor->recv_pack.torque = rx_msg[4] << 8;
  motor->recv_pack.torque |= rx_msg[5];
  motor->recv_pack.temp = rx_msg[6];

  // 刷新角度值
  motor->last_ecd_angle = motor->ecd_angle;
  motor->ecd_angle = NORMALIZE_TO_2PI(ecd_to_angle(motor->recv_pack.ecd, 8191, 0, 0.0));

  // 反馈反转(如果启用反转)
  /*note: 上电角度可能不是0.0，由于电机转子的角度传感器是绝对位置*/
  if (motor->reverse_flag)
  {
    motor->recv_pack.speed_rpm = -motor->recv_pack.speed_rpm;
    motor->recv_pack.torque = -motor->recv_pack.torque;
    motor->ecd_angle = NORMALIZE_TO_2PI(angle_normalize(2 * PI - motor->ecd_angle, 0.0));
  }

  // 刷新圈数
  if (motor->circle_count_flag)
  {
    if (motor->ecd_angle > (2 * PI * 7 / 8) && motor->last_ecd_angle < (2 * PI * 1 / 8))
    {
      motor->circle_count--;
    }
    else if (motor->last_ecd_angle > (2 * PI * 7 / 8) && motor->ecd_angle < (2 * PI * 1 / 8))
    {
      motor->circle_count++;
    }
  }

  // 圈数累加
  if (motor->angle_sum_flag)
  {
    int32_t delta_angle = (motor->recv_pack.ecd) - (motor->last_ecd);
    if (delta_angle > 8191 / 2)
      delta_angle -= 8191;
    else if (delta_angle < -8191 / 2)
      delta_angle += 8191;

    motor->ecd_sum += delta_angle;

    motor->angle_sum = motor->ecd_sum * 2 * PI / 8191;
  }

  // 掉线检测计数置零
  //__DJI_Motor_offline_counter(motor);
}

static fp32 __DJI_Motor_speed_loop_calc(DJI_Motor_Ctrl_t *motor)
{
  // vofa测试
  // vofa_output_speed = motor->set_speed;
  return PID_Calc(&(motor->pid_speed_loop), __DJI_Motor_Ctrl_get_speed(motor), motor->set_speed);
}

static fp32 __DJI_Motor_angle_loop_calc(DJI_Motor_Ctrl_t *motor)
{
  fp32 output_speed = 0.0f;

  if (motor->circle_count_flag)
  {
    output_speed = PID_Calc(
        &(motor->pid_pos_loop),
        __DJI_Motor_Ctrl_get_angle(motor),
        motor->set_angle);
  }
  else
  {
    output_speed = PID_Calc(
        &(motor->pid_pos_loop),
        0,
        angle_normalize(motor->set_angle, -__DJI_Motor_Ctrl_get_angle(motor)));
  }

  //__DJI_Motor_Ctrl_stall_detect(motor);

  // vofa测试
  // vofa_output_speed = output_speed;

  return PID_Calc(&(motor->pid_speed_loop),
                  __DJI_Motor_Ctrl_get_speed(motor),
                  output_speed);
}

static void __DJI_Motor_warning(void)
{
  // buzzer_on(0,0);
}
