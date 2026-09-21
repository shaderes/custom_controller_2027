#include "dm4310_drv.h"

#include "arm_math.h"

extern FDCAN_HandleTypeDef hfdcan2;

Joint_Motor_t DM_Motor_J2 = {0};
Joint_Motor_t DM_Motor_J3 = {0};
Joint_Motor_t DM_Motor_J5 = {0};
Joint_Motor_t DM_Motor_gripper = {0};

float Hex_To_Float(uint32_t *Byte, int num) // 十六进制到浮点数
{
	return *((float *)Byte);
}

uint32_t FloatTohex(float HEX) // 浮点数到十六进制转换
{
	return *(uint32_t *)&HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode, float Kp, float Kd)
{
	motor->mode = mode;
	motor->enable = 1;
	motor->para.id = id;
	motor->Kp = Kp;
	motor->Kd = Kd;
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
	if (data_len == FDCAN_DLC_BYTES_8)
	{ // 返回的数据有8个字节
		motor->para.id = (rx_data[0]) & 0x0F;
		motor->para.state = (rx_data[0]) >> 4;
		motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
		motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
		motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
		motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); //
		motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); //
		motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); //
		motor->para.Tmos = (float)(rx_data[6]);
		motor->para.Tcoil = (float)(rx_data[7]);
	}
}

void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;

	data[0] = 0xFF;
	data[1] = 0xFF; 
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

	fdcanx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;

	fdcanx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(Joint_Motor_t *motor_ptr, float pos, float vel, float kp, float kd, float torq)
{
	uint8_t *data = motor_ptr->output;
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	motor_ptr->output_id = motor_ptr->para.id + MIT_MODE;

	pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(vel == 0.0f ? -0.001f : vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;
}

void mit_nonforce_ctrl(Joint_Motor_t *motor_ptr)
{
	mit_ctrl(motor_ptr, 0.01, 0.01, 0.01, 0.01, 0.01);
}

void mit_ctrl_pos_speed(Joint_Motor_t *motor_ptr, float pos, float vel)
{
	mit_ctrl(motor_ptr, pos, (motor_ptr->para.pos) > 0 ? vel : -vel, motor_ptr->Kp, motor_ptr->Kd, 0);
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
extern void pos_speed_ctrl(Joint_Motor_t *motor_ptr, float pos, float vel)
{
	uint8_t *pbuf, *vbuf;
	uint8_t *data = motor_ptr->output;

	motor_ptr->output_id = motor_ptr->para.id + POS_MODE;
	pbuf = (uint8_t *)&pos;
	vbuf = (uint8_t *)&vel;

	data[0] = *pbuf;
	data[1] = *(pbuf + 1);
	data[2] = *(pbuf + 2);
	data[3] = *(pbuf + 3);

	data[4] = *vbuf;
	data[5] = *(vbuf + 1);
	data[6] = *(vbuf + 2);
	data[7] = *(vbuf + 3);
}

extern void pos_speed_lock(Joint_Motor_t *motor_ptr, float vel)
{
	// if(motor_ptr->lock_angle==0)
	// motor_ptr->lock_angle=motor_ptr->para.pos;
	// pos_speed_ctrl(motor_ptr,motor_ptr->lock_angle,vel);
}

/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t *hcan, uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];

	id = motor_id + SPEED_MODE;
	vbuf = (uint8_t *)&vel;

	data[0] = *vbuf;
	data[1] = *(vbuf + 1);
	data[2] = *(vbuf + 2);
	data[3] = *(vbuf + 3);

	fdcanx_send_data(hcan, id, data, 4);
}

void __dm4310_mit_output_ctrl(hcan_t *hcan, Joint_Motor_t *motor_ptr)
{
	if (!(motor_ptr->enable))
		return;

	uint16_t id = motor_ptr->output_id;
	fdcanx_send_data(hcan, id, motor_ptr->output, 8);
}

void DM_Motor_Offline_Handler(Joint_Motor_t *motor_ptr)
{
	if (motor_ptr == NULL )
		return;
	//使能并发送保持当前位置让电机应答产生反馈刷新掉线标志
	//disable_motor_mode(&hfdcan2, motor_ptr->para.id, motor_ptr->mode);
	float current_angle = motor_ptr->para.pos;
	pos_speed_ctrl(motor_ptr, current_angle, 0.001);

	// if (motor_ptr->enable)//除执行刷新外，不让电机执行其他行为避免意外
	// {
	// 	//禁用所有模式的指令
	// 	//disable_motor_mode(&hfdcan2, motor_ptr->para.id, motor_ptr->mode);

	// 	// 也可以发送 MIT 无力矩指令
	// 	// mit_nonforce_ctrl(motor);

	// 	// 清除使能标志
	// 	motor_ptr->enable = 0;
	// }
	// 蜂鸣器报警
}

//如果有需要速度设置可按照下方编写
void dm_set_pos(Joint_Motor_t *motor_ptr, float angle)
{
	if (motor_ptr->enable == 0)
	{
		motor_ptr->enable = 1;
	}
	if (motor_ptr->para.id == 0x01)
		pos_speed_ctrl(motor_ptr, angle, 1.4);
	else if (motor_ptr->para.id == 0x02)
		pos_speed_ctrl(motor_ptr, angle, 1.4);
	else if (motor_ptr->para.id == 0x03)
		pos_speed_ctrl(motor_ptr, angle, 1);
	else if (motor_ptr->para.id == 0x04)
		pos_speed_ctrl(motor_ptr, angle, 1);

}
