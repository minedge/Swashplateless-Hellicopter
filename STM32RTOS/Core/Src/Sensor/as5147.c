/**
 * *******************************************************************************************************
 * 	@file as5147.c
 * 	@author Team NOVUS Graduation Project
 * 	@brief	AS5147 module
 * 	@details	This file provides setting functions to manage the fllowing
 * 				functionalities of the setting AS5147 and read & write values of register
 * 	@date 2021-4-15
 * 	@version 0.0.1
 * 
 * *******************************************************************************************************
 */
#include "sensor/as5147.h"

/**
  * @}
  */

/**
  * @brief  initialize as5147 setting
  * @param  hspix SPI hangle Structure definition
  * @param  GPIO_port GPIO init structure definition
  * @param  GPIO_num GPIO pin number
  * @retval boolean
  * seccess 0, if fail return false
  */

int8_t as5147_Init(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIO_port, uint16_t GPIO_num){

	as5147_chip_num = addSPIChip(hspix, GPIO_port, GPIO_num);

	if(as5147_chip_num < 0){
		return -1;
	}

	registerRead(AS5047P_ERRFL);
	registerRead(AS5047P_ERRFL);

	if(as5147_setZeroPosition() < 0){
		return -1;
	}

	return 0;
}


/**
  * @}
  */


/**
  * @brief  SET zero position from as5147
  * @param  none
  * @retval boolean
  * seccess 0, if fail return false
  */

int8_t as5147_setZeroPosition(){
	Frame current_zero_position = { 0, };
	Frame current_position = { 0, };
	uint16_t zero_position_L = 0;
	uint16_t zero_position_M = 0;
	int8_t error;

	current_zero_position.raw = registerRead(AS5047P_ZPOSL);
	if(current_zero_position.values.data < 0) return -1;

	error = registerWrite(AS5047P_ZPOSL, current_zero_position.values.data & (AS5047P_ZPOSL_COMP_I_ERR_EN | AS5047P_ZPOSL_COMP_H_ERR_EN));
	error |= registerWrite(AS5047P_ZPOSL, 0x0000);
	if(error != 0) return -1;

	current_position.raw = registerRead(AS5047P_ANGLEUNC);
	if(current_position.values.data < 0) return -1;

	zero_position_L = (current_zero_position.values.data & (AS5047P_ZPOSL_COMP_I_ERR_EN | AS5047P_ZPOSL_COMP_H_ERR_EN) ) | (current_position.values.data & AS5047P_ZPOSL_ZPOSL);
	zero_position_M = (current_position.values.data >> 6 ) & AS5047P_ZPOSM_ZPOSM;

	error = registerWrite(AS5047P_ZPOSL, zero_position_L);
	error |= registerWrite(AS5047P_ZPOSM, zero_position_M);
	if(error != 0) return -1;

	HAL_Delay(1);

	return 0;
}



/**
  * @}
  */


/**
  * @brief  read current position from AS5147
  * @param  none
  * @retval position value without DAEC (0 ~ 360)
  */

float as5147_readPosition(){
	Frame position = { 0, };

	//position.raw = registerRead(AS5047P_ANGLECOM);AS5047P_ANGLEUNC
	//!NOTE::Next line for debug
	position.raw = registerRead(AS5047P_ANGLEUNC);

	return position.values.data / 16384. * 360;
}


/**
  * @}
  */


/**
  * @}
  */


/**
  * @brief  read register according to register address
  * @param  register_address register_address based on AS5147 datasheet & comment on as5147.h
  * @retval register value in register's address
  */

uint16_t registerRead(uint16_t resgister_address){
	uint16_t register_data = 0;

	Frame command = packCommandFrame(resgister_address, AS5047P_ACCESS_READ);
	SPI_write2ByteRegister(&command.raw, as5147_chip_num);

	//command = packCommandFrame(AS5047P_NOP, AS5047P_ACCESS_READ);
	//register_data = SPI_readWrite2ByteRegister(&command.raw, as5147_chip_num);

	//register_data = SPI_read2ByteRegister(as5147_chip_num);

	SPI_read2ByteRegister(as5147_chip_num, &register_data);

	return register_data;
}


/**
  * @}
  */


/**
  * @brief  write data to register address
  * @param  register_address register_address based on AS5147 datasheet & comment on as5147.h
  * @param  data register value based on AS5147 datasheet
  * @retval return 0
  */

int8_t registerWrite(uint16_t resgister_address, uint16_t data){
	HAL_StatusTypeDef state;

	Frame command = packCommandFrame(resgister_address, AS5047P_ACCESS_WRITE);
	state = SPI_write2ByteRegister(&command.raw, as5147_chip_num);
	if(state != HAL_OK) return -1;

	command = packCommandFrame(data, AS5047P_ACCESS_WRITE);
	state = SPI_write2ByteRegister(&command.raw, as5147_chip_num);
	if(state != HAL_OK) return -1;

	return 0;
}


/**
  * @}
  */


/**
  * @brief SPI transaction consists of a 16-bit command frame 
  * 		followed by a 16-bit data frame. 
  * @param  rw intput read or write state
  * @param  data register value based on AS5147 datasheet
  * @retval Frame struct which has data & R/W state & parity Bit 
  */

Frame packCommandFrame(uint16_t data, uint8_t rw){
	Frame frame = { 0, };
	frame.values.data = data & AS5047P_FRAME_DATA;
	frame.values.rw = rw;
	frame.values.pard = calcParity(frame.raw);

	return frame;
}


/**
  * @}
  */


/**
  * @brief calculate parity bit
  * @param  data register value
  * @retval data
  */
uint8_t calcParity(uint16_t data){
	data ^= data >> 8;              // example for 8-bir (this line scales it up to 16 bit)
	data ^= data >> 4;              // ( a b c d e f g h ) xor ( 0 0 0 0 a b c d ) = ( a b c d ae bf cg dh )
	data ^= data >> 2;              // ( a b c d ae bf cg dh ) xor ( 0 0 a b c d ae bf ) = ( a b ac bd ace bdf aceg bdfh )
	data ^= data >> 1;              // ( a b ac bd ace bdf aceg bdfh ) xor ( 0 a b ac bd ace bdf aceg ) = ( a ab abc abcd abcde abcdef abcdefg abcdefgh )

	data = data & 0x0001;

	return (uint8_t)data;
}



void updatePosition(MOTOR* motor){

#if 0
	motor->pre_ang = motor->ang;

	sens_time = HAL_GetTick() - sens_start;
	motor->ang = as5147_readPosition();
	sens_start = HAL_GetTick();

	motor->ang -= motor->offset;
	if(motor->ang < 0)motor->ang += 360;
#else
	motor->pre_ang = motor->rad;
	sens_time = HAL_GetTick() - sens_start;
	motor->rad = ((as5147_readPosition() * 3.141592) / 180) - motor->offset;

	sens_start = HAL_GetTick();
	if(motor->rad < 0) motor->rad += 6.283184;
#endif
}

void updateRPM(MOTOR* motor){
	motor->pre_rpm = motor->rpm;
	float dif = (motor->ang - motor->pre_ang);
	if (dif < 0)dif += 360;
	motor->rpm = ((dif * (1000. / 1)) / 360.) * 60;
	if(motor->rpm > RPM_MAX || motor->rpm < RPM_MIN){
		motor->rpm = motor->pre_rpm;
	}

	float LPF_HZ = 500;
	motor->pre_lpf = motor->lpf;
	motor->lpf = ((LPF_HZ * motor->pre_lpf) + 1 * motor->rpm )/(LPF_HZ + 1);
}

/**
  * @}
  */


/**
  * @brief  get offset for select front position
  * @param  none
  * @retval none
  */

void setOffset(MOTOR* motor){
	motor->offset = (as5147_readPosition() * 3.141592) / 180;
}
