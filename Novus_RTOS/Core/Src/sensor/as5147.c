/**
 * *******************************************************************************************************
 * 	@file as5147.c
 * 	@author NOVUS Graduation Project Team
 * 	@brief	AS5147 setup module
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


/*
* @brief as5147 must get chip_num through as5147 _Init() 
*        when call addChip() in spi.h will return chip_num 0 to 255
*		 chip_num use every times of SPI protocal communication
*/
static uint8_t chip_num;


/**
  * @}
  */


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

	chip_num = addChip(hspix, GPIO_port, GPIO_num);
	if(chip_num < 0){
		return -1;
	}

	registerRead(AS5047P_ERRFL);
	int16_t error = registerRead(AS5047P_ERRFL);

	if(error != 0) return -1;

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

	HAL_Delay(1);

	current_position.raw = registerRead(AS5047P_ANGLECOM);
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

	position.raw = registerRead(AS5047P_ANGLECOM);

	return position.values.data * 360 / 16384.;
}

/**
  * @}
  */

float as5147_readMag(){
	float mag = 0;

	mag = (registerRead(AS5047P_MAG) & 0x3FF) * 360 / 16384.;

	return mag;
}


/**
  * @}
  */


/**
  * @brief  read register according to register address
  * @param  register_address register_address based on AS5147 datasheet & comment on as5147.h
  * @retval register value in register's address
  */

uint16_t registerRead(uint16_t resgister_address){
	int16_t register_data = 0;

	Frame command = packCommandFrame(resgister_address, AS5047P_ACCESS_READ);
	write2ByteRegister(&command.raw, chip_num);

	register_data = read2ByteRegister(chip_num);

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
	state = write2ByteRegister(&command.raw, chip_num);
	if(state != HAL_OK) return -1;

	command = packCommandFrame(data, AS5047P_ACCESS_WRITE);
	state = write2ByteRegister(&command.raw, chip_num);
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

/**
  * @}
  */


/**
  * @brief calculate rpm based on motor variable angle
  * @param  dif difference between current angle and pre-angle
  * @retval revolution per minute
  */

float calcRPM(float dif, float loop_time){
	if(dif < 0) dif += 360;
	float w = (dif * (2*PI)/360.) * (1./loop_time);			// rad/sec
	float rpm = (w * 60 / (2*PI));							// rotation/min
	return rpm;
}
