/*
 * uart.c
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */


#include "Serial/uart.h"

/**
  * @}
  */

uint16_t addChannel(UART_HandleTypeDef* huart){
	uint16_t channel_num = 0;
	uint8_t index = 0;

	for(index = 0; index < 256; index++){
		if(channel_list[index] == 0 || channel_list[index] == huart){
			channel_num = index;
			break;
		}
	}

	if(index != 256){
		channel_list[channel_num] = huart;
		return channel_num;
	}else{
		return -1;
	}
}

/**
  * @}
  */

void deleteChannel(uint8_t channel_num){
	channel_list[channel_num] = 0;
}


/**
  * @}
  */

uint8_t readByte(uint8_t channel_num){
	uint8_t buffer;

	HAL_UART_Receive(channel_list[channel_num], &buffer, 1, HAL_TIMEOUT);

	return buffer;
}

/**
  * @}
  */

uint8_t* readNByte(uint8_t channel_num, uint16_t size){
	uint8_t* buffer = (uint8_t*)malloc(sizeof(uint8_t)*size);

	HAL_UART_Receive(channel_list[channel_num], buffer, size, HAL_TIMEOUT);

	return buffer;
}

/**
  * @}
  */

HAL_StatusTypeDef writeByte(uint8_t channel_num, uint8_t pData){
	return HAL_UART_Transmit(channel_list[channel_num], &pData, 1, HAL_TIMEOUT);
}

/**
  * @}
  */

HAL_StatusTypeDef writeNByte(uint8_t channel_num, uint8_t* pData, uint16_t size){
	return HAL_UART_Transmit(channel_list[channel_num], pData, size, HAL_TIMEOUT);
}
