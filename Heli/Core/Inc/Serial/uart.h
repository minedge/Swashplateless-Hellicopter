/*
 * uart.h
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */

#ifndef INC_SERIAL_UART_H_
#define INC_SERIAL_UART_H_

#include "stm32g4xx_hal.h"
#include "stdlib.h"

/**
  * @}
  */


UART_HandleTypeDef* channel_list[256];


/**
  * @}
  */


/*	chip setting functions*/
uint16_t addChannel(UART_HandleTypeDef* huart);
void deleteChannel(uint8_t channel_num);

/**
  * @}
  */

/*	read byte value*/
uint8_t readByte(uint8_t channel_num);
uint8_t* readNByte(uint8_t channel_num, uint16_t size);

/**
  * @}
  */

/*	read byte value*/
HAL_StatusTypeDef writeByte(uint8_t channel_num, uint8_t pData);
HAL_StatusTypeDef writeNByte(uint8_t channel_num, uint8_t* pData, uint16_t size);


#endif /* INC_SERIAL_UART_H_ */
