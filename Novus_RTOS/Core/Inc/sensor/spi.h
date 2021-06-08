#pragma once
/**
 * @file		: spi.h
 * @brief		: Header fpr spi.c file.
 *				  This file contains the common defines of the application.
 * @author NOVUS Graduation Project Team
 *  *************************************************************************
 *  @attention
 *  Setting & Management
 *	Chip Select
 * 		addChip
 * 		deleteChip
 * 	Read & Write Reg
 * 		readByteRegister
 * 		read2ByteRegister
 * 		writeByteRegister
 * 		write2ByteRegister
 * 		readWriteByteRegister
 * 		readWrite2ByteRegister
 */


#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32g4xx_hal.h"

/**
  * @}
  */


/**
  * @}
  */

/*
* @brief SPI Protocoal information of every single chip which use spi communication
*/


typedef struct chip_info_list{
	SPI_HandleTypeDef* hspi;		/*!< pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.*/
	GPIO_TypeDef* CS_port;			/*!< GPIO_TypeDef struct  chip select port*/
	uint16_t CS_pin;				/*!<  Chip Select*/
}CHIP_LIST;	


/**
  * @}
  */


/**
  * @}
  */


/*	chip setting functions*/
uint8_t addChip(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIO_port, uint16_t GPIO_num);
void deleteChip(uint16_t chip_num);


/**
  * @}
  */


/*	read register's value*/
uint8_t readByteRegister(uint8_t chip_num);
uint16_t read2ByteRegister(uint8_t chip_num);


/**
  * @}
  */


/*	write data to register*/
HAL_StatusTypeDef writeByteRegister(uint8_t* command, uint8_t chip_num);
HAL_StatusTypeDef write2ByteRegister(uint16_t* command, uint8_t chip_num);


/**
  * @}
  */


/* read & write simultaneously to register */
uint8_t readWriteByteRegister(uint8_t* command, uint8_t chip_num);
uint16_t readWrite2ByteRegister(uint16_t* command, uint8_t chip_num);

#endif /* INC_SPI_H_ */
