/**
 * 	@file spi.c
 * 	@author NOVUS Graduation Project Team
 * 	@brief	Sensor SPI COnnection Setup module
 * 	@date 2021-4-15
 * 	@version 0.0.1
 */
#include "Serial/spi.h"

/**
  * @}
  */


/**
  * @}
  */

/**
  * @brief  initialize chip
  * @param  hspix 	pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param	GPIO_port GPIO init structure definition
  * @param	GPIO_num GPIO pin number
  * @retval boolean
  * success chip_num, if fail return false
  */
uint16_t addSPIChip(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIO_port, uint16_t GPIO_num){
	uint8_t chip_num = 0;
	uint8_t index = 0;

	for(index = 0; index < 256; index++){
		if(SPI_chip_list[index].CS_pin == 0 || SPI_chip_list[index].hspi == hspix){
			chip_num = index;
			break;
		}
	}

	if(index != 256){
		SPI_chip_list[chip_num].hspi = hspix;
		SPI_chip_list[chip_num].CS_port = GPIO_port;
		SPI_chip_list[chip_num].CS_pin = GPIO_num;
		HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_SET);

		return chip_num;
	}else{
		return -1;
	}
}


/**
  * @}
  */


/**
  * @brief  reset chip setting to zero
  * @param  chip_num chip number
  * @retval none
  */
void deleteSPIChip(uint16_t chip_num){
	SPI_chip_list[chip_num].hspi = 0;
	SPI_chip_list[chip_num].CS_port = 0;
	SPI_chip_list[chip_num].CS_pin = 0;
}


/**
  * @}
  */


/**
  * @brief  read register value
  * @param  chip_num spi chip number
  * @retval data in register
  */
uint16_t SPI_read2ByteRegister(uint8_t chip_num, uint16_t* pRxData){
	//uint8_t pbuffer[2] = { 0, };
	//uint16_t* read_data = (uint16_t*)pbuffer;

	HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_RESET);

	HAL_SPI_Receive(SPI_chip_list[chip_num].hspi, (uint8_t*)pRxData, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_SET);

	//return *read_data;
	return 0;
}


/**
  * @}
  */


/**
  * @brief  write register value
  * @param	command data what will write in register
  * @param  chip_num spi chip number
  * @retval HAL status
  */
HAL_StatusTypeDef SPI_write2ByteRegister(uint16_t* command, uint8_t chip_num){

	HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef state = HAL_SPI_Transmit(SPI_chip_list[chip_num].hspi, (uint8_t*)command, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_SET);

	return state;
}


/**
  * @}
  */


/**
  * @brief  read & write simultaneously to register
  * @param	command data what will write in register
  * @param  chip_num spi chip number
  * @retval HAL status
  */
uint16_t SPI_readWrite2ByteRegister(uint16_t* command, uint8_t chip_num){
	uint8_t pbuffer[2];
	uint16_t* read_data = (uint16_t*)pbuffer;

	HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(SPI_chip_list[chip_num].hspi, (uint8_t*)command, (uint8_t*)read_data, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI_chip_list[chip_num].CS_port, SPI_chip_list[chip_num].CS_pin, GPIO_PIN_SET);

	return *read_data;
}
