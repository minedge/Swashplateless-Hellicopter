/**
 * 	@file spi.c
 * 	@author NOVUS Graduation Project Team
 * 	@brief	Sensor SPI COnnection Setup module
 * 	@date 2021-4-15
 * 	@version 0.0.1
 */
#include "sensor/spi.h"

/**
  * @}
  */


/**
  * @}
  */

/*
* @brief SPI Protocoal information list of every single chip which use spi communication
*/
static CHIP_LIST chip_list[256];

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
  * success 1, if fail return false
  */
uint8_t addChip(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIO_port, uint16_t GPIO_num){
	uint8_t chip_num = 0;
	uint8_t index = 0;

	for(; index < 256; index++){
		if(chip_list[index].CS_pin == 0){
			chip_num = index;
			break;
		}
	}

	if(index != 255){
		chip_list[index].hspi = hspix;
		chip_list[index].CS_port = GPIO_port;
		chip_list[index].CS_pin = GPIO_num;
		HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_SET);

		return 0;
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
void deleteChip(uint16_t chip_num){
	chip_list[chip_num].hspi = 0;
	chip_list[chip_num].CS_port = 0;
	chip_list[chip_num].CS_pin = 0;
}


/**
  * @}
  */


/**
  * @brief  read register value
  * @param  chip_num spi chip number
  * @retval data in register
  */
uint16_t read2ByteRegister(uint8_t chip_num){
	uint16_t read_data = 0;
	uint16_t* pbuffer = &read_data;

	HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef state = HAL_SPI_Receive(chip_list[chip_num].hspi, (uint8_t*)pbuffer, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_SET);

	if(state != HAL_OK){
		return state;
	}

	return read_data;
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
HAL_StatusTypeDef write2ByteRegister(uint16_t* command, uint8_t chip_num){

	HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef state = HAL_SPI_Transmit(chip_list[chip_num].hspi, (uint8_t*)command, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_SET);

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
uint16_t readWrite2ByteRegister(uint16_t* command, uint8_t chip_num){
	uint16_t read_data = 0;
	uint16_t* pbuffer = &read_data;

	HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_RESET);

	HAL_StatusTypeDef state = HAL_SPI_TransmitReceive(chip_list[chip_num].hspi, (uint8_t*)command, (uint8_t*)pbuffer, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(chip_list[chip_num].CS_port, chip_list[chip_num].CS_pin, GPIO_PIN_SET);

	if(state != HAL_OK){
		return -1;
	}

	return read_data;
}
