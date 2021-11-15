/*
 * i2c.h
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */

#ifndef INC_SERIAL_I2C_H_
#define INC_SERIAL_I2C_H_

#include "stm32g4xx_hal.h"

/**
  * @}
  */



/**
  * @}
  */

/*
* @brief I2C Protocoal information of every single chip which use i2c communication
*/
typedef struct i2c_chip_info_list{
	I2C_HandleTypeDef* hi2c;		/*!< pointer to a I2C_HandleTypeDef structure that contains the configuration information for I2C module.*/
	uint16_t dev_addr;				/*!< Device chip address*/
	uint16_t mem_addr_size;			/*!< Device register address size*/
}I2C_CHIP_LIST;


/**
  * @}
  */

/*
* @brief I2C Protocoal information list of every single chip which use i2c communication
*/
I2C_CHIP_LIST i2c_chip_list[256];


/**
  * @}
  */


/**
  * @}
  */


/*	chip setting functions*/
uint16_t addI2CChip(I2C_HandleTypeDef* hi2c, uint16_t dev_addr, uint16_t mem_add_size);
void deleteI2CChip(uint16_t chip_num);

/**
  * @}
  */

/*	read register's value*/
uint8_t I2C_readByteRegister(uint8_t chip_num, uint16_t mem_addr);
uint16_t I2C_read2ByteRegister(uint8_t chip_num, uint16_t mem_addr);

/**
  * @}
  */

/*	write data to register*/
HAL_StatusTypeDef I2C_writeByteRegister(uint8_t chip_num, uint16_t mem_addr, uint8_t* data);

/**
  * @}
  */


#endif /* INC_SERIAL_I2C_H_ */
