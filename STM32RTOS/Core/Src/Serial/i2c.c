/*
 * i2c.c
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */


#include "Serial/i2c.h"

/**
  * @}
  */


/**
  * @}
  */


/*	chip setting functions*/

uint16_t addI2CChip(I2C_HandleTypeDef* hi2c, uint16_t dev_addr, uint16_t mem_addr_size){
	uint8_t chip_num = 0;
	uint8_t index = 0;

	for(index = 0; index < 256; index++){
		if(i2c_chip_list[index].dev_addr == 0 || i2c_chip_list[index].hi2c == hi2c){
			chip_num = index;
			break;
		}
	}

	if(index != 256){
		i2c_chip_list[chip_num].hi2c = hi2c;
		i2c_chip_list[chip_num].dev_addr = dev_addr;
		i2c_chip_list[chip_num].mem_addr_size = mem_addr_size;

		return chip_num;
	}else{
		return -1;
	}
}

/**
  * @}
  */

void deleteI2CChip(uint16_t chip_num){
	i2c_chip_list[chip_num].hi2c = 0;
	i2c_chip_list[chip_num].dev_addr = 0;
	i2c_chip_list[chip_num].mem_addr_size = 0;
}

/**
  * @}
  */

/*	read register's value*/
uint8_t I2C_readByteRegister(uint8_t chip_num, uint16_t mem_addr){
	uint8_t buffer = 0;

	//HAL_I2C_Master_Transmit(i2c_chip_list[chip_num].hi2c, i2c_chip_list[chip_num].dev_addr, (uint8_t*)&mem_addr, 1, HAL_TIMEOUT);
	HAL_I2C_Mem_Read(i2c_chip_list[chip_num].hi2c, (i2c_chip_list[chip_num].dev_addr & 0x7F) << 1, mem_addr, I2C_MEMADD_SIZE_8BIT, &buffer, 1, HAL_TIMEOUT);

	return buffer;
}

/**
  * @}
  */

uint16_t I2C_read2ByteRegister(uint8_t chip_num, uint16_t mem_addr){
	uint8_t buffer[2] = { 0, };

	//HAL_I2C_Master_Transmit(i2c_chip_list[chip_num].hi2c, (i2c_chip_list[chip_num].dev_addr & 0x7f) << 1, (uint8_t*)&mem_addr, 1, HAL_TIMEOUT);

	//HAL_I2C_Master_Receive(i2c_chip_list[chip_num].hi2c, (i2c_chip_list[chip_num].dev_addr & 0x7f) << 1, buffer, 2, HAL_TIMEOUT);

	HAL_I2C_Mem_Read(i2c_chip_list[chip_num].hi2c, (i2c_chip_list[chip_num].dev_addr & 0x7f) << 1, mem_addr, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_TIMEOUT);

	uint16_t data = buffer[1] <<8 | buffer[0];

	return data;
}

/**
  * @}
  */

/*	write data to register*/
HAL_StatusTypeDef I2C_writeByteRegister(uint8_t chip_num, uint16_t mem_addr, uint8_t* data){
	return HAL_I2C_Mem_Write(i2c_chip_list[chip_num].hi2c, i2c_chip_list[chip_num].dev_addr, mem_addr, i2c_chip_list[chip_num].mem_addr_size, data, 1, HAL_TIMEOUT);
}

/**
  * @}
  */
