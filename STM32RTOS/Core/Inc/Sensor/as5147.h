/**
#pragma once
  ******************************************************************************
  * @file           : as5147.h
  * @brief          : Header for as5147.c file.
  *                   This file contains the common defines of the application.
  * @author NOVUS Graduation Project Team
  ******************************************************************************
  * @details
  *
  * Setting & Management
  * 
  * as5147 SETTING
  *     as5147_Init - as5147 initialize
  *		as5147_setZeroPosition - set angle to zeroposition
  *		as5147_readPosition - read angle from as5147
  *
  * as5147 R/W
  * 	registerRead - read register value
  * 	registerWrite - write register value
  * 	packCommandFrame - 
  * 	calcParity
  * 
  * angle caculator
  * 	calcRPM - calculate RPM
  *
  ******************************************************************************
  */

  /* Define to prevent recursive inclusion -------------------------------------*/

#ifndef INC_AS5147_H_
#define INC_AS5147_H_

#include "Serial/spi.h"
#include "Math/novus_math.h"

/**
  * @}
  */
  
/**
  * @}
  */

#define AS5047P_OPT_ENABLED 		1
#define AS5047P_OPT_DISABLED 		0

#define AS5047P_ACCESS_WRITE 		0
#define AS5047P_ACCESS_READ 		1

#define AS5047P_FRAME_PARD		( 1 << 15)
#define AS5047P_FRAME_EF 		( 1 << 14)
#define AS5047P_FRAME_DATA		0x3FFF

#define AS5047P_ABIRES_100 	100
#define AS5047P_ABIRES_200 	200
#define AS5047P_ABIRES_400 	400
#define AS5047P_ABIRES_800 	800
#define AS5047P_ABIRES_1200 	1200
#define AS5047P_ABIRES_1600 	1600
#define AS5047P_ABIRES_2000 	2000
#define AS5047P_ABIRES_4000 	4000
#define AS5047P_ABIRES_1024 	1024
#define AS5047P_ABIRES_2048 	2048
#define AS5047P_ABIRES_4096 	4096

/**
  * @}
  */

// --- Volatile registers
#define AS5047P_NOP          	0x0000
#define AS5047P_ERRFL        	0x0001
#define AS5047P_PROG        	0x0003
#define AS5047P_DIAAGC       	0x3FFC
#define AS5047P_MAG          	0x3FFD
#define AS5047P_ANGLEUNC     	0x3FFE
#define AS5047P_ANGLECOM     	0x3FFF

/**
  * @}
  */

// --- Non-volatile registers
#define AS5047P_ZPOSM        	0x0016
#define AS5047P_ZPOSL        	0x0017
#define AS5047P_SETTINGS1    	0x0018
#define AS5047P_SETTINGS2    	0x0019

/**
  * @}
  */

// --- Fields in registers
#define AS5047P_ERRFL_PARERR		( 1 << 2)
#define AS5047P_ERRFL_INVCOMM		( 1 << 1)
#define AS5047P_ERRFL_FRERR		( 1 << 0)
#define AS5047P_PROG_PROGVER		( 1 << 6)
#define AS5047P_PROG_PROGOTP		( 1 << 3)
#define AS5047P_PROG_OTPREF		( 1 << 2)
#define AS5047P_PROG_PROGEN		( 1 << 0)
#define AS5047P_DIAAGC_MAGL		( 1 << 11)
#define AS5047P_DIAAGC_MAGH		( 1 << 10)
#define AS5047P_DIAAGC_COF		( 1 << 9)
#define AS5047P_DIAAGC_LF		( 1 << 8)
#define AS5047P_DIAAGC_AGC		( 0x00FF << 0)
#define AS5047P_MAG_CMAG		( 0x3FFF << 0)
#define AS5047P_ANGLEUNC_CORDICANG	( 0x3FFF << 0)
#define AS5047P_ANGLECOM_DAECANG	( 0x3FFF << 0)
#define AS5047P_ZPOSM_ZPOSM		( 0x00FF << 0)
#define AS5047P_ZPOSL_COMP_H_ERR_EN	( 1 << 7)
#define AS5047P_ZPOSL_COMP_I_ERR_EN	( 1 << 6)
#define AS5047P_ZPOSL_ZPOSL		( 0x003F << 0)
#define AS5047P_SETTINGS1_BIT0		( 1 << 0)
#define AS5047P_SETTINGS1_NOISESET	( 1 << 1)
#define AS5047P_SETTINGS1_DIR		( 1 << 2)
#define AS5047P_SETTINGS1_UVW_ABI	( 1 << 3)
#define AS5047P_SETTINGS1_DAECDIS	( 1 << 4)
#define AS5047P_SETTINGS1_ABIBIN	( 1 << 5)
#define AS5047P_SETTINGS1_DATASEL	( 1 << 6)
#define AS5047P_SETTINGS1_PWMON		( 1 << 7)
#define AS5047P_SETTINGS2_UVWPP		( 0x0007 << 0)
#define AS5047P_SETTINGS2_HYS		( 0x0003 << 3)
#define AS5047P_SETTINGS2_ABIRES	( 0x0007 << 5)

/**
  * @}
  */
  


/**
  * @defgroup Controller_Setup
  * @brief Minimum & Maximum rang of RPM
  */
#define RPM_MIN 0
#define RPM_MAX 6000

/**
  * @}
  */

/**
 *  @brief spi Transaction command & Write & Read Frame bit
 *         15bit : parity bit
 *         14bit : R/W
 *         13:0bit : data
 */
typedef struct __attribute__ ((packed)) {
	uint16_t data:14;
	uint16_t rw:1;
	uint16_t pard:1;
} FrameBit;

/**
 *  @brief spi Transaction command & Write & Read Frame
 *  @details
 */
typedef union{
	uint16_t raw;
	FrameBit values;
}Frame;

/**
  * @}
  */


/*
* @brief motor information struct
*/

typedef struct motor_inforamtion{
	float offset;

	float pre_ang;
	float ang;				/*< 모터 각도		*/

	float rad;

	float pre_rpm;
	float rpm;					/*< 모터 속도		*/

	float pre_lpf;
	float lpf;

	float pre_pwm;
	float pwm;				/*< Output			*/
}MOTOR;

/**
  * @}
  */

/*
* @brief Global variable motor have motor information
*/

volatile uint32_t sens_time;
volatile uint32_t sens_start;

volatile uint32_t exang_time;
volatile uint32_t exang_start;

/*
* @brief as5147 must get chip_num through as5147 _Init()
*        when call addChip() in spi.h will return chip_num 0 to 255
*		 chip_num use every times of SPI protocal communication
*/
uint16_t as5147_chip_num;

/**
  * @}
  */

/* encoder functions */

int8_t as5147_Init(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIO_port, uint16_t GPIO_num);
int8_t as5147_setZeroPosition();
float as5147_readPosition();

/**
  * @}
  */

/*
* @brief Position update fuction
*/

void updatePosition(MOTOR* motor);
void updateRPM(MOTOR* motor);
void setOffset(MOTOR* motor);


/**
  * @}
  */

/* register R/W functions */

uint16_t registerRead(uint16_t resgister_address);
int8_t registerWrite(uint16_t resgister_address, uint16_t data);

/**
  * @}
  */

/* register Frame functions*/

Frame packCommandFrame(uint16_t data, uint8_t rw);
uint8_t calcParity(uint16_t data);

/**
  * @}
  */

/**
  * @}
  */

#endif /* INC_AS5147_H_ */
