/*
 * WT931.h
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */

#ifndef INC_SENSOR_WT931_H_
#define INC_SENSOR_WT931_H_

#include <stdlib.h>
#include "Serial/I2C.h"


/**
  * @}
  */


#define WT931_CALSW			0x01					// Calibration
#define WT931_RSW			0x02					// Return data content
#define WT931_RATE			0x03					// Return data Speed
#define WT931_BAUD			0x04					// Baud rate

/**
  * @}
  */


#define WT931_AXOFFSET		0x05					// X axis Acceleration bias
#define WT931_AYOFFSET		0x06					// Y axis Acceleration bias
#define WT931_AZOFFSET		0x07					// Z axis Acceleration bias
#define WT931_GXOFFSET		0x08					// X axis angular velocity bias
#define WT931_GYOFFSET		0x09					// Y axis angular velocity bias
#define WT931_GZOFFSET		0x0A					// Z axis angular velocity bias
#define WT931_HXOFFSET		0x0B					// X axis Magnetic bias
#define WT931_HYOFFSET		0x0C					// Y axis Magnetic bias
#define WT931_HZOFFSET		0x0D					// Z axis Magnetic bias

/**
  * @}
  */


#define WT931_D0MODE		0x0E					// D0 mode
#define WT931_D1MODE		0x0F					// D1 mode
#define WT931_D2MODE		0x10					// D2 mode
#define WT931_D3MODE		0x11					// D3 mode
#define WT931_D0PWMH		0x12					// D0PWM High-level width
#define WT931_D1PWMH		0x13					// D1PWM High-level width
#define WT931_D2PWMH		0x14					// D2PWM High-level width
#define WT931_D3PWMH		0x15					// D3PWM High-level width
#define WT931_D0PWMT		0x16					// D0PWM Period
#define WT931_D1PWMT		0x17					// D1PWM Period
#define WT931_D2PWMT		0x18					// D2PWM Period
#define WT931_D3PWMT		0x19					// D3PWM Period

/**
  * @}
  */


#define WT931_IICADDR		0x1A					// IIC address
#define WT931_LEDOFF		0x1B					// Turn off LED
#define WT931_GPSBAUD		0x1C					// GPS baud rate

/**
  * @}
  */


#define WT931_YYMM			0x30					// Year, Month
#define WT931_DDHH			0x31					// Day, Hour
#define WT931_MMSS			0x32					// Minute, Second
#define WT931_MS			0x33					// Millisecond

/**
  * @}
  */


#define WT931_AX			0x34					// X axis Acceleration
#define WT931_AY			0x35					// Y axis Acceleration
#define WT931_AZ			0x36					// Z axis Acceleration

#define WT931_GX			0x37					// X axis angular velocity
#define WT931_GY			0x38					// Y axis angular velocity
#define WT931_GZ			0x39					// Z axis angular velocity

#define WT931_HX			0x3A					// X axis Magnetic
#define WT931_HY			0x3B					// Y axis Magnetic
#define WT931_HZ			0x3C					// Z axis Magnetic

#define WT931_ROLL			0x3D					// X axis Angle
#define WT931_PITCH			0x3E					// Y axis Angle
#define WT931_YAW			0x3F					// Z axis Angle
#define WT931_TEMP			0x40					// Temperature

/**
  * @}
  */


#define WT931_D0STATUS		0x41					// D0 Status
#define WT931_D1STATUS		0x42					// D1 Status
#define WT931_D2STATUS		0x43					// D2 Status
#define WT931_D3STATUS		0x44					// D3 Status

/**
  * @}
  */


#define WT931_PRESSURE_L	0x45					// Pressure Low Byte
#define WT931_PRESSURE_H	0x46					// Pressure High Byte

/**
  * @}
  */


#define WT931_HEIGHT_L		0x47					// Height Low Byte
#define WT931_HEIGHT_H		0x48					// Height High Byte

/**
  * @}
  */


#define WT931_LON_L			0x49					// Longitude Low Byte
#define WT931_LON_H			0x4A					// Longitude High Byte
#define WT931_LAT_L			0x4B					// Latitude Low Byte
#define WT931_LAT_H			0x4C					// Latitude High Byte

/**
  * @}
  */


#define WT931_GPS_HEIGHT	0x4D					// GPS Height
#define WT931_GPS_YAW		0x4E					// GPS YAW
#define WT931_GPS_V_L		0x4F					// GPS speed Low Byte
#define WT931_GPS_V_H		0x50					// GPS speed High Byte

/**
  * @}
  */


#define WT931_Q0			0x51					// Quaternion Q0
#define WT931_Q1			0x52					// Quaternion Q1
#define WT931_Q2			0x53					// Quaternion Q2
#define WT931_Q3			0x54					// Quaternion Q3

/**
  * @}
  */


/**
  * @}
  */


typedef struct _imu_data{
	float pi;
	float theta;
	float psi;

	float P;
	float Q;
	float R;
}IMU;


/**
  * @}
  */

uint16_t wt931_chip_num;



/**
  * @}
  */

int8_t wt931_Init(I2C_HandleTypeDef* hi2cx);

IMU readIMU();

#endif /* INC_SENSOR_WT931_H_ */
