/*
 * WT931.c
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */


#include "Sensor/WT931.h"

int8_t wt931_Init(I2C_HandleTypeDef* hi2cx){
	wt931_chip_num = addI2CChip(hi2cx, 0x50, 1);

	if(wt931_chip_num < 0){
		return -1;
	}

	return 0;
}

IMU readIMU(){
	IMU imu;

	float angle[3] = { 0, };

	angle[0] = (float)I2C_read2ByteRegister(wt931_chip_num, WT931_ROLL);
	angle[1] = (float)I2C_read2ByteRegister(wt931_chip_num, WT931_PITCH);
	angle[2] = (float)I2C_read2ByteRegister(wt931_chip_num, WT931_YAW);

#if 1
	imu.pi = angle[0] / 32768 * 180;
	if(imu.pi > 180) imu.pi -= 360;
	imu.theta = angle[1] / 32768 * 180;
	if(imu.theta > 180) imu.theta -= 360;
	imu.psi = angle[2] / 32768 * 180;
	if(imu.psi > 180) imu.psi -= 360;
#else
	float temp_pi = angle[0] / 32768 * 180;
	if(temp_pi > 180) temp_pi -= 360;
	float temp_theta = angle[1] / 32768 * 180;
	if(temp_theta > 180) temp_theta -= 360;
	float temp_psi = angle[2] / 32768 * 180;
	if(temp_psi > 180) temp_psi -= 360;

	imu.pi = temp_pi;
	imu.theta = temp_theta;
	imu.psi = temp_psi;
#endif

#if 1
	float angular_velocity[3] = { 0, };

	angular_velocity[0] = (float)I2C_read2ByteRegister(wt931_chip_num, WT931_GX);
	angular_velocity[1] = (float)I2C_read2ByteRegister(wt931_chip_num, WT931_GY);
	angular_velocity[2] = (float)I2C_read2ByteRegister(wt931_chip_num, WT931_GZ);

	imu.P = angular_velocity[0] / 32768 * 2000;
	if(imu.P > 2000) imu.P -= 4000;
	imu.Q = angular_velocity[1] / 32768 * 2000;
	if(imu.Q > 2000) imu.Q -= 4000;
	imu.R = angular_velocity[2] / 32768 * 2000;
	if(imu.R > 2000) imu.R -= 4000;

	imu.P = imu.P * -1;
	imu.Q = imu.Q * -1;

#else
	imu.P = 0;
	imu.Q = 0;
	imu.R = 0;
#endif

	return imu;
}
