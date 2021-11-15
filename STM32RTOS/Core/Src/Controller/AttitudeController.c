/*
 * controller.c
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */


#include <Controller/AttitudeController.h>


PQR_CMD CommandGennerate(IMU imu_data, RC rc_cmd){
	PQR_CMD pqr_cmd;

	pqr_cmd.P_cmd = rc_cmd.d_pi - sin(imu_data.theta*3.141592/180.)*rc_cmd.d_psi;
	pqr_cmd.Q_cmd = cos(imu_data.pi*3.141592/180.)*rc_cmd.d_theta + cos(imu_data.theta*3.141592/180.)*sin(imu_data.pi*3.141592/180.)*rc_cmd.d_psi;
	pqr_cmd.R_cmd = cos(imu_data.theta*3.141592/180.)*cos(imu_data.pi*3.141592/180.)*rc_cmd.d_psi - sin(imu_data.pi*3.141592/180.)*rc_cmd.d_theta;

	pqr_cmd.pi = rc_cmd.pi;
	pqr_cmd.theta = rc_cmd.theta;
	pqr_cmd.psi = 0;

	return pqr_cmd;
}
