/*
 * PID.h
 *
 *  Created on: 2021. 10. 21.
 *      Author: mined
 */

#ifndef INC_CONTROLLER_PID_H_
#define INC_CONTROLLER_PID_H_

#include "Sensor/WT931.h"
#include "Controller/CyclicPitchController.h"

typedef struct _pid_gain{
	float Kp;
	float Ki;
	float Kd;
}PID_GAIN;

typedef struct _error_struct{
	float error;
	float pre_error;
	float sum_error;
}PID_ERROR;

PID_GAIN P_gain, Q_gain, R_gain;
PID_GAIN pi_gain, theta_gain;

PID_ERROR P_err, Q_err, R_err;
PID_ERROR pi_err, theta_err;

uint16_t loop_time, start_time;

void setPIDGain(float Kp, float Ki, float Kd, uint8_t spc);

PQR_CMD PIDController(PQR_CMD cmd, IMU imu_data);

#endif /* INC_CONTROLLER_PID_H_ */
