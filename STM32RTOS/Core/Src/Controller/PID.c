/*
 * PID.c
 *
 *  Created on: 2021. 10. 21.
 *      Author: mined
 */


#include "Controller/PID.h"


void setPIDGain(float Kp, float Ki, float Kd, uint8_t spc){
	switch(spc){
	case 0:
		P_gain.Kp = Kp;
		P_gain.Ki = Ki;
		P_gain.Kd = Kd;
		break;
	case 1:
		Q_gain.Kp = Kp;
		Q_gain.Ki = Ki;
		Q_gain.Kd = Kd;
		break;
	case 2:
		R_gain.Kp = Kp;
		R_gain.Ki = Ki;
		R_gain.Kd = Kd;
		break;
	case 3:
		pi_gain.Kp = Kp;
		pi_gain.Ki = Ki;
		pi_gain.Kd = Kd;
		break;
	case 4:
		theta_gain.Kp = Kp;
		theta_gain.Ki = Ki;
		theta_gain.Kd = Kd;
		break;
	default:
		break;
	}

	start_time = HAL_GetTick();
}

PQR_CMD PIDController(PQR_CMD cmd, IMU imu_data){
	PQR_CMD pid_cmd;
	pid_cmd.P_cmd = 0;
	pid_cmd.Q_cmd = 0;
	pid_cmd.R_cmd = 0;
	pid_cmd.pi = 0;
	pid_cmd.theta = 0;
	pid_cmd.psi = 0;

	P_err.pre_error = P_err.error;
	Q_err.pre_error = Q_err.error;
	R_err.pre_error = R_err.error;;
	pi_err.pre_error = pi_err.error;
	theta_err.pre_error = theta_err.error;

	P_err.error = imu_data.P - cmd.P_cmd;
	Q_err.error = imu_data.Q - cmd.Q_cmd;
	R_err.error = imu_data.R - cmd.R_cmd;
	pi_err.error = imu_data.pi - cmd.pi;
	theta_err.error = imu_data.theta - cmd.theta;

	P_err.sum_error += P_err.error;
	Q_err.sum_error += Q_err.error;
	R_err.sum_error += R_err.error;
	pi_err.sum_error += pi_err.error;
	theta_err.sum_error += theta_err.error;

	loop_time = HAL_GetTick() - start_time;
	pid_cmd.P_cmd = P_err.error * P_gain.Kp + P_err.sum_error * P_gain.Ki * loop_time + ((P_err.error-P_err.pre_error) / loop_time) * P_gain.Kd;
	pid_cmd.Q_cmd = Q_err.error * Q_gain.Kp + Q_err.sum_error * Q_gain.Ki * loop_time + ((Q_err.error-Q_err.pre_error) / loop_time) * Q_gain.Kd;
	pid_cmd.R_cmd = R_err.error * R_gain.Kp + R_err.sum_error * R_gain.Ki * loop_time + ((R_err.error-R_err.pre_error) / loop_time) * R_gain.Kd;
	pid_cmd.pi = pi_err.error * pi_gain.Kp + pi_err.sum_error * pi_gain.Ki * loop_time + ((pi_err.error-pi_err.pre_error) / loop_time) * pi_gain.Kd;
	pid_cmd.theta = theta_err.error * theta_gain.Kp + theta_err.sum_error * theta_gain.Ki * loop_time + ((theta_err.error-theta_err.pre_error) / loop_time) * theta_gain.Kd;
	start_time = HAL_GetTick();

	if(pid_cmd.P_cmd > 85) pid_cmd.P_cmd = 85;
	if(pid_cmd.P_cmd < -85) pid_cmd.P_cmd = -85;

	if(pid_cmd.Q_cmd > 85) pid_cmd.Q_cmd = 85;
	if(pid_cmd.Q_cmd < -85) pid_cmd.Q_cmd = -85;

	return pid_cmd;
}
