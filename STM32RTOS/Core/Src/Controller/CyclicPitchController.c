/*
 * controller.c
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */


#include <Controller/CyclicPitchController.h>

void setAmplitudeGain(float gain){
	amplitude_gain = gain;
}
void setShiftGain(float gain){
	cyclic_shift_gain = gain;
}

float setAmplitude(PWM_CMD pwm_cmd, PQR_CMD pqr_cmd){
	float amplitude_v = sqrt(pow(pqr_cmd.P_cmd, 2) + pow(pqr_cmd.Q_cmd, 2));
	float amplitude_a = sqrt(pow(pqr_cmd.pi, 2) + pow(pqr_cmd.theta, 2));

	float amplitude = (amplitude_v + amplitude_a)/2;
	amplitude += pow(pwm_cmd.throttle - PWM_MIN, 2) * amplitude_gain;

	return amplitude;
}

float setCyclicShift(MOTOR mag_data, PQR_CMD pqr_cmd){
	if((pqr_cmd.Q_cmd > -0.1 && pqr_cmd.Q_cmd < 0.1) && (pqr_cmd.P_cmd > -0.1 && pqr_cmd.P_cmd < 0.1)) // stick zero input margin error
		return -1;

	float cmd_mix = pqr_cmd.Q_cmd / sqrt(pow(pqr_cmd.P_cmd, 2) + pow(pqr_cmd.Q_cmd, 2));

	float start_rad = 0;
	if(pqr_cmd.P_cmd < 0){
			start_rad = (2*3.141592) - acos(cmd_mix);
	}
	else{
			start_rad = acos(cmd_mix);
	}

	return start_rad;
}
