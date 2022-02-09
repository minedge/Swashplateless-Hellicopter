/*
 * CyclicPitchControl.h
 *
 *  Created on: Jan 28, 2022
 *      Author: mined
 */

#ifndef INC_CONTROLLER_CYCLICPITCHCONTROL_H_
#define INC_CONTROLLER_CYCLICPITCHCONTROL_H_

#include "Sensor/as5147.h"

typedef struct _roll_pitch_yaw_throttle_command{
	float roll_cmd;
	float pitch_cmd;
	float yaw_cmd;
	float throttle_cmd;

	float limit_cmd;
}RPYT_CMD;

typedef struct _pwm_signal_command{
	int main_rotor;
	int tail_rotor;
}PWM_CMD;


float getOffset(RPYT_CMD rpyt_cmd, MOTOR mag);

float getAmplitude(RPYT_CMD rpyt_cmd);
float getShift(RPYT_CMD rpyt_cmd);

#endif /* INC_CONTROLLER_CYCLICPITCHCONTROL_H_ */
