/*
 * controller.h
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */

#ifndef INC_CONTROLLER_CYCLICPITCHCONTROLLER_H_
#define INC_CONTROLLER_CYCLICPITCHCONTROLLER_H_

#include "Sensor/as5147.h"

#define PWM_START 1000
#define PWM_MIN 1000
#define PWM_MAX 1850

typedef struct _PWM_command{
	float throttle;
	float tail;
	float cyclic_pitch;
}PWM_CMD;

typedef struct _P_Q_R_command{
	float P_cmd;
	float Q_cmd;
	float R_cmd;

	float pi;
	float theta;
	float psi;
}PQR_CMD;

float amplitude_gain;
float cyclic_shift_gain;

void setAmplitudeGain(float gain);
void setShiftGain(float gain);

float setAmplitude();
float setCyclicShift();



#endif /* INC_CONTROLLER_CYCLICPITCHCONTROLLER_H_ */
