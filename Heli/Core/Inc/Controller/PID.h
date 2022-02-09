/*
 * PID.h
 *
 *  Created on: Jan 28, 2022
 *      Author: mined
 */

#ifndef INC_CONTROLLER_PID_H_
#define INC_CONTROLLER_PID_H_

enum _pid_modes{
	P = 1,
	PI,
	PD,
	PID
};

typedef struct _pid_controller_gain{
	float p;
	float i;
	float d;
}PID_GAIN;

typedef struct _pid_controller_block{
	int mode;

	PID_GAIN k;

	float pre_error;
	float integrated_error;

	float loop_time;
	float loop_start;
}PID_BLOCK;

extern volatile unsigned long gTick;

PID_BLOCK pid_list[20];

int regPID(int mode, float gain_p, float gain_i, float gain_d);

float PIDoutput(int id, float state, float target);

void resetState(int id);

#endif /* INC_CONTROLLER_PID_H_ */
