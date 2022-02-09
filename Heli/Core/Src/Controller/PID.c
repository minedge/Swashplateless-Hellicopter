/*
 * PID.c
 *
 *  Created on: Jan 28, 2022
 *      Author: mined
 */

#include "Controller/PID.h"

int regPID(int mode, float gain_p, float gain_i, float gain_d){

	int index = 0;

	for(;pid_list[index].mode != 0; index++){}

	pid_list[index].mode = mode;
	pid_list[index].k.p = gain_p;
	pid_list[index].k.i = gain_i;
	pid_list[index].k.d = gain_d;

	pid_list[index].loop_time = 1;
	pid_list[index].loop_start = 0;
	pid_list[index].pre_error = 0;
	pid_list[index].integrated_error = 0;

	return index;
}

float PIDoutput(int id, float state, float target){
	float error = target - state;
	float pid_output = 0;

	PID_BLOCK pid = pid_list[id];

	switch(pid.mode){
	case P:
		pid_output = error*pid.k.p;
		break;
	case PI:
		pid.integrated_error += error;
		pid_output = error*pid.k.p + pid.integrated_error*pid.k.i;
		break;
	case PD:
		pid.loop_time = (gTick - pid.loop_start)/100000.;
		pid_output = error*pid.k.p + ((error - pid.pre_error)/pid.loop_time)*pid.k.d;
		pid.loop_start = gTick;
		break;
	case PID:
		pid.integrated_error += error;
		pid.loop_time = (gTick - pid.loop_start)/100000.;
		pid_output = error*pid.k.p + pid.integrated_error*pid.k.i + ((error - pid.pre_error)/pid.loop_time)*pid.k.d;
		pid.loop_start = gTick;
		break;
	}

	return pid_output;
}



void resetState(int id){
	pid_list[id].loop_time = 1;
	pid_list[id].loop_start = 0;
	pid_list[id].pre_error = 0;
	pid_list[id].integrated_error = 0;
}
