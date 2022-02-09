/*
 * CyclicPitchControl.c
 *
 *  Created on: Jan 28, 2022
 *      Author: mined
 */


#include "Controller/CyclicPitchControl.h"


float getOffset(RPYT_CMD rpyt_cmd, MOTOR mag){
	return sin(mag.rad + getShift(rpyt_cmd)) * getAmplitude(rpyt_cmd);
}

float getAmplitude(RPYT_CMD rpyt_cmd){
	float amplitude = sqrt(pow(rpyt_cmd.roll_cmd,2)+pow(rpyt_cmd.pitch_cmd,2));

	if(amplitude > rpyt_cmd.limit_cmd)
		amplitude = rpyt_cmd.limit_cmd;

	return amplitude;
}

float getShift(RPYT_CMD rpyt_cmd){
	float shift = atan2(rpyt_cmd.roll_cmd, rpyt_cmd.pitch_cmd);

	shift += PIE/2;

	return shift;
}
