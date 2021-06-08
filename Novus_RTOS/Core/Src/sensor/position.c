#include "sensor/position.h"

void updatePosition(MOTOR* motor, RC rc){

	motor->pre_ang = motor->ang;

	sens_time = HAL_GetTick() - sens_start;
	motor->ang = as5147_readPosition();
	sens_start = HAL_GetTick();

#if 1
	if(rc.aux3 > 1300){
		motor->ang = map(motor->ang, 180, 360, 0, 360);
		if(offset_flag == 1){
			motor->offset = map(motor->offset, 180, 360, 0, 360);
			offset_flag = 0;
		}
	}
#endif

	motor->ang -= motor->offset;
	if(motor->ang < 0)motor->ang += 360;

}

void updateRPM(MOTOR* motor){
	motor->pre_rpm = motor->rpm;
	motor->rpm = calcRPM(motor->pre_ang - motor->ang, 0.9/1000.);
	if(motor->rpm > RPM_MAX || motor->rpm < RPM_MIN){
		motor->rpm = motor->pre_rpm;
	}

#if 1
	motor->pre_lpf = motor->lpf;
	motor->lpf = LPF(motor->rpm, 0.9, motor->pre_rpm, motor->pre_lpf);
#endif
}

/**
  * @}
  */


/**
  * @brief  get offset for select front position
  * @param  none
  * @retval none
  */

void setOffset(MOTOR* motor){

	motor->offset = as5147_readPosition();

	offset_flag = 1;
}
