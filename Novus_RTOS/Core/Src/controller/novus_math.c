#include "controller/novus_math.h"

float map(float target, int from_min, int from_max, int to_min, int to_max){
    float mult = (float)(to_max - to_min) / (float)(from_max - from_min);
    target = target - from_min;
    return to_min + (target * mult);
}

float getLPF_RPM(float raw_rpm, float pre_rpm, float loop_time){

	float lpf_rpm = ((LPF_GAIN * pre_rpm) + (loop_time * raw_rpm)) / (LPF_GAIN + loop_time);

	return lpf_rpm;
}

float LPF(float raw_rpm, float loop_time, float pre_rpm, float pre_lpf){

	float a1,b0,b1,w0;

	w0 = 2*3.14*LPF_GAIN;
	a1 = (w0 - 2*loop_time)/(2*loop_time + w0);
	b0 = w0/(2*loop_time + w0);
	b1 = b0;


	return (b0*raw_rpm) + (b1*pre_rpm) - (a1*pre_lpf);
}
