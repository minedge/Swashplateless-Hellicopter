#pragma once
#ifndef __NOVUS_MATH_H
#define __NOVUS_MATH_H

#include <stdint.h>
#include <math.h>

#define PI 3.14159265

#define LPF_GAIN 0.1

float map(float target, int from_min, int from_max, int to_min, int to_max);
float getLPF_RPM(float raw_rpm, float pre_rpm, float loop_time);
float LPF(float raw_rpm, float loop_time, float pre_rpm, float pre_lpf);

#endif
