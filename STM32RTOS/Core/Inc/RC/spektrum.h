#pragma once
 /*
 * spectrum.h
 *
 *  Created on: 2021. 4. 7.
 *      Author: sb030
 */
#include "Serial/uart.h"

#ifndef INC_SPECTRUM_H_
#define INC_SPECTRUM_H_

#define THROTTLE    0
#define AILERON     1
#define ELEVATOR    2
#define RUDDER      3
#define AUX1        4
#define AUX2        5
#define AUX3        6
#define AUX4        7
#define AUX5        8
#define AUX6        9
#define AUX7        10
#define AUX8        11

#define RC_MIN 342
#define RC_MAX 1706

#define SWITCH_POS(aux)			(aux < 1000 ? 0 : (aux < 1500 ? 1 : 2))

struct CHANNEL_DATA {
    uint8_t phase;
    uint8_t id;
    uint16_t pos;
};

typedef struct Spectrum_Radio_Controller{
	float thro;

	float pi;
	float theta;
	float psi;

	float d_pi;
	float d_theta;
	float d_psi;

	uint8_t mode;
	uint8_t arm;

	struct CHANNEL_DATA channel[12];
}RC;


RC rc;
uint8_t rc_byte_data[16];

void Spektrum_Read();

#endif /* INC_SPECTRUM_H_ */
