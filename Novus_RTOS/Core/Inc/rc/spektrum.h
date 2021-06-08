#pragma once
 /*
 * spectrum.h
 *
 *  Created on: 2021. 4. 7.
 *      Author: sb030
 */
#include <stdint.h>

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

typedef struct Spectrum_Radio_Controller{

	uint16_t throttle;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;
}RC;

struct CHANNEL_DATA {
    uint8_t phase;
    uint8_t id;
    uint16_t pos;
};

uint8_t tmtc_data[8];

uint8_t rc_byte_data[16];
uint16_t bit_data;
uint8_t fade, sys;
struct CHANNEL_DATA channel[12];
RC rc;

void spectrum_read(RC* rc);

#endif /* INC_SPECTRUM_H_ */
