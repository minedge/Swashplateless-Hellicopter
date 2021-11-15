/*
 * tx_pc.h
 *
 *  Created on: 2021. 4. 21.
 *      Author: sb030
 *
 * 참고 사이트
 * https://dkeemin.com/stm32f0xx-tim%ED%83%80%EC%9D%B4%EB%A8%B8-%EC%9D%B8%ED%84%B0%EB%9F%BD%ED%8A%B8-%EC%BD%94%EB%93%9C-%EC%9E%91%EC%84%B1%ED%95%98%EA%B8%B0/
 * https://www.python2.net/questions-629179.htm
 */

#ifndef INC_TX_PC_H_
#define INC_TX_PC_H_

#include "main.h"

#define STX 0xFF

#define TMTC_ACK 255
#define HEART_BEAT 254

#define SET_AMPLITUDE_GAIN 0x65
#define SET_SHIFT_GAIN 0x66
#define SET_PID_GAIN 0x80

#define SET_MODE 0xF1
#define MODE_ARM
#define MODE_DISARM
#define MODE_MANUAL
#define MODE_STABILIZE


typedef struct _payload_data_struct{
	uint8_t data[16];
}PAYLOAD;

typedef struct _telemetry_packet_struct{
	uint8_t header;
	uint8_t msg_id;
	PAYLOAD payload;
	uint8_t crc;
}PACKET;

uint8_t* uplink_packet_pointer;
PACKET uplink_packet;
uint8_t* downlink_packet_pointer;
PACKET downlink_packet;

UART_HandleTypeDef* huart;

void tmtcInit(UART_HandleTypeDef* huartx);

uint8_t checkPacket();
void parseMessage();

void makePacket(uint8_t msg_id, PAYLOAD payload);

void sendHeartBeat();
void sendACK();

void trans_pc(uint8_t len, uint8_t msgid);

#endif /* INC_TX_PC_H_ */
