/*
 * tx_pc.c
 *
 *  Created on: 2021. 4. 21.
 *      Author: sb030
 */

#include "tmtc/tmtc.h"

void tmtcInit(UART_HandleTypeDef* huartx){
	huart = huartx;
    uplink_packet_pointer = (uint8_t*)&uplink_packet;
    downlink_packet_pointer = (uint8_t*)&downlink_packet;
}

uint8_t checkPacket(){
	uint8_t crc_check = 0;

	for(int i = 0; i < sizeof(PACKET)-1; i++){
			crc_check += uplink_packet_pointer[i];
	}
	crc_check ^= 0xFF;

	if(crc_check == uplink_packet.crc)
		return 0;
	else
		return -1;
}

void parseMessage(){
	int spc = -1;
	int* m_spc = 0;
	float mp = 0, mi = 0, md = 0;

	switch(uplink_packet.msg_id){
	case SET_AMPLITUDE_GAIN:
		setAmplitudeGain(*(float*)(&uplink_packet.payload.data[0]));
		break;
	case SET_SHIFT_GAIN:
		setShiftGain(*(float*)(&uplink_packet.payload.data[0]));
		break;
	case SET_PID_GAIN:
		m_spc = (int*)&uplink_packet.payload.data[0];
		for(int i = 0; i < 6; i++){
			if(*m_spc & (0x0F000000 >> (i*8))){
				spc = i;
				break;
			}
		}
		mp = *(float*)(&uplink_packet.payload.data[4]);
		mi = *(float*)(&uplink_packet.payload.data[8]);
		md = *(float*)(&uplink_packet.payload.data[12]);
		setPIDGain(mp, mi, md, spc);
		break;
	case SET_MODE:
		break;
	}
}

void makePacket(uint8_t msg_id, PAYLOAD payload){
	downlink_packet.header = STX;
	downlink_packet.msg_id = msg_id;
	downlink_packet.payload = payload;

	for(int i = 0; i < sizeof(PACKET); i++){
			downlink_packet.crc += downlink_packet_pointer[i];
	}

	downlink_packet.crc ^= 0xFF;
}

void sendHeartBeat(){
	PAYLOAD heart_payload;

	float* P = (float*)(&heart_payload.data[0]);
	float* I = (float*)(&heart_payload.data[4]);
	float* D = (float*)(&heart_payload.data[8]);
	float* T = (float*)(&heart_payload.data[12]);

	*P = 0.024;
	*I = 0.0001;
	*D = 0.001;
	*T = 0.0;

	makePacket(HEART_BEAT, heart_payload);

	HAL_UART_Transmit(huart, downlink_packet_pointer, sizeof(PACKET), HAL_TIMEOUT);
}

void sendACK(){
	PAYLOAD ack_payload;

	for(int i = 0; i < sizeof(PAYLOAD); i++){
		ack_payload.data[i] = i%2;
	}

	makePacket(TMTC_ACK, ack_payload);

	HAL_UART_Transmit(huart, downlink_packet_pointer, sizeof(PACKET), HAL_TIMEOUT);
}
