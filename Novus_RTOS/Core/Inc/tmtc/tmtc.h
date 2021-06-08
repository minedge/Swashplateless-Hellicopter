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

#include"rc/spektrum.h"
#include"sensor/position.h"
#include"controller/setup.h"
#include"stm32g4xx_hal_uart.h"

#ifndef INC_TX_PC_H_
#define INC_TX_PC_H_

#define STX 0xFF
#define ETX 0xEE

struct PAYLOAD{
	uint32_t A;
	uint32_t B;
	uint32_t C;
	uint32_t D;
	uint32_t E;
	uint32_t F;
	uint32_t G;
}payload;

struct TrnasPacket{
	uint8_t header;
	uint8_t len;
	uint8_t msgid;
	struct PAYLOAD payload;
	uint8_t end;
}tp;

uint8_t transpc_index;

void get_packet_moter(MOTOR motor, SPT_Value setpoint_value);
void get_packet_rc(RC rc);
void trans_pc(uint8_t len, uint8_t msgid);

#endif /* INC_TX_PC_H_ */
