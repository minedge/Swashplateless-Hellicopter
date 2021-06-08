/*
 * tx_pc.c
 *
 *  Created on: 2021. 4. 21.
 *      Author: sb030
 */

#include "tmtc/tmtc.h"

UART_HandleTypeDef huart2;

void get_packet_moter(MOTOR motor, SPT_Value setpoint_value){
	payload.A = *(uint32_t*)(&motor.ang);
	payload.B = *(uint32_t*)(&motor.rpm);
	payload.C = *(uint32_t*)(&setpoint_value.speed);
	payload.D = *(uint32_t*)(&motor.pwm);
	payload.E = 0x00;
    payload.F = 0x00;
    payload.G = 0x00;
    trans_pc(5,0);
}

void get_packet_rc(RC rc){
	payload.A = rc.throttle;
	payload.B = rc.roll;
	payload.C = rc.pitch;
	payload.D = rc.yaw;
	payload.E = rc.aux1;
	payload.F = rc.aux2;
	payload.G = rc.aux3;

    trans_pc(7,1);
}

/** @J.Yeon
  * @brief  패킷 정의 후 UART 송신
  * @param  uint8_t 보내는 패킷구조체 중 실제 구조체 크기
  * @param  uint8_t 메시지ID
  * @retval None
  */
void trans_pc(uint8_t len, uint8_t msgid){
	tp.header = STX;
	tp.len = len;
	tp.msgid = msgid;
	tp.payload = payload;
	tp.end = ETX;

	uint8_t buffer[32] = {
			tp.header,
			tp.len,
			tp.msgid,
			(uint8_t)((tp.payload.A & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.A & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.A & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.A & 0X000000FF)),
			(uint8_t)((tp.payload.B & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.B & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.B & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.B & 0X000000FF)),
			(uint8_t)((tp.payload.C & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.C & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.C & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.C & 0X000000FF)),
			(uint8_t)((tp.payload.D & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.D & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.D & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.D & 0X000000FF)),
			(uint8_t)((tp.payload.E & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.E & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.E & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.E & 0X000000FF)),
			(uint8_t)((tp.payload.F & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.F & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.F & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.F & 0X000000FF)),
			(uint8_t)((tp.payload.G & 0XFF000000) >> 24),
			(uint8_t)((tp.payload.G & 0X00FF0000) >> 16),
			(uint8_t)((tp.payload.G & 0X0000FF00) >> 8),
			(uint8_t)((tp.payload.G & 0X000000FF)),
			tp.end

	};

	HAL_UART_Transmit(&huart2, buffer, 32, 1000); //읽어드린 값 터미널로 출력
}
