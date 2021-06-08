#include <rc/spektrum.h>


/** @J.Yeon
  * @brief  uart로 받은 값을 각 채널별 data 저장
  * @param  struct RC
  * @retval 각 채널별 data(RC)
  */
void spectrum_read(RC* rc){
    fade = rc_byte_data[0];
    sys = rc_byte_data[1];

    // 비트연산을 통한 각채널별 data 추출
    for (int index = 1; index <= 7; index++) {
        bit_data = (rc_byte_data[index * 2] << 8) | rc_byte_data[(index * 2) + 1];
        uint8_t id = (bit_data & 0x7800) >> 11;
        uint16_t pos = bit_data & 0x07FF;
        
        if(pos >= RC_MIN && pos <= RC_MAX){
            channel[id].id = id;
            channel[id].pos = pos;
        }
    }

    // 각 채널별 입력값 입력
    rc->throttle = channel[0].pos;
	rc-> roll = channel[1].pos;
	rc-> pitch = channel[2].pos;
	rc-> yaw = channel[3].pos;
	rc-> aux1 = channel[4].pos;
	rc-> aux2 = channel[5].pos;
	rc-> aux3 = channel[6].pos;
}
