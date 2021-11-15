#include <rc/spektrum.h>

/** @J.Yeon
  * @brief  uart로 받은 값을 각 채널별 data 저장
  * @param  struct RC
  * @retval 각 채널별 data(RC)
  */
void Spektrum_Read(){
	//uint8_t fade = rc_byte_data[0];
	//uint8_t sys = rc_byte_data[1];

    // 비트연산을 통한 각채널별 data 추출
    for (int index = 1; index <= 7; index++) {
    	uint16_t bit_data = (rc_byte_data[index * 2] << 8) | rc_byte_data[(index * 2) + 1];
        uint8_t id = (bit_data & 0x7800) >> 11;
        uint16_t pos = bit_data & 0x07FF;
        
        if(pos >= RC_MIN && pos <= RC_MAX){
            rc.channel[id].id = id;
            rc.channel[id].pos = pos;
        }
    }

}
