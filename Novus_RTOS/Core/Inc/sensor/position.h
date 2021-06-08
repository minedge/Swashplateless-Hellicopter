/*
 * position.h
 *
 *  Created on: Jun 5, 2021
 *      Author: mined
 */

#ifndef INC_SENSOR_POSITION_H_
#define INC_SENSOR_POSITION_H_


#include "sensor/as5147.h"
#include "rc/spektrum.h"

/**
  * @defgroup Controller_Setup
  * @brief Minimum & Maximum rang of RPM
  */
#define RPM_MIN 250
#define RPM_MAX 7000

/**
  * @}
  */


/*
* @brief motor information struct
*/

typedef struct motor_inforamtion{
	float offset;

	float pre_ang;
	float ang;				/*< 모터 각도		*/

	float pre_rpm;
	float rpm;					/*< 모터 속도		*/

	float pre_lpf;
	float lpf;

	float pre_pwm;
	float pwm;				/*< Output			*/

	float acceleration;		/*< 모터 가속도		*/
	float time;				/*< cycle_time		*/
}MOTOR;

/**
  * @}
  */

/**
  * @}
  */

/*
* @brief Global variable motor have motor information
*/

volatile uint8_t offset_flag;
volatile uint32_t sens_time;
volatile uint32_t sens_start;

/**
  * @}
  */

/**
  * @}
  */

/*
* @brief Position update fuction
*/

void updatePosition(MOTOR* motor, RC rc);
void updateRPM(MOTOR* motor);
void setOffset(MOTOR* motor);

#endif /* INC_SENSOR_POSITION_H_ */
