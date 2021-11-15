/*
 * controller.h
 *
 *  Created on: 2021. 10. 19.
 *      Author: mined
 */

#ifndef INC_CONTROLLER_ATTITUDECONTROLLER_H_
#define INC_CONTROLLER_ATTITUDECONTROLLER_H_

#include <math.h>
#include "RC/spektrum.h"
#include "Controller/CyclicPitchController.h"
#include "Sensor/WT931.h"

PQR_CMD CommandGennerate(IMU imu_data, RC rc_cmd);

#endif /* INC_CONTROLLER_ATTITUDECONTROLLER_H_ */
