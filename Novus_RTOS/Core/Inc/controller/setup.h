#pragma once
/**
  ******************************************************************************
  * @file           : setup.h
  * @brief          : Header for setup.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Setting & Management
  *  Controller's GAIN
  *		SPD_GAIN spd_gain - Speed Controller  (additionally I_gain)
  *		MNT_GAIN mnt_gain - Moment Controller (additionally I_gain)
  *		amp_gain - Decide the amount amplitude of sin wave multiplied according to stick command
  *	 Set Point
  *		Target speed (setpoint of speed)
  *		Target roll_amplitude, pitch_amplitude (amplitude of sin wave)
  *		Target cyclic_shift (Cyclic Shift of sin wave)
  *
  ******************************************************************************
  */

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SETUP_H
#define __SETUP_H

/* Includes ------------------------------------------------------------------*/
#include "sensor/position.h"
#include "rc/spektrum.h"

/** @addtogroup NOVUS_Controller
  * @{
  */

//!NOTE :: I_gain은 현재 사용하지 않음. 사용시 주석 해제. @mhlee (2021.03.30)
/**
  * @defgroup Controller_Setup
  * @brief Convert PD Controller to PID Controller
  */
//#define I_CONTROLLER    /*!< Using PID Controller or PD Controller */

/**
  * @}
  */


/**
  * @defgroup Controller_Setup
  * @brief no operate margin when RC controller stick in center position
  */
#define RC_MARGIN_RANGE 3   /*!< Recommend range 1~5        */


/**
  * @}
  */

/**
  * @brief speed controller에 사용될 gain
  */
typedef struct Speed_controller_gain{
    float P_gain;       /*!< Proportional Controller Gain   */

    float D_gain;       /*!< Differential  Controller Gain  */

    #ifdef I_CONTROLLER
        float I_gain;   /*!< Integral  Controller Gain      */
    #endif
}SPD_GAIN;

/**
  * @}
  */


/**
  * @brief moment controller에 사용될 gain
  */
typedef struct Moment_controller_gain{
    float P_gain;       /*!< Proportional Controller Gain   */

    float D_gain;       /*!< Differential  Controller Gain  */

    #ifdef I_CONTROLLER
        float I_gain;   /*!< Integral  Controller Gain      */
    #endif
}MNT_GAIN;

/**
  * @}
  */


/**
  * @brief Set Point Value
  */
typedef struct {
    float speed;        /*!< Motor Target Speed(RPM) Set Point */
    float moment_speed;

    float amplitude;    /*!< Target Amplitude of Sin Wave 
                            controll target = speed + amplitude*/

    float cyclic_shift; /*!< Target Cyclic Shift of Sin Wave
                            controll target = speed + amplitude * sin(Motor.pos + cyclic_shift) */
}SPT_Value;

/**
  * @}
  */


/**
  * @brief Global Variable to use
  */
float amplitude_gain;                             /*!< Amplitude gain will multiply with RC controller's Scalar */
SPD_GAIN speed_gain;
MNT_GAIN moment_gain;

/**
  * @}
  */

/* setting operation functions *****************************************************/
#ifdef I_CONTROLLER
void setSpeedGain(float p, float d, float i);
void setMomentGain(float p, float d, float i);
#else
void setSpeedGain(float p, float d);
void setMomentGain(float p, float d);
#endif
void setAmplitudeGain(float gain, float rpm);

/**
  * @}
  */

/**
  * @}
  */

/* Unit Conversion operation functions *****************************************************/
float getStickPercent(uint16_t stick_pos);
float getStickVector(uint16_t stick_pos);
float getStickScalar(float stick_vector);

/**
  * @}
  */

/* check operation functions *****************************************************/
float checkMargin(float stick_vector);

/**
  * @}
  */

/**
  * @}
  */

/* setpoint operation functions *****************************************************/
SPT_Value setpoint(RC rc, MOTOR motor);
/* setpoint sub functions *****************************************************/
float setSpeed(uint16_t throttle);
float setAmplitude(uint16_t roll_stick_pos, uint16_t pitch_stick_pos);
float setCyclicShift(uint16_t roll_stick_pos, uint16_t pitch_stick_pos);

/**
  * @}
  */
  
/**
  * @}
  */

#endif
