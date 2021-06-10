 /**
  ******************************************************************************
  * @file    setup.c
  * @author  NOVUS Graduation Project Team
  * @brief   Motor Speed Controller Setup module
  *          This file provides setting functions to manage the following
  *          functionalities of the setting Controller gain and Target Point:
  *           + Gain setting and getting unit converted value function
  *           + The function Decied target point using RC command 
  *
  ******************************************************************************
  */
  
/**
  * @}
  */

/* Includes ------------------------------------------------------------------*/
#include "controller/setup.h"

/**
  * @}
  */

/** @addtogroup NOVUS_Controller
  * @{
  */
  
/**
  * @}
  */

/**
  * @brief  Setting Gain for Speed Controller & Moment Controller
  *         If using PD controller 2 parameter is needed p / d
  * @param  p Proportional Controller Gain, Recommaned amount of Change 0.1
  * @param  d Differential  Controller Gain, Recommaned amount of Change 0.01
  * @retval None
  */
#ifdef I_CONTROLLER
void setSpeedGain(float p, float d, float i){
	speed_gain.P_gain = p;
	speed_gain.D_gain = d;
	speed_gain.I_gain = i;
}

void setMomentGain(float p, float d, float i){
	moment_gain.P_gain = p;
	moment_gain.D_gain = d;
	moment_gain.I_gain = i;
}
#else
/**
  * @brief  Setting Gain for Speed Controller & Moment Controller
  *         If using PID controller 3 parameter is needed p / d / i
  * @param  p Proportional Controller Gain, Recommaned amount of Change 0.1
  * @param  d Differential  Controller Gain, Recommaned amount of Change 0.01
  * @param  i Integral  Controller Gain, Recommaned amount of Change 0.001
  * @retval None
  */
void setSpeedGain(float p, float d){
	speed_gain.P_gain = p;
	speed_gain.D_gain = d;
}

void setMomentGain(float p, float d){
	moment_gain.P_gain = p;
	moment_gain.D_gain = d;
}
#endif

/**
  * @brief  Setting amount of multiplied sin wave's Amplitude
  * @param  gain Setting amount of multiply with RC controller's Scalar
  *             Recommaned amount of Change 0.1
  * @retval None
  */
void setAmplitudeGain(float gain){
	amplitude_gain = gain;//*map(rpm, RPM_MIN, RPM_MAX, 0, 1);
}


/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @brief  Unit Conversion RC command - PWM data to Percentage 
  * @param  stick_pos RC Stick position pwm value RC_MIN to RC_MAX
  * @retval percent of stick position
  */
float getStickPercent(uint16_t stick_pos){
    float percent = map(stick_pos, RC_MIN, RC_MAX, 0, 100);

    return percent;
}

/**
  * @brief  Unit Conversion RC command - PWM data to Percentage Vector
  *         '+'Vector = UP/RIGHT
  *         '-'Vector = DOWN/LEFT
  * @param  stick_pos RC Stick position pwm value RC_MIN to RC_MAX
  * @retval percent vector of stick position
  */
float getStickVector(uint16_t stick_pos){
    float vector = map(stick_pos, RC_MIN, RC_MAX, -100, 100);

    return vector;
}

/**
  * @brief  Unit Conversion RC command - Percentage Vector to Percentage Scalar
  * @param  stick_vector percent vector of RC stick position
  * @retval percent scalar of stick position
  */
float getStickScalar(float stick_vector){
    float stick_scalar = 0;

    if(stick_vector < 0){
        stick_scalar = stick_vector * (-1);
    }else{
        stick_scalar = stick_vector;
    }

    return stick_scalar;
}

/**
  * @brief  Check Margin of RC Command 
  *         When Roll/Pitch Stick is in center position it will ignore a tiny movement
  * @param  stick_vector percent vector of RC stick position
  * @retval percent vector of stick position
  */
float checkMargin(float stick_vector){
    if(stick_vector > (RC_MARGIN_RANGE * (-1)) && stick_vector < RC_MARGIN_RANGE){
        stick_vector = 0;
    }
    return stick_vector;
}

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @brief  SetPoint Function decied target speed, target amplitude, target cyclic shift
  *         based on RC command.
  * @param  rc have 7ch information (throttle, roll, pitch, yaw, aux1, aux2, aux3)
  *         this is PWM based data
  * @retval SPT_Value setpoint - updated setpoint value
  */
SPT_Value setpoint(RC rc, MOTOR motor){
    SPT_Value setpoint;

    setpoint.speed = setSpeed(rc.throttle);
	setAmplitudeGain(1);
    setpoint.amplitude = map(setAmplitude(rc.roll, rc.pitch), 0, 100, 0, map(motor.rpm, RPM_MIN, RPM_MAX, 10, 500));
    setpoint.cyclic_shift = setCyclicShift(rc.roll, rc.pitch);
	//setpoint.moment_speed = setpoint.speed + (sin((motor.ang*(2*PI/360.)) + setpoint.cyclic_shift) * setpoint.amplitude);

    return setpoint;
}

/**
  * @}
  */

/**
  * @brief  Calculate target speed based throttle information
  * @param  throttle RC throttle channel command based on PWM
  * @retval updated target speed setpoint value
  */
float setSpeed(uint16_t throttle){
    float throttle_percent = getStickPercent(throttle);

    //!NOTE :: Percent to RPM @mhlee
    float speed = map(throttle_percent, 0, 100, RPM_MIN, RPM_MAX);
    
    return speed;
}

/**
  * @brief  Calculate target amplitude of sin wave using roll/pitch RC information
  * @param  roll_stick_pos RC roll channel command based on PWM
  * @param  pitch_stick_pos RC pitch channel command based on PWM
  * @retval updated target amplitude setpoint value
  */
float setAmplitude(uint16_t roll_stick_pos, uint16_t pitch_stick_pos){
    float roll_scalar = getStickScalar(getStickVector(roll_stick_pos));
    float pitch_scalar = getStickScalar(getStickVector(pitch_stick_pos));

    float pitch_ratio = pitch_scalar / (roll_scalar + pitch_scalar);
    float roll_ratio = 1 - pitch_ratio;

    float cmd_scalar = (pitch_scalar * pitch_ratio) + (roll_scalar * roll_ratio);

    float amplitude = (cmd_scalar * amplitude_gain);

    return amplitude;
}

/**
  * @brief  Calculate target cyclic shift of sin wave using roll/pitch RC information
  * @param  roll_stick_pos RC roll channel command based on PWM
  * @param  pitch_stick_pos RC pitch channel command based on PWM
  * @retval updated target cyclic shift setpoint value
  */
float setCyclicShift(uint16_t roll_stick_pos, uint16_t pitch_stick_pos){
    float roll_vector = getStickVector(roll_stick_pos);
    float pitch_vector = getStickVector(pitch_stick_pos);

    roll_vector = checkMargin(roll_vector);
    pitch_vector = checkMargin(pitch_vector);

    float roll_scalar = getStickScalar(roll_vector);
    float pitch_scalar = getStickScalar(pitch_vector);

    float shift = 0;

    if(roll_vector == 0 && pitch_vector == 0){
        shift = 0;                      /*!< There is any command Roll and Pitch  */
    }else if(roll_vector == 0 && pitch_vector != 0){
        if(pitch_vector < 0){
            shift = PI;                 /*!< There is only negative Pitch command  */
        }else{
            shift = 0;                  /*!< There is only positive Pitch command  */
        }
    }else if(roll_vector != 0 && pitch_vector == 0){
        if(roll_vector < 0){
            shift = (3 * PI) / 2.0;     /*!< There is only negative Roll command  */
        }else{
            shift = PI / 2.0;           /*!< There is only positive Roll command  */
        }
    }else{
        float shift_ratio = (PI/2) * (roll_scalar / (roll_scalar+pitch_scalar));

        if(pitch_vector < 0){
            if(roll_vector < 0){
                shift = PI + shift_ratio;       /*!< There is negative Pitch and negative Roll command  */
            }else{
                shift = PI - shift_ratio;     /*!< There is negative Pitch and positive Roll command  */
            }
        }else{
            if(roll_vector < 0){
                shift = (2*PI) - shift_ratio;       /*!< There is positive Pitch and negative Roll command  */
            }else{
                shift = shift_ratio;            /*!< There is positive Pitch and positive Roll command  */
            }
        }
    }

    return shift;
}
