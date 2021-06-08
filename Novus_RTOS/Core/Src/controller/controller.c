#include "controller/controller.h"

TIM_HandleTypeDef* time_handler;

void controllerInit(TIM_HandleTypeDef* htimex){
	time_handler = htimex;
	time_handler->Instance->CCR1 = 1000;
#ifdef I_CONTROLLER
	integral_error = 0;
	setSpeedGain(0.132, 0.01147, 0.00015);
#else
	setSpeedGain(0.0003, 0);
	setMomentGain(0.0002, 0);
#endif
	HAL_TIM_PWM_Start(time_handler, TIM_CHANNEL_1);
}

float PD_Controller(float p, float d, float error){
    float control_value = 0;

    float proportion_controll_value = error * p;

    float differential_controll_value = ((error - pre_error) / (loop_time/1000.)) * d;

    control_value = proportion_controll_value + differential_controll_value;

    return control_value;
}

float PID_Controller(float p, float d, float i, float error){
    float control_value = 0;

    float proportion_controll_value = error * p;

    integral_error += error;
    float integral_controll_value = integral_error * i;

    float differential_controll_value = ((error - pre_error) / (loop_time/1000.)) * d;

    control_value = proportion_controll_value + differential_controll_value + integral_controll_value;

    return control_value;
}

MODE getMode(RC rc, MOTOR* motor){
	if(rc.aux3 > 500){
		if(rc.aux1 < 500){
			return ARM;
		}else if(rc.aux1 > 500 && rc.aux1 < 1400){
			return NON_MOMENT;
		}else{
			setSpeedGain(moment_gain.P_gain, moment_gain.D_gain);
			return MOMENT;
		}
	}else{
		if(rc.aux2 > 1000){
			setOffset(motor);
		}
		return CUT_OFF;
	}
}

float speedController(float setpoint, MOTOR motor){
    float error = setpoint - motor.lpf;

#ifdef I_CONTROLLER
    float control_value = PID_Controller(speed_gain.P_gain, speed_gain.D_gain, speed_gain.I_gain, error);
#else
    float control_value = PD_Controller(speed_gain.P_gain, speed_gain.D_gain, error);
#endif

    pre_error = error;

    return control_value;
}

float momentController(SPT_Value setpoint, MOTOR motor){
	float control_value = map(rc.throttle, RC_MIN, RC_MAX, PWM_MIN, PWM_MAX) + (sin((motor.ang*(2*PI/360.)) + setpoint.cyclic_shift) * setpoint.amplitude);

	return control_value;
}

volatile float command = 0;

void outputMotor(float control_command, SPT_Value setpoint, MODE mode, MOTOR* motor){
    //! NOTE :: rpm command to throttle percent scalar
    switch(mode){
        case CUT_OFF:
#ifdef I_CONTROLLER
        	integral_error = 0;
#endif
        	pre_error = 0;
            command = 1000;
            break;
        case ARM:
#ifdef I_CONTROLLER
        	integral_error = 0;
#endif
        	pre_error = 0;
            command = 1200;
            break;
#if 0
        case NON_MOMENT:
            command = motor->pwm + (control_command / 50.);
            if(command < PWM_MIN) command = PWM_MIN;
            else if(command > PWM_MAX) command = PWM_MAX;
        	break;
#endif
        case MOMENT:
			command = control_command;
    	    if(command < PWM_MIN) command = PWM_MIN;
    	    else if(command > PWM_MAX) command = PWM_MAX;
        	break;
        default:
        	command = 1000;
        	pre_error = 0;
        	break;
    }

    PWM_Generator(command, motor);
}

void PWM_Generator(float command, MOTOR* motor){
    motor->pwm = command;
    time_handler->Instance->CCR1 = command;
}
