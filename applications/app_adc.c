/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TC_DIFF_MAX_PASS				60  // TODO: move to app_conf
#define POLES                           72  //how many poles our motor has
#define SPEED                           10.0 //average speed in meters / second (used for avg driving mode)

#define CTRL_USES_BUTTON(ctrl_type)(\
		ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON || \
		ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC || \
		ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER || \
		ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON || \
		ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON || \
		ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON)

// Threads
static THD_FUNCTION(adc_thread, arg);
static THD_WORKING_AREA(adc_thread_wa, 512);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile float adc1_override = 0.0;
static volatile float adc2_override = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool adc_detached = false;
static volatile bool buttons_detached = false;
static volatile bool rev_override = false;
static volatile bool cc_override = false;
//variables for calibration
//static volatile int next = 10; //keeps track of next state //set for inital phase
//static volatile int state = 0; //keeps track of current state
//static volatile int phase = 1; //keeps track of calibration phase ***work on incorperating this***
//static volatile float time_at_speed = 0; //keeps track of the time we are at a given speed
static volatile float pwr_array[90] = {0.0}; //records powers in phase 2 to be read in phase 3
//static volatile bool collect = false; //flag for PI, states if data should be read or not
//static volatile float current_speed = 0.0;  // what it says on the tin
//static volatile float avg_pwr_at_speed = 0.0; // what it says on the tin

void app_adc_configure(adc_config *conf) {
	if (!buttons_detached && (((conf->buttons >> 0) & 1) || CTRL_USES_BUTTON(conf->ctrl_type))) {
		if (use_rx_tx_as_buttons) {
			palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
			palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
		} else {
			palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
		}
	}

	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_start(bool use_rx_tx) {
#ifdef HW_ADC_EXT_GPIO
	palSetPadMode(HW_ADC_EXT_GPIO, HW_ADC_EXT_PIN, PAL_MODE_INPUT_ANALOG);
#endif
#ifdef HW_ADC_EXT2_GPIO
	palSetPadMode(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN, PAL_MODE_INPUT_ANALOG);
#endif

	if (buttons_detached) {
		use_rx_tx_as_buttons = false;
	} else {
		use_rx_tx_as_buttons = use_rx_tx;
	}

	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_adc_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_get_voltage(void) {
	return read_voltage;
}

float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

float app_adc_get_voltage2(void) {
	return read_voltage2;
}

void app_adc_detach_adc(bool detach){
	adc_detached = detach;
}

void app_adc_adc1_override(float val){
	val = utils_map(val, 0.0, 1.0, 0.0, 3.3);
	utils_truncate_number(&val, 0, 3.3);
	adc1_override = val;
}

void app_adc_adc2_override(float val){
	val = utils_map(val, 0.0, 1.0, 0.0, 3.3);
	utils_truncate_number(&val, 0, 3.3);
	adc2_override = val;
}

void app_adc_detach_buttons(bool state){
	buttons_detached = state;
}

void app_adc_rev_override(bool state){
	rev_override = state;
}

void app_adc_cc_override(bool state){
	cc_override = state;
}

#define WHEEL_DIAMETER 19.5

float get_car_speed(){
	//returns the current speed of the car in m/s 
	float revs = mc_interface_get_rpm();
	revs = revs / POLES; 
	float distance = revs * 3.1415926 * WHEEL_DIAMETER;
	distance = distance / 39.37; //gets distance per minute in meters
	return distance / 60; //div by 60 to get m/s

}


#define MAX_RPM 72 * 350
#define MAX_AMPS 20

float get_power_value(float watts){
	//returns a float between 0 and 1 to drive the motor at the desired power
	// float max_amps = motor_now()->m_conf.lo_current_motor_max_now;
	float rpm = mc_interface_get_rpm();
	if (rpm < 0){
		rpm = -(rpm);
	}
	float voltage = (rpm / MAX_RPM) * 48;
	float amps = watts / voltage;
	return amps / MAX_AMPS;

}



// void update_speed_time(){
// 	//will tick up the amount of time the car remains at a given speed, resets to 0 once a speed 
// 	//change is detected
// 	//NOTE: MAY NEED A BIT OF LEEWAY FOR CURRENT SPEED EQUALITY 
// 	if (current_speed - 0.05 <= get_car_speed() <= current_speed + 0.05){
// 		time_at_speed += 0.1;
// 	}
// 	else{
// 		current_speed = get_car_speed();
// 		time_at_speed = 0.0;
// 	}
// }

// void update_speed_pwr(pwr){
// 	//averages the power usage while speed is constant, the variable speed_pwr can then be used to 
// 	//update the appropriate variables
// 	//AGAIN MAY NEED SOME LEEWAY IN EQUALITY CUZ FLOATS
// 	if (current_speed - 0.05 <= get_car_speed() <= current_speed + 0.05){
// 		//update power with rolling average
// 		avg_pwr_at_speed = ((avg_pwr_at_speed * 1000 * time_at_speed) + pwr) / (time_at_speed * 1000);
// 	}
// 	else{
// 		//if 1st time at speed start the average off at the current power
// 		avg_pwr_at_speed = pwr;
// 	}
// }

float burn_to_speed(float speed, float pwr){
	//Given a speed in m/s, seeks to get to that speed and maintain that power if button is pressed
	//use for average driving mode button, or turbo
	if (get_car_speed() < speed && pwr >= 0.5){
		return 1;
	}
	return 0;
}

float burn_at_power(float pwr){
	//lookup the speed, calculate the desired power for it and drive at that power
	//use for the standard drive mode
	if (pwr >= 0.5){
		return pwr_array[round(get_car_speed() * 10)];
	}
	return 0;
}

// float standby_no_data(float turbo){
// 	//function is called if we are in standby 
// 	if ( turbo >= 0.5 ){
// 		return 1.0;
// 	}

// 	else{
// 		return 0.0;
// 	}
	
// }

// void change_state(float turbo, float pwr, float time_at_speed){
// 	//given the current state, next state, if pwr is high, and how long we've been at a speed
// 	//we select the next state, need a system to discard data and go to appropriate place if turbo
// 	switch (state)
// 	{
// 	case 0:
// 		//if we are in standby see if there is pwr, if so check phase and return data accordingly
// 		collect = false; //whenever we get to standby, stop collecting data, whenever we leave begin
// 		if (pwr >= 0.5 && get_car_speed() == 0){
// 			collect = true;
// 			state = next;
// 		}
// 		break;

// 	case 10:
// 		//check to see if we're at max speed then go to coast state
// 		phase = 1;
// 		if (time_at_speed > 2.0){
// 			next = 20;
// 			state = 11;
// 		}
// 		break;
	
// 	case 11:
// 		//coast to 0 then go to standby for phase 2
// 		if (get_car_speed() <= 0.5){
// 			state = 0;
// 		}
// 		break;
	
// 	//any phase 2 case where next state is an increment of 5 
// 	case 20: case 25: case 30: case 35: case 40: case 45: case 50: case 55: 
// 		phase = 2;

// 		if (time_at_speed > 2.0){
// 				next = state + 5; 
// 				pwr_array[state] = avg_pwr_at_speed; //update the pwr array with the avg power 
// 				state = 0; //resets state to standby so we jump to the next state on the next loop
// 			}	
// 		break;


// 	//any phase 3 case where next state is an increment of 2
// 	case 120: case 125: case 130: case 135: case 140: case 145: case 150: case 155:
// 		phase = 3;

// 		if (time_at_speed > 2.0){
// 				next = state + 5; 
// 				state = 0;
// 			}	
// 		break;
	
// 	//any phase 2 case where next state is an increment of 2
// 	case 60: case 62: case 64: case 66: case 68: case 70: case 72: case 74: case 78: case 80:
// 	case 82: case 84: case 86: case 88: case 90:

// 		if (time_at_speed > 2.0){
// 				if (state == 90){
// 					next = 120;
// 				}
// 				else {
// 					next = state + 2; 
// 				}
// 				pwr_array[state] = avg_pwr_at_speed; //update the pwr array with the avg power 
// 				state = 0;
// 			}	
// 		break;

	
// 	//any phase 3 case where next state is an increment of 2
// 	case 160: case 162: case 164: case 166: case 168: case 170: case 172: case 174: case 178: 
// 	case 180: case 182: case 184: case 186: case 188: case 190:
	
// 		if (time_at_speed > 2.0){
// 				next = state + 2; 
// 				state =  0;
// 			}	
// 		break;
	
// 	default:
// 		//if we aren't in a state treat as though we are in standby
// 		next = 10;
// 		state = 0;
// 		phase = 1;
// 		break;

// 	}
// }

// float execute_calibration_step(float turbo, float pwr)
// {
// 	//given the state we are in, this function will return the appropriate motor power

// 	switch (state)
// 	{
// 	case 0:
// 		//standby allow the car to be positioned for the next state
// 		return standby_no_data(turbo);
// 		break;
	
// 	case 10:
// 		//max power burn, return 1.0
// 		return 1.0;
// 		break;
	
// 	case 11:
// 		//coast collection state
// 		return 0.0;
// 		break;
	
// 	//next group of states is phase 2 where we go to each speed and record the power in pwr array
// 	case 20: case 25: case 30: case 35: case 40: case 45: case 50: case 55: case 60: 
// 	case 62: case 64: case 66: case 68: case 70: case 72: case 74: case 78: case 80:
// 	case 82: case 84: case 86: case 88: case 90:

// 		return burn_to_speed(state, pwr);
// 		break;
	
// 	//next group of states is phase 3 where we go through each power in the power array
// 	case 120: case 125: case 130: case 135: case 140: case 145: case 150: case 155: case 160: 
// 	case 162: case 164: case 166: case 168: case 170: case 172: case 174: case 178: case 180:
// 	case 182: case 184: case 186: case 188: case 190:

// 		return burn_at_power(state - 100, pwr);
// 		break;

// 	default:
// 		//if we aren't in a state treat as though we are in standby
// 		return standby_no_data(turbo);
// 		break;
// 	} 
// }


static THD_FUNCTION(adc_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC");
	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT) {
			ms_without_power = 0;
		}

		// Read the external ADC pin voltage
		float pwr = ADC_VOLTS(ADC_IND_EXT);

		// Override pwr value, when used from LISP
		if(adc_detached){
			pwr = adc1_override;
		}

		read_voltage = pwr;

		// Optionally apply a filter
		static float filter_val = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(filter_val, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			pwr = filter_val;
		}

		/* // Map the read voltage
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Mapping with respect to center voltage
			if (pwr < config.voltage_center) {
				pwr = utils_map(pwr, config.voltage_start,
						config.voltage_center, 0.0, 0.5);
			} else {
				pwr = utils_map(pwr, config.voltage_center,
						config.voltage_end, 0.5, 1.0);
			}
			break;

		default:
			// Linear mapping between the start and end voltage
			pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
			break;
		} */

		// Linear mapping between the start and end voltage
		pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage_inverted) {
			pwr = 1.0 - pwr;
		}

		decoded_level = pwr;

		// Read the external ADC pin and convert the value to a voltage.
// #ifdef ADC_IND_EXT2
// 		float brake = ADC_VOLTS(ADC_IND_EXT2);
// #else
// 		float brake = 0.0;
// #endif

// #ifdef HW_HAS_BRAKE_OVERRIDE
// 		hw_brake_override(&brake);
// #endif

// 		// Override brake value, when used from LISP
// 		if(adc_detached == true){
// 			brake = adc2_override;
// 		}

// 		read_voltage2 = brake;

		float turbo = ADC_VOLTS(ADC_IND_EXT2);
		read_voltage2 = turbo;

		// Optionally apply a filter
		static float filter_val_2 = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(filter_val_2, turbo, FILTER_SAMPLES);

		if (config.use_filter) {
			turbo = filter_val_2;
		}

		// Map and truncate the read voltage
		turbo = utils_map(turbo, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
		utils_truncate_number(&turbo, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage2_inverted) {
			turbo = 1.0 - turbo;
		}

		decoded_level2 = turbo;

		// Read the button pins (Keep for setup)
		bool cc_button = false;
		bool rev_button = false;
		if (use_rx_tx_as_buttons) {
			cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
			if ((config.buttons >> 1) & 1) {
				cc_button = !cc_button;
			}
			rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
			if ((config.buttons >> 2) & 1) {
				rev_button = !rev_button;
			}
		} else {
			// When only one button input is available, use it differently depending on the control mode
			if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON ||
                    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER ||
					config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON) {
				rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
				if ((config.buttons >> 2) & 1) {
					rev_button = !rev_button;
				}
			} else {
				cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
				if ((config.buttons >> 1) & 1) {
					cc_button = !cc_button;
				}
			}
		}

		// Override button values, when used from LISP
		if (buttons_detached) {
			cc_button = cc_override;
			rev_button = rev_override;
			if ((config.buttons >> 1) & 1) {
				cc_button = !cc_button;
			}
			if ((config.buttons >> 2) & 1) {
				rev_button = !rev_button;
			}
		}

		if (!((config.buttons >> 0) & 1)) {
			cc_button = false;
		}

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		// switch (config.ctrl_type) {
		// case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		// case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		// case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		// case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		// case ADC_CTRL_TYPE_PID_REV_CENTER:
		// 	// Scale the voltage and set 0 at the center
		// 	pwr *= 2.0;
		// 	pwr -= 1.0;
		// 	break;

		// case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		// case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
		// 	pwr -= brake;
		// 	break;

		// case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
		// case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		// case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
		// case ADC_CTRL_TYPE_PID_REV_BUTTON:
		// 	// Invert the voltage if the button is pressed
		// 	if (rev_button) {
		// 		pwr = -pwr;
		// 	}
		// 	break;

		// default:
		// 	break;
		// }

		// Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);

		// Turbo vs Normal Button Selection
		//update the state and then find the appropriate power. That's it

		//Functional block for telling the car how to accelerate
		// update_speed_time();
		// change_state(turbo, pwr, time_at_speed);
		// pwr = execute_calibration_step(turbo, pwr);
		// update_speed_pwr(); 
		if (pwr >= 0.5){ //standard drive mode
			pwr = burn_at_power(pwr);
		}
		else if (turbo >= 0.5){ //turbo / avg drive mode
			pwr = burn_to_speed(SPEED, turbo);
		}
		else{ //no power if neither button pressed
			pwr = 0;
		}
		//keeps the average power at speed for storage in array
		// Apply throttle curve
		// pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}

		float current_rel = 0.0;
		bool current_mode = false;
		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();
		bool send_duty = false;

		current_mode = true;
		current_rel = pwr;

		// // Use the filtered and mapped voltage for control according to the configuration.
		// switch (config.ctrl_type) {
		// case ADC_CTRL_TYPE_CURRENT:
		// case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		// case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
		// 	current_mode = true;
		// 	current_rel = pwr;

		// 	if (fabsf(pwr) < 0.001) {
		// 		ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		// 	}
		// 	break;

        // case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		// case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		// case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		// case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		// case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
		// 	current_mode = true;
		// 	if (pwr >= 0.0) {
		// 		// if pedal assist (PAS) thread is running, use the highest current command
		// 		if (app_pas_is_running()) {
		// 			pwr = utils_max_abs(pwr, app_pas_get_current_target_rel());
		// 		}
		// 		current_rel = pwr;
		// 	} else {
		// 		current_rel = fabsf(pwr);
		// 		current_mode_brake = true;
		// 	}

		// 	if (pwr < 0.001) {
		// 		ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		// 	}

		// 	if ((config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC ||
		// 	    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER) && rev_button) {
		// 		current_rel = -current_rel;
		// 	}
		// 	break;

		// case ADC_CTRL_TYPE_DUTY:
		// case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		// case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
		// 	if (fabsf(pwr) < 0.001) {
		// 		ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		// 	}

		// 	if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
		// 		mc_interface_set_duty(utils_map(pwr, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
		// 		send_duty = true;
		// 	}
		// 	break;

		// case ADC_CTRL_TYPE_PID:
		// case ADC_CTRL_TYPE_PID_REV_CENTER:
		// case ADC_CTRL_TYPE_PID_REV_BUTTON:
		// 	if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
		// 		current_rel = pwr;
		// 	} else {
		// 		current_rel = pwr;
		// 	}

		// 	if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
		// 		float speed = 0.0;
		// 		if (pwr >= 0.0) {
		// 			speed = pwr * mcconf->l_max_erpm;
		// 		} else {
		// 			speed = pwr * fabsf(mcconf->l_min_erpm);
		// 		}

		// 		mc_interface_set_pid_speed(speed);
		// 		send_duty = true;
		// 	}

		if (fabsf(pwr) < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}
		// 	break;

		// default:
		// 	continue;
		// }

		bool range_ok = read_voltage >= config.voltage_min && read_voltage <= config.voltage_max;

		// If safe start is enabled and the output has not been zero for long enough
		if ((ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) || !range_ok) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, timeout_get_brake_current());
					}
				}
			}

			continue;
		}

		// Reset timeout
		timeout_reset();

		// If c is pressed and no throttle is used, maintain the current speed with PID control
		static bool was_pid = false;

		// Filter RPM to avoid glitches
		static float rpm_filtered = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(rpm_filtered, mc_interface_get_rpm(), RPM_FILTER_SAMPLES);

		if (current_mode && cc_button && fabsf(pwr) < 0.001) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				was_pid = true;
				pid_rpm = rpm_filtered;
			}

			mc_interface_set_pid_speed(pid_rpm);

			// // Send the same duty cycle to the other controllers
			// if (config.multi_esc) {
			// 	float current = mc_interface_get_tot_current_directional_filtered();

			// 	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
			// 		can_status_msg *msg = comm_can_get_status_msg_index(i);

			// 		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
			// 			comm_can_set_current(msg->id, current);
			// 		}
			// 	}
			// }

			continue;
		}

		was_pid = false;

		// // Find lowest RPM (for traction control)
		// float rpm_local = mc_interface_get_rpm();
		// float rpm_lowest = rpm_local;
		// if (config.multi_esc) {
		// 	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		// 		can_status_msg *msg = comm_can_get_status_msg_index(i);

		// 		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
		// 			float rpm_tmp = msg->rpm;

		// 			if (fabsf(rpm_tmp) < fabsf(rpm_lowest)) {
		// 				rpm_lowest = rpm_tmp;
		// 			}
		// 		}
		// 	}
		// }

		// // Optionally send the duty cycles to the other ESCs seen on the CAN-bus
		// if (send_duty && config.multi_esc) {
		// 	float duty = mc_interface_get_duty_cycle_now();

		// 	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		// 		can_status_msg *msg = comm_can_get_status_msg_index(i);

		// 		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
		// 			comm_can_set_duty(msg->id, duty);
		// 		}
		// 	}
		// }

		mc_interface_set_current_rel(current_rel);

		// if (current_mode) {
		// 	if (current_mode_brake) {
		// 		mc_interface_set_brake_current_rel(current_rel);

		// 		// Send brake command to all ESCs seen recently on the CAN bus
		// 		if (config.multi_esc) {
		// 			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		// 				can_status_msg *msg = comm_can_get_status_msg_index(i);

		// 				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
		// 					comm_can_set_current_brake_rel(msg->id, current_rel);
		// 				}
		// 			}
		// 		}
		// 	} else {
		// 		float current_out = current_rel;
		// 		bool is_reverse = false;
		// 		if (current_out < 0.0) {
		// 			is_reverse = true;
		// 			current_out = -current_out;
		// 			current_rel = -current_rel;
		// 			rpm_local = -rpm_local;
		// 			rpm_lowest = -rpm_lowest;
		// 		}

		// 		// Traction control
		// 		if (config.multi_esc) {
		// 			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		// 				can_status_msg *msg = comm_can_get_status_msg_index(i);

		// 				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
		// 					if (config.tc && config.tc_max_diff > 1.0) {
		// 						float rpm_tmp = msg->rpm;
		// 						if (is_reverse) {
		// 							rpm_tmp = -rpm_tmp;
		// 						}

		// 						float diff = rpm_tmp - rpm_lowest;
        //                         if (diff < TC_DIFF_MAX_PASS) diff = 0;
        //                         if (diff > config.tc_max_diff) diff = config.tc_max_diff;
		// 						current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
		// 					}

		// 					if (is_reverse) {
		// 						comm_can_set_current_rel(msg->id, -current_out);
		// 					} else {
		// 						comm_can_set_current_rel(msg->id, current_out);
		// 					}
		// 				}
		// 			}

		// 			if (config.tc) {
		// 				float diff = rpm_local - rpm_lowest;
        //                 if (diff < TC_DIFF_MAX_PASS) diff = 0;
        //                 if (diff > config.tc_max_diff) diff = config.tc_max_diff;
		// 				current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
		// 			}
		// 		}

		// 		if (is_reverse) {
		// 			mc_interface_set_current_rel(-current_out);
		// 		} else {
		// 			mc_interface_set_current_rel(current_out);
		// 		}
		// 	}
		// }
	}
}
