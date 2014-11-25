/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file central_data.c
 * 
 * \author MAV'RIC Team
 *   
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#include "central_data.h"
#include "conf_constants.h"
#include "delay.h"

static central_data_t central_data;

void central_data_init()
{	
	// Init servos
	//servo_pwm_init(central_data.servos);
	
	servos_conf_t servos_config =
	{
		.servos_count = 4,
		.types =
		{
			MOTOR_CONTROLLER,
			MOTOR_CONTROLLER,
			MOTOR_CONTROLLER,
			MOTOR_CONTROLLER
		},
	};
	servos_init( &central_data.servos, &servos_config, &central_data.mavlink_communication.mavlink_stream);
	servos_set_value_failsafe( &central_data.servos );
	pwm_servos_write_to_hardware( &central_data.servos );

	delay_ms(100);	


	// Init main sheduler
	scheduler_conf_t scheduler_config =
	{
		.max_task_count = 15,
		.schedule_strategy = ROUND_ROBIN,
		.debug = true
	};
	scheduler_init(&central_data.scheduler, &scheduler_config, &central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100); 

	// Init mavlink communication
	mavlink_communication_conf_t mavlink_config = 
	{	
		.scheduler_config =
		{
			.max_task_count = 30,
			.schedule_strategy = ROUND_ROBIN,
			.debug = true
		},
		.mavlink_stream_config = 
		{
			.rx_stream   = central_data.telemetry_up_stream,
			.tx_stream   = central_data.telemetry_down_stream,
			.sysid       = MAVLINK_SYS_ID,
			.compid      = 50,
			.use_dma     = false
		},
		.message_handler_config = 
		{
			.max_msg_callback_count = 20,
			.max_cmd_callback_count = 20,
			.debug                  = true
		},
		.onboard_parameters_config =
		{
			.max_param_count = MAX_ONBOARD_PARAM_COUNT,
			.debug           = true
		}
	};
	mavlink_communication_init(&central_data.mavlink_communication, &mavlink_config);
	
	delay_ms(100); 

	// Init state structure
	state_t state_config =
	{
		.mav_mode = { .byte = MAV_MODE_SAFE },
		.mav_state = MAV_STATE_BOOT,
		.simulation_mode = HIL_OFF,
		//.simulation_mode = HIL_ON,
		.autopilot_type = MAV_TYPE_QUADROTOR,
		.autopilot_name = MAV_AUTOPILOT_GENERIC,
		.sensor_present = 0b1111110000100111,
		.sensor_enabled = 0b1111110000100111,
		.sensor_health = 0b1111110000100111,
		.remote_active = 1
	};
	state_init(	&central_data.state,
				&state_config,
				&central_data.analog_monitor,
				&central_data.mavlink_communication.mavlink_stream,
				&central_data.mavlink_communication.message_handler); 
	
	delay_ms(100);

	state_machine_conf_t state_machine_conf =
	{
		.state_machine.use_mode_from_remote = 1
	};
	
	state_machine_init( &central_data.state_machine,
						&state_machine_conf,
						&central_data.state,
						&central_data.waypoint_handler,
						&central_data.sim_model,
						&central_data.remote);
	delay_ms(100);

	// Init imu
	imu_init(	&central_data.imu,
				&central_data.mavlink_communication.mavlink_stream);
	
	delay_ms(100);

	// Init ahrs
	ahrs_init(	&central_data.ahrs,
				&central_data.mavlink_communication.mavlink_stream);

	delay_ms(100);

	
	// Init qfilter
	qfilter_init(   &(central_data.attitude_filter), 
					&central_data.imu, 
					&central_data.ahrs);
	
	delay_ms(100);


	// Init remote
	remote_conf_t remote_config =
	{
		.type = REMOTE_TURNIGY,
		.mode_config =
		{
			.safety_channel = CHANNEL_GEAR,
			.safety_mode = 
			{
				.byte = MAV_MODE_ATTITUDE_CONTROL,
				// .flags =
				// {
				// .MANUAL = MANUAL_ON,
				// }
			},
			.mode_switch_channel = CHANNEL_FLAPS,
			.mode_switch_up = 
			{
				.byte = MAV_MODE_VELOCITY_CONTROL 
				// .flags =
				// {
				// .MANUAL = MANUAL_ON,
				// .STABILISE = STABILISE_ON,
				// }
			},
			.mode_switch_middle = 
			{
				.byte = MAV_MODE_POSITION_HOLD,
				// .flags =
				// {
				// .MANUAL = MANUAL_ON,
				// .GUIDED = GUIDED_ON,
				// }
			},
			.mode_switch_down = 
			{
				.byte = MAV_MODE_GPS_NAVIGATION
				// .flags =
				// {
				// .AUTO = AUTO_ON,
				// } 
			},
			.use_custom_switch = false,
			.custom_switch_channel = CHANNEL_AUX1,
			.use_test_switch = false,
			.test_switch_channel = CHANNEL_AUX2,
			.use_disable_remote_mode_switch = false,
			.test_switch_channel = CHANNEL_AUX2,
		},
	};
	remote_init( 	&central_data.remote, 
					&remote_config, 
					&central_data.mavlink_communication.mavlink_stream,
					&central_data.mavlink_communication.message_handler );

}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&central_data;
}