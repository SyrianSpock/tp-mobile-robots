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

#include "conf_platform.h"
#include "conf_constants.h"

#include "delay.h"
#include "servos_default_config.h"
#include "qfilter_default_config.h"
#include "position_estimation_default_config.h"
#include "scheduler_default_config.h"
#include "mavlink_communication_default_config.h"
#include "state_default_config.h"
#include "state_machine_default_config.h"
#include "ahrs_default_config.h"
#include "navigation_default_config.h"
#include "stabilisation_copter_default_config.h"
#include "simulation_default_config.h"
#include "attitude_controller_p2_default_config.h"
#include "servos_mix_quadcopter_diag_default_config.h"
#include "remote_default_config.h"
#include "data_logging_default_config.h"

//#include "imu_default_config.h"
#include "imu_config.h"

#include "attitude_controller_default_config.h"
#include "velocity_controller_copter_default_config.h"
 
static central_data_t central_data;

void central_data_init()
{	
	//--------------------------------------------------------------------------
	// Init servos
	servos_init( &central_data.servos, &servos_default_config );
	servos_set_value_failsafe( &central_data.servos );
	pwm_servos_write_to_hardware( &central_data.servos );	

	//--------------------------------------------------------------------------
	// Init main sheduler
	scheduler_init(	&central_data.scheduler, &scheduler_default_config );

	//--------------------------------------------------------------------------	 
	// Init mavlink communication
	mavlink_communication_conf_t mavlink_config = mavlink_communication_default_config;
	mavlink_config.mavlink_stream_config.sysid = MAVLINK_SYS_ID;
	mavlink_communication_init(	&central_data.mavlink_communication, 
								&mavlink_config,
								central_data.telemetry_up_stream, 
								central_data.telemetry_down_stream);

	//--------------------------------------------------------------------------
	// Init state structure
	state_t state_config = state_default_config;
	state_config.simulation_mode = HIL_ON;
	state_config.remote_active = 0;
	state_init(	&central_data.state,
				&state_config,
				&central_data.analog_monitor ); 
	
	//--------------------------------------------------------------------------
	// Init state machine	
	state_machine_conf_t state_machine_config =
	{
		.state_machine.use_mode_from_remote = 0
	};
	state_machine_init( &central_data.state_machine,
						&state_machine_config,
						&central_data.state,
						&central_data.waypoint_handler,
						&central_data.sim_model,
						&central_data.remote);
						
	//--------------------------------------------------------------------------
	// Init imu
	imu_init(	&central_data.imu,
				//&imu_default_config,
				&imu_config,
				&central_data.state );	

	//--------------------------------------------------------------------------
	// Init ahrs
	ahrs_init(	&central_data.ahrs,
				&ahrs_default_config );

	//--------------------------------------------------------------------------
	// Init qfilter
	qfilter_init(   &central_data.attitude_filter, 
					&qfilter_default_config,
					&central_data.imu, 
					&central_data.ahrs);
	
	//--------------------------------------------------------------------------	
	// Init position_estimation_init
	position_estimation_init(   &central_data.position_estimation,
								&position_estimation_default_config,
								&central_data.state,
								&central_data.pressure,
								&central_data.gps,
								&central_data.ahrs,
								&central_data.imu);

	//--------------------------------------------------------------------------
	// Init navigation
	navigation_init(&central_data.navigation,
					&navigation_default_config,
					&central_data.controls_nav,
					&central_data.ahrs.qe,
					&central_data.waypoint_handler,
					&central_data.position_estimation,
					&central_data.state,
					&central_data.controls_joystick,
					&central_data.remote,
					&central_data.mavlink_communication);
	
	//--------------------------------------------------------------------------
	// Init waypont handler
	waypoint_handler_init(  &central_data.waypoint_handler,
							&central_data.position_estimation,
							&central_data.ahrs,
							&central_data.state,
							&central_data.mavlink_communication,
							&central_data.mavlink_communication.mavlink_stream);
	// waypoint_handler_init_homing_waypoint(&central_data.waypoint_handler);
	waypoint_handler_nav_plan_init(&central_data.waypoint_handler);
	
	//--------------------------------------------------------------------------
	// Init stabilisers
	stabilisation_copter_init(	&central_data.stabilisation_copter,
								&stabilisation_copter_default_config,
								&central_data.controls,
								&central_data.imu,
								&central_data.ahrs,
								&central_data.position_estimation,
								&central_data.servos );
	
	stabilisation_init( &central_data.controls );
	
	//--------------------------------------------------------------------------
	// Init simulation (should be done after position_estimation)
	simulation_init(&central_data.sim_model,
					&simulation_default_config,
					&central_data.ahrs,
					&central_data.imu,
					&central_data.position_estimation,
					&central_data.pressure,
					&central_data.gps,
					&central_data.state,
					&central_data.servos,
					&central_data.state.nav_plan_active );

	//--------------------------------------------------------------------------
	// Init hud	
	hud_telemetry_init(	&central_data.hud_structure, 
						&central_data.position_estimation, 
						&central_data.controls, 
						&central_data.ahrs );
	
	//--------------------------------------------------------------------------
	// Init joystick
	joystick_parsing_init(	&central_data.joystick_parsing,
							&central_data.controls_joystick,
							&central_data.state );

	//--------------------------------------------------------------------------	
	// Init sonar
	// sonar_i2cxl_init( 	&central_data.sonar_i2cxl );

	//--------------------------------------------------------------------------	
	// Init P^2 attitude controller
	attitude_controller_p2_init( 	&central_data.attitude_controller_p2,
									&attitude_controller_p2_default_config,
									&central_data.command.attitude,
									&central_data.command.torque,
									&central_data.ahrs );

	//--------------------------------------------------------------------------	
	// Init attitude controller
	attitude_controller_init( 	&central_data.attitude_controller,
								&attitude_controller_default_config,
								&central_data.ahrs,
								&central_data.command.attitude,
								&central_data.command.rate,
								&central_data.command.torque );

	//--------------------------------------------------------------------------	
	// Init velocity controller
	velocity_controller_copter_init( 	&central_data.velocity_controller,
										&velocity_controller_copter_default_config,
										&central_data.ahrs,
										&central_data.position_estimation,
										&central_data.command.velocity,
										&central_data.command.attitude,
										&central_data.command.thrust );

	//--------------------------------------------------------------------------	
	// Init vector field navigation
	vector_field_waypoint_conf_t vector_field_config;
	vector_field_waypoint_init( &central_data.vector_field_waypoint,
								&vector_field_config,
								&central_data.waypoint_handler,
								&central_data.position_estimation,
								&central_data.command.velocity );

	//--------------------------------------------------------------------------	
	// Init servo mixing
	servo_mix_quadcotper_diag_init( &central_data.servo_mix, 
									&servo_mix_quadcopter_diag_default_config, 
									&central_data.command.torque, 
									&central_data.command.thrust, 
									&central_data.servos);

	//--------------------------------------------------------------------------
	// Init remote
	remote_init( 	&central_data.remote, 
					&remote_default_config );

	//--------------------------------------------------------------------------
	// Init data logging
	data_logging_init(  &central_data.data_logging,
						&data_logging_default_config);

}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&central_data;
}