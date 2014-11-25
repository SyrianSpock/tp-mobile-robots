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
 * \file mavlink_telemetry.c
 * 
 * \author MAV'RIC Team
 *   
 * \brief Definition of the messages sent by the autopilot to the ground station
 *
 ******************************************************************************/


#include "mavlink_telemetry.h"
#include "central_data.h"
#include "onboard_parameters.h"
#include "mavlink_stream.h"
#include "scheduler.h"
#include "analog_monitor.h"
#include "tasks.h"
#include "mavlink_waypoint_handler.h"
#include "analog_monitor.h"
#include "state.h"
#include "position_estimation.h"
#include "hud_telemetry.h"
#include "ahrs.h"
#include "remote.h"

#include "remote_telemetry.h"
#include "servos_telemetry.h"
#include "state_telemetry.h"
#include "gps_ublox_telemetry.h"
#include "imu_telemetry.h"
#include "bmp085_telemetry.h"
#include "ahrs_telemetry.h"
#include "position_estimation_telemetry.h"
#include "stabilisation_telemetry.h"
#include "joystick_parsing_telemetry.h"
#include "simulation_telemetry.h"
#include "scheduler_telemetry.h"
#include "sonar_telemetry.h"

#include "constants.h"

central_data_t *central_data;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Initialise the callback functions
 * 
 * \param   central_data            The pointer to the central_data structure
 */
void mavlink_telemetry_init_communication_module(central_data_t *central_data);


/**
 * \brief   Add all onboard parameters to the parameter list
 *
 * \param	onboard_parameters		The pointer to the onboard parameters structure
 */
void mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters);


/**
 * \brief	Add onboard logging parameters
 *
 * \param	data_logging			The pointer to the data logging structure
 */
void mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_init_communication_module(central_data_t *central_data)
{
    state_telemetry_init(   &central_data->state,
                            &central_data->mavlink_communication.message_handler);

    imu_telemetry_init( &central_data->imu,
                        &central_data->mavlink_communication.message_handler);

    remote_telemetry_init(  &central_data->remote,
                            &central_data->mavlink_communication.message_handler);

    joystick_parsing_telemetry_init(&central_data->joystick_parsing,
                                    &central_data->mavlink_communication.message_handler);

    simulation_telemetry_init(  &central_data->sim_model,
                                &central_data->mavlink_communication.message_handler);
}


void mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters)
{

	pid_controller_t* rate_pid 		= central_data->attitude_controller.rate_pid;
	pid_controller_t* angle_pid 	= central_data->attitude_controller.angle_pid;
	pid_controller_t* velocity_pid 	= central_data->velocity_controller.pid;
//	stabiliser_t* velocity_stabiliser= &central_data->stabilisation_copter.stabiliser_stack.velocity_stabiliser;
	
	// System ID	
	onboard_parameters_add_parameter_int32( onboard_parameters, 
											(int32_t*)&central_data->mavlink_communication.mavlink_stream.sysid, 
											"ID_SYSID" );

	// Simulation mode
	onboard_parameters_add_parameter_int32( onboard_parameters, 
											(int32_t*)&central_data->state.simulation_mode, 
											"Sim_mode" );
	
	// Roll rate PID
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].p_gain, 			"RRollPid_P_G"		);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].integrator.postgain,"RRollPid_I_PstG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].integrator.pregain, "RRollPid_I_PreG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].differentiator.gain,"RRollPid_D_Gain"	);
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].integrator.clip          , "RRollPid_I_CLip"  );
	// onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].differentiator.clip      , "RRollPid_D_Clip"  );
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[ROLL].differentiator.LPF             , "RRollPid_D_LPF"   );
	
	// Roll attitude PID
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[ROLL].p_gain,				"ARollPid_P_G"		);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[ROLL].integrator.postgain,	"ARollPid_I_PstG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[ROLL].integrator.pregain,	"ARollPid_I_PreG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[ROLL].differentiator.gain, 	"ARollPid_D_Gain"	);
	//onboard_parameters_add_parameter_float() onboard_parameters, &angle_pid[ROLL].integrator.clip      , "ARollPid_I_CLip"  );
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[ROLL].differentiator.clip  , "ARollPid_D_Clip"  );
	// onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[ROLL].differentiator.LPF         , "ARollPid_D_LPF"   );

	// Pitch rate PID
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].p_gain, 				"RPitchPid_P_G"		);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].integrator.postgain, 	"RPitchPid_I_PstG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].integrator.pregain,	"RPitchPid_I_PreG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].differentiator.gain,	"RPitchPid_D_Gain"	);
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].integrator.clip         , "RPitchPid_I_CLip" );
	// onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].differentiator.clip     , "RPitchPid_D_Clip" );
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[PITCH].differentiator.LPF            , "RPitchPid_D_LPF"  );
	
	// Pitch attitude PID
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[PITCH].p_gain,				"APitchPid_P_G"		);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[PITCH].integrator.postgain,	"APitchPid_I_PstG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[PITCH].integrator.pregain,	"APitchPid_I_PreG"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[PITCH].differentiator.gain, 	"APitchPid_D_Gain"	);
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[PITCH].integrator.clip     , "APitchPid_I_CLip" );
	//onboard_parameters_add_parameter_float() onboard_parameters, &angle_pid[PITCH].differentiator.clip , "APitchPid_D_Clip" );
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[PITCH].differentiator.LPF        , "APitchPid_D_LPF"  );

	// Yaw rate PID
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].p_gain,				"RYawPid_P_G"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].integrator.postgain, "RYawPid_I_PstG");
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].integrator.pregain,	"RYawPid_I_PreG");
	onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].differentiator.gain,	"RYawPid_D_Gain");
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].clip_max                        , "RYawPid_P_CLmx"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].clip_min                        , "RYawPid_P_CLmn"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].integrator.clip           , "RYawPid_I_CLip"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].differentiator.clip       , "RYawPid_D_Clip"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &rate_pid[YAW].differentiator.LPF              , "RYawPid_D_LPF"    );
	
	// Yaw attitude PID
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].p_gain,				"AYawPid_P_G"	);
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].integrator.postgain,"AYawPid_I_PstG");
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].integrator.pregain,	"AYawPid_I_PreG");
	onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].differentiator.gain,"AYawPid_D_Gain");
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].clip_max                    , "AYawPid_P_CLmx"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].clip_min                    , "AYawPid_P_CLmn"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].integrator.clip       , "AYawPid_I_CLip"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].differentiator.clip   , "AYawPid_D_Clip"   );
	//onboard_parameters_add_parameter_float( onboard_parameters, &angle_pid[YAW].differentiator.LPF          , "AYawPid_D_LPF"    );


	// X velocity PID
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[X].p_gain, 				"VelXPid_P_G"   );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[X].integrator.postgain, 	"VelXPid_I_PstG");
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[X].integrator.pregain, 	"VelXPid_I_PreG");
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[X].differentiator.gain, 	"VelXPid_D_Gain");

	// Y velocity PID
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Y].p_gain, 				"VelYPid_P_G"    );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Y].integrator.postgain, 	"VelYPid_I_PstG" );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Y].integrator.pregain, 	"VelYPid_I_PreG" );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Y].differentiator.gain, 	"VelYPid_D_Gain" );

	// Z velocity PID
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Z].p_gain, 				"VelZPid_P_G"      );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Z].integrator.postgain, 	"VelZPid_I_PstG"   );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Z].integrator.pregain, 	"VelZPid_I_PreG"   );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Z].differentiator.gain, 	"VelZPid_D_Gain"   );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Z].differentiator.LPF, 	"VelZPid_D_LPF"    );
	onboard_parameters_add_parameter_float( onboard_parameters, &velocity_pid[Z].soft_zone_width, 		"VelZPid_soft"     );



	// qfilter
	// onboard_parameters_add_parameter_float(onboard_parameters, &central_data->attitude_filter.kp                                        , "QF_kp_acc"        );
	// onboard_parameters_add_parameter_float(onboard_parameters, &central_data->attitude_filter.kp_mag                                    , "QF_kp_mag"        );
	
	// Biaises
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_gyro.bias[X]									  , "Bias_Gyro_X"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_gyro.bias[Y]									  , "Bias_Gyro_Y"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_gyro.bias[Z]									  , "Bias_Gyro_Z"      );
	
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_accelero.bias[X]								  , "Bias_Acc_X"       );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_accelero.bias[Y]								  , "Bias_Acc_Y"       );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_accelero.bias[Z]								  , "Bias_Acc_Z"       );
	
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_compass.bias[X]								  , "Bias_Mag_X"       );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_compass.bias[Y]								  , "Bias_Mag_Y"       );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_compass.bias[Z]								  , "Bias_Mag_Z"       );
	
	// Scale factor
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_gyro.scale_factor[X]							  , "Scale_Gyro_X"     );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_gyro.scale_factor[Y]							  , "Scale_Gyro_Y"     );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_gyro.scale_factor[Z]							  , "Scale_Gyro_Z"     );
	
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_accelero.scale_factor[X]                       , "Scale_Acc_X"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_accelero.scale_factor[Y]                       , "Scale_Acc_Y"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_accelero.scale_factor[Z]                       , "Scale_Acc_Z"      );
	
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_compass.scale_factor[X]                        , "Scale_Mag_X"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_compass.scale_factor[Y]                        , "Scale_Mag_Y"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->imu.calib_compass.scale_factor[Z]                        , "Scale_Mag_Z"      );

	// Position estimation
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->position_estimation.kp_alt_baro                              , "Pos_kp_alt_baro"       );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->position_estimation.kp_vel_baro                              , "Pos_kp_velb"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->position_estimation.kp_pos_gps[0]                            , "Pos_kp_pos0"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->position_estimation.kp_pos_gps[1]                            , "Pos_kp_pos1"      );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->position_estimation.kp_pos_gps[2]                            , "Pos_kp_pos2"      );
	
	// Navigation
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->navigation.dist2vel_gain                            , "vel_dist2Vel"     );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->navigation.cruise_speed                            , "vel_cruiseSpeed"  );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->navigation.max_climb_rate                          , "vel_climbRate"    );
	onboard_parameters_add_parameter_float(onboard_parameters, &central_data->navigation.soft_zone_size							  , "vel_softZone"     );
	
	// Remote
	onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state.remote_active,"Remote_Active");
	onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state_machine.use_mode_from_remote, "Remote_Use_Mode");

	// Data logging
	onboard_parameters_add_parameter_int32(onboard_parameters,(int32_t*)&central_data->data_logging.log_data, "Log_continue");
}

void mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging)
{
	// if _USE_LFN == 0: Name: max 8 characters + 3 for extension; if _USE_LFN != 0: Name: max 255 characters + more flexible extension type
	data_logging_create_new_log_file(data_logging, "NewFile");
	
	// Add your logging parameters here, name length max = MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN = 16
	// Supported type: all numeric types included in mavlink_message_type_t (i.e. all except MAVLINK_TYPE_CHAR)
	
	data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[X], "acc_x");
	data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Y], "acc_y");
	data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Z], "acc_z");
	
	data_logging_add_parameter_double(data_logging, &central_data->gps.latitude, "latitude");
	data_logging_add_parameter_double(data_logging, &central_data->gps.longitude, "longitude");
	data_logging_add_parameter_float(data_logging, &central_data->gps.altitude, "altitude");
	
	// data_logging_add_parameter_int8(data_logging, &central_data->state_machine.rc_check, "rc_check");
	//data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state_machine.rc_check, "rc_check");
	
	//data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state.mav_state, "mav_state");
	data_logging_add_parameter_uint8(data_logging, &central_data->state.mav_mode.byte, "mav_mode");
	
};
	
//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_init(void)
{
	central_data = central_data_get_pointer_to_struct();
	
	mavlink_telemetry_add_onboard_parameters(&central_data->mavlink_communication.onboard_parameters);

	mavlink_telemetry_add_data_logging_parameters(&central_data->data_logging);

	mavlink_telemetry_init_communication_module(central_data);

	mavlink_communication_t* mavlink_communication = &central_data->mavlink_communication;

	stabiliser_t* stabiliser_show = &central_data->stabilisation_copter.stabiliser_stack.rate_stabiliser;

	// Hearbeat, status and HUD
	mavlink_communication_add_msg_send(	mavlink_communication,
										1000000,	
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&state_telemetry_send_heartbeat,		
										&central_data->state,					
										0	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										1000000,	
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&state_telemetry_send_status,			
										&central_data->state,					
										1	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&hud_telemetry_send_message,						
										&central_data->hud_structure,			
										16	);

	// Scheduler
	mavlink_communication_add_msg_send(	mavlink_communication,	
										250000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&scheduler_telemetry_send_rt_stats,	
										&central_data->scheduler,				
										19	);
	
	// Simulation
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&simulation_telemetry_send_state,		
										&central_data->sim_model,				
										17	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&simulation_telemetry_send_quaternions,
										&central_data->sim_model,				
										18	);	

	// Baro and GPS
	mavlink_communication_add_msg_send(	mavlink_communication,	
										1000000,	
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&gps_ublox_telemetry_send_raw,			
										&central_data->gps,						
										2	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&bmp085_telemetry_send_pressure,		
										&central_data->pressure,				
										3	);
	
	// IMU and AHRS
	mavlink_communication_add_msg_send(	mavlink_communication,	
										250000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&imu_telemetry_send_scaled,			
										&central_data->imu,						
										4	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										100000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&imu_telemetry_send_raw,				
										&central_data->imu,						
										5	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										200000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&ahrs_telemetry_send_attitude,			
										&central_data->ahrs,					
										6	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&ahrs_telemetry_send_attitude_quaternion,	
										&central_data->ahrs,					
										7	);
	
	// Position estimation
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&position_estimation_telemetry_send_position,			
										&central_data->position_estimation,		
										8	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										250000,		
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&position_estimation_telemetry_send_global_position,	
										&central_data->position_estimation,		
										9	);
	
	// Remote and joystick
	mavlink_communication_add_msg_send(	mavlink_communication,	
										500000,		
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&remote_telemetry_send_scaled,					
										&central_data->remote,					
										10	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										250000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&remote_telemetry_send_raw,						
										&central_data->remote,					
										11	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										250000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&joystick_parsing_telemetry_send_manual_ctrl_msg,	
										&central_data->joystick_parsing,		
										15 	);
	
	// Servos
	mavlink_communication_add_msg_send(	mavlink_communication,	
										1000000,	
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&servos_telemetry_mavlink_send,	
										&central_data->servos,					
										12	);
	
	// Stabilisation
	mavlink_communication_add_msg_send(	mavlink_communication,	
										200000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&stabilisation_telemetry_send_rpy_thrust_setpoint,	
										&central_data->controls,				
										13	);
	mavlink_communication_add_msg_send(	mavlink_communication,	
										200000,		
										RUN_NEVER,		
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&stabilisation_telemetry_send_rpy_speed_thrust_setpoint,		stabiliser_show,						14	);
	
	// Others
	mavlink_communication_add_msg_send(	mavlink_communication,	
										100000,		
										RUN_REGULAR,	
										PERIODIC_ABSOLUTE,	
										PRIORITY_NORMAL,	
										(mavlink_send_msg_function_t)&sonar_telemetry_send,						
										&central_data->sonar_i2cxl.data,				
										20	);


	scheduler_sort_tasks(&central_data->mavlink_communication.scheduler);
	
	print_util_dbg_print("MAVlink telemetry initialiased\r\n");
}