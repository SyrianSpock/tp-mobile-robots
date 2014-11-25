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
#include "radar_module_driver.h"
#include "analog_monitor.h"
#include "tasks.h"
#include "mavlink_waypoint_handler.h"
#include "analog_monitor.h"
#include "state.h"
#include "position_estimation.h"
#include "hud.h"
#include "ahrs.h"
#include "remote.h"

central_data_t *central_data;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

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

void mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters)
{	
	
	// System ID	
	onboard_parameters_add_parameter_int32    ( onboard_parameters , (int32_t*)&central_data->mavlink_communication.mavlink_stream.sysid              , "ID_SYSID"         );

	// qfilter
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp											, "QF_kp_acc"        );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.ki											, "QF_ki_acc"        );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp_mag										, "QF_kp_mag"        );
	//onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain				, "YawAPid_D_Gain"   );
	
	// Biaises
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[X]									  , "Bias_Gyro_X"      );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Y]									  , "Bias_Gyro_Y"      );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Z]									  , "Bias_Gyro_Z"      );
	
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[X]								  , "Bias_Acc_X"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Y]								  , "Bias_Acc_Y"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Z]								  , "Bias_Acc_Z"       );
	
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[X]								  , "Bias_Mag_X"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Y]								  , "Bias_Mag_Y"       );
	onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Z]								  , "Bias_Mag_Z"       );
	
	// Scale factor
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[X]							  , "Scale_Gyro_X"     );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Y]							  , "Scale_Gyro_Y"     );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Z]							  , "Scale_Gyro_Z"     );
	
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[X]                       , "Scale_Acc_X"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Y]                       , "Scale_Acc_Y"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Z]                       , "Scale_Acc_Z"      );
	
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[X]                        , "Scale_Mag_X"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Y]                        , "Scale_Mag_Y"      );
	onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Z]                        , "Scale_Mag_Z"      );
	
	
	onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state.remote_active,"Remote_Active");
	onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state_machine.use_mode_from_remote, "Remote_Use_Mode");

	onboard_parameters_add_parameter_int32(onboard_parameters,(int32_t*)&central_data->data_logging.log_data, "Log_continue");

}

	
//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_init(void)
{
	central_data = central_data_get_pointer_to_struct();
	
	mavlink_telemetry_add_onboard_parameters(&central_data->mavlink_communication.onboard_parameters);

	//mavlink_telemetry_add_data_logging_parameters(&central_data->data_logging);

	scheduler_t* mavlink_scheduler = &central_data->mavlink_communication.scheduler; 

	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&state_send_heartbeat,								&central_data->state, 					MAVLINK_MSG_ID_HEARTBEAT	);							// ID 0
	//scheduler_add_task(mavlink_scheduler,  1000000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&state_send_status,									&central_data->state,					MAVLINK_MSG_ID_SYS_STATUS	);							// ID 1
	scheduler_add_task(mavlink_scheduler,  100000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_scaled,									&central_data->imu, 								MAVLINK_MSG_ID_SCALED_IMU	);							// ID 26
	scheduler_add_task(mavlink_scheduler,  100000,    RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_raw,										&central_data->imu, 								MAVLINK_MSG_ID_RAW_IMU	);								// ID 27
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&ahrs_send_attitude,								&central_data->ahrs,				 				MAVLINK_MSG_ID_ATTITUDE	);								// ID 30
	//scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&ahrs_send_attitude_quaternion,						&central_data->ahrs,				 				MAVLINK_MSG_ID_ATTITUDE_QUATERNION	);					// ID 31
	
	//scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_send_scaled,								&central_data->remote,								MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);					// ID 34

	//scheduler_add_task(mavlink_scheduler,  250000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_send_raw,									&central_data->remote,								MAVLINK_MSG_ID_RC_CHANNELS_RAW	);						// ID 35
	//scheduler_add_task(mavlink_scheduler,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&servos_mavlink_send,								&central_data->servos, 								MAVLINK_MSG_ID_SERVO_OUTPUT_RAW	);						// ID 36
	//scheduler_add_task(mavlink_scheduler,  250000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&joystick_parsing_send_manual_ctrl_msg,				&central_data->joystick_parsing,					MAVLINK_MSG_ID_MANUAL_CONTROL);	// ID 69
	//scheduler_add_task(mavlink_scheduler,  250000,   RUN_NEVER,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&scheduler_send_rt_stats,							&central_data->scheduler, 						MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251
	// scheduler_add_task(mavlink_scheduler,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&mavlink_telemetry_send_sonar,						&central_data->i2cxl_sonar, 						MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251

	scheduler_sort_tasks(mavlink_scheduler);
	
	print_util_dbg_print("MAVlink telemetry initialiased\r\n");
}