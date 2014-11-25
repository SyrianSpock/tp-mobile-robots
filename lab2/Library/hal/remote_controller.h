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
 * \file remote_controller.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the remote control
 * 
 ******************************************************************************/


#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "conf_platform.h"
#include "mavlink_communication.h"
#include "time_keeper.h"
#include "stabilisation.h"

#ifdef SPEKTRUM_REMOTE				///< If you use the SPEKTRUM remote
	#include "spektrum.h"
	#include "spektrum_satellite.h"
#endif

#ifdef TURNIGY_REMOTE				///< If you use the TURNIGY remote
	#include "turnigy.h"
	#include "spektrum_satellite.h"
#endif

#ifdef JOYSTICK_REMOTE				///< If you use the JOYSTICK as a remote
	#include "joystick.h"
	#include "joystick_rc.h"
#endif


/**
 * \brief Return the roll angle from the remote
 *
 * \return the roll angle from the remote
 */
static float inline remote_controller_get_roll_from_remote(void)	
{
	return spektrum_satellite_get_neutral(RC_ROLL) * RC_ROLL_DIR * RC_SCALEFACTOR; 
}


/**
 * \brief Return the pitch angle from the remote
 *
 * \return the pitch angle from the remote
 */
static float inline remote_controller_get_pitch_from_remote(void)	
{
	return spektrum_satellite_get_neutral(RC_PITCH) * RC_PITCH_DIR * RC_SCALEFACTOR; 
}


/**
 * \brief Return the yaw angle from the remote
 *
 * \return the yaw angle from the remote
 */
static float inline remote_controller_get_yaw_from_remote(void)	
{
	return spektrum_satellite_get_neutral(RC_YAW) * RC_YAW_DIR * RC_SCALEFACTOR; 
}


/**
 * \brief Return the thrust command from the remote
 *
 * \return the thrust command from the remote
 */
static float inline remote_controller_get_thrust_from_remote(void)	
{
	return spektrum_satellite_get_channel(RC_THROTTLE) * RC_THROTTLE_DIR * RC_SCALEFACTOR; 
}


/**
 * \brief return an object containing the stick position of the remote (roll, pitch, yaw and thrust)
 *
 * \return an object containing the stick position of the remote (roll, pitch, yaw and thrust)
 */
static inline void remote_controller_get_command_from_remote(control_command_t * controls)
{
	controls->rpy[ROLL]= remote_controller_get_roll_from_remote() * RC_INPUT_SCALE;
	controls->rpy[PITCH]= remote_controller_get_pitch_from_remote() * RC_INPUT_SCALE;
	controls->rpy[YAW]= remote_controller_get_yaw_from_remote() * RC_INPUT_SCALE;
	controls->thrust = remote_controller_get_thrust_from_remote();
}


/**
 * \brief return the motor state to switch on/off the motors
 *
 * \param	motor_state		The pointer to the motor state
 */
static inline void remote_controller_get_motor_state(int8_t *motor_state)
{
	if((remote_controller_get_thrust_from_remote() < -0.95f) && (remote_controller_get_yaw_from_remote() > 0.9f))
	{
		*motor_state = 1;
	}
	else if((remote_controller_get_thrust_from_remote() < -0.95f) && (remote_controller_get_yaw_from_remote() < -0.9f))
	{
		*motor_state = -1;
	}
	else
	{
		*motor_state = 0;
	}
}


/**
 * \brief							Gets the velocity vector from the remote
 *
 * \param	controls				The pointer to the control structure 
 */
static inline void remote_controller_get_velocity_vector_from_remote(control_command_t* controls)
{
	
	controls->tvel[X]= - 10.0f * remote_controller_get_pitch_from_remote() * RC_INPUT_SCALE;
	controls->tvel[Y]= 10.0f * remote_controller_get_roll_from_remote() * RC_INPUT_SCALE;
	controls->tvel[Z]= - 1.5f * remote_controller_get_thrust_from_remote();
	controls->rpy[YAW] = remote_controller_get_yaw_from_remote() * RC_INPUT_SCALE;
}


/**
 * \brief return a switch state of the remote
 *
 * \param chan_switch pointer to a channel switch
 */
static inline void remote_controller_get_channel_mode(uint8_t* chan_switch)
{
	if (spektrum_satellite_get_channel(RC_SAFETY) < 0)
	{
		*chan_switch |= 0x00;
	}
	else if(spektrum_satellite_get_channel(RC_SAFETY) > 0 && spektrum_satellite_get_channel(RC_ID_MODE) < 0)
	{
		*chan_switch |= 0x01;
	}
	else if (spektrum_satellite_get_channel(RC_SAFETY) > 0 && spektrum_satellite_get_channel(RC_ID_MODE) > 20)
	{
		*chan_switch |= 0x03;
	}
	else
	{
		*chan_switch |= 0x02;
	}
}


/**
 * \brief	Task to send the mavlink RC scaled message
 * 
 * \return	The status of execution of the task
 */
// static inline task_return_t remote_controller_send_scaled_rc_channels(control_command_t* controls)
// {
// 	mavlink_message_t msg;
	
// 	mavlink_msg_rc_channels_scaled_pack(	controls->mavlink_stream->sysid,
// 											controls->mavlink_stream->compid,
// 											&msg,
// 											time_keeper_get_millis(),
// 											1,
// 											spektrum_satellite_get_channel(0) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(1) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(2) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(3) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(4) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(5) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(6) * 1000.0f * RC_SCALEFACTOR,
// 											spektrum_satellite_get_channel(7) * 1000.0f * RC_SCALEFACTOR,
// 											0	);
	
// 	mavlink_stream_send(controls->mavlink_stream,&msg);
	
// 	return TASK_RUN_SUCCESS;
// }


/**
 * \brief	Task to send the mavlink RC raw message
 * 
 * \return	The status of execution of the task
 */
// static inline task_return_t remote_controller_send_raw_rc_channels(control_command_t* controls)
// {
// 	// mavlink_message_t msg;
	
// 	// mavlink_msg_rc_channels_raw_pack(	controls->mavlink_stream->sysid,
// 	// 									controls->mavlink_stream->compid,
// 	// 									&msg,
// 	// 									time_keeper_get_millis(),
// 	// 									1,
// 	// 									spektrum_satellite_get_channel(0) + 1000,
// 	// 									spektrum_satellite_get_channel(1) + 1000,
// 	// 									spektrum_satellite_get_channel(2) + 1000,
// 	// 									spektrum_satellite_get_channel(3) + 1000,
// 	// 									spektrum_satellite_get_channel(4) + 1000,
// 	// 									spektrum_satellite_get_channel(5) + 1000,
// 	// 									spektrum_satellite_get_channel(6) + 1000,
// 	// 									spektrum_satellite_get_channel(7) + 1000,
// 	// 									spektrum_satellite_check()	);
	
// 	// mavlink_stream_send(controls->mavlink_stream,&msg);
	
// 	return TASK_RUN_SUCCESS;
// }


#ifdef __cplusplus
	}
#endif

#endif //REMOTE_CONTROLLER_H_