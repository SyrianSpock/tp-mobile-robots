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
 * \file remote.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This file is the driver for the remote control
 *
 ******************************************************************************/


#ifndef REMOTE_H__
#define REMOTE_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include "spektrum_satellite.h"
#include "scheduler.h"
#include "mavlink_stream.h" 

#include "mav_modes.h"

#define REMOTE_CHANNEL_COUNT 8


typedef enum
{
	SIGNAL_GOOD = 100,
	SIGNAL_BAD 	= 50,
	SIGNAL_LOST = 0,
} signal_quality_t;


typedef enum
{
	NORMAL 		= 1,
	INVERTED 	= -1,
} channel_inv_t;


typedef enum
{
	CHANNEL_THROTTLE = 0,
	CHANNEL_ROLL     = 1,
	CHANNEL_PITCH    = 2,
	CHANNEL_YAW      = 3,
	CHANNEL_GEAR     = 4,
	CHANNEL_FLAPS    = 5,
	CHANNEL_AUX1     = 6,
	CHANNEL_AUX2     = 7,
} remote_channel_t;


typedef enum
{
	REMOTE_TURNIGY	= 0,
	REMOTE_SPEKTRUM = 1,
} remote_type_t;


typedef struct
{
	remote_channel_t 	safety_channel;				///< See remote_mode_t for documentation
	mav_mode_t			safety_mode;				///< 
	remote_channel_t 	mode_switch_channel;		///< 
	mav_mode_t 			mode_switch_up;				///< 
	mav_mode_t 			mode_switch_middle;			///< 
	mav_mode_t 			mode_switch_down;			///< 
	bool				use_custom_switch;			///< 
	remote_channel_t 	custom_switch_channel;		///< 
	bool				use_test_switch;			///< 
	remote_channel_t 	test_switch_channel;		///< 
	bool				use_disable_remote_mode_switch;		///<
	remote_channel_t	disable_remote_mode_channel;			///<
} remote_mode_conf_t;


typedef struct
{
	remote_channel_t 	safety_channel;				///< Channel to use as 2-way "safety" switch. When 100%: safety mode, When -100%: normal mode (defined by mode_switch_channel)
	mav_mode_t			safety_mode;				///< Mode when the safety channel is at 100% (ARMED and HIL bit flags are ignored)
	remote_channel_t 	mode_switch_channel;		///< Channel to use as 3-way mode switch. The 3 corresponding modes are used when the safety channel is at -100%
	mav_mode_t 			mode_switch_up;				///< Mode when the mode switch is UP (ARMED and HIL bit flags are ignored)
	mav_mode_t 			mode_switch_middle;			///< Mode when the mode switch is MIDDLE (ARMED and HIL bit flags are ignored)
	mav_mode_t 			mode_switch_down;			///< Mode when the mode switch is DOWN (ARMED and HIL bit flags are ignored)
	bool				use_custom_switch;			///< Indicates whether a switch to activate the custom flag should be used
	remote_channel_t 	custom_switch_channel;		///< Channel to use as 2-way custom switch. If not in safety, the switch overrides the custom bit flag: 0 when switch is -100%, 1 when switch is 100%
	bool				use_test_switch;			///< Indicates whether a switch to activate the test flag should be used
	remote_channel_t 	test_switch_channel;		///< Channel to use as 2-way test switch. If not in safety, the switch overrides the test bit flag: 0 when switch is -100%, 1 when switch is 100%
	bool				use_disable_remote_mode_switch;		///< Indicates whether a switch should be used to use/override the mode indicated by the remote
	remote_channel_t	disable_remote_mode_channel;			///< Channel to use as 2-way switch. When 100%: follow mode indicated by the remote, when -100%: override what the remote indicates
	mav_mode_t			current_desired_mode;		///< Mav mode indicated by the remote
} remote_mode_t;


typedef struct
{
	remote_type_t type;
	remote_mode_conf_t mode_config;
} remote_conf_t;


typedef struct
{
	spektrum_satellite_t* sat;
	float channels[REMOTE_CHANNEL_COUNT];
	channel_inv_t channel_inv[REMOTE_CHANNEL_COUNT];
	float trims[REMOTE_CHANNEL_COUNT];
	float scale;
	int16_t deadzone;
	signal_quality_t signal_quality;
	remote_type_t type;
	remote_mode_t mode;
	const mavlink_stream_t* mavlink_stream;
} remote_t;


void remote_init(remote_t* remote, const remote_conf_t* config, const mavlink_stream_t* mavlink_stream, mavlink_message_handler_t *mavlink_handler);


task_return_t remote_update(remote_t* remote);


signal_quality_t remote_check(const remote_t* remote);


/**
 * \brief Update the remote channel central position array stored in .c file
 * Warning: you should ensure first that the remote has the stick in their neutral position
 *
 * \param index Specify which channel we are interested in
 */
void remote_calibrate(remote_t* remote, remote_channel_t channel);


float remote_get_throttle(const remote_t* remote);


float remote_get_roll(const remote_t* remote);


float remote_get_pitch(const remote_t* remote);


float remote_get_yaw(const remote_t* remote);


void remote_mode_init(remote_mode_t* remote_mode, const remote_mode_conf_t* config);


void remote_mode_update(remote_t* remote);


mav_mode_t remote_mode_get(const remote_t* remote);


task_return_t remote_send_raw(const remote_t* remote);


task_return_t remote_send_scaled(const remote_t* remote);


#ifdef __cplusplus
	}
#endif

#endif