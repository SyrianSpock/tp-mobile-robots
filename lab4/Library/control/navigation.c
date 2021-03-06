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
 * \file navigation.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Waypoint navigation controller
 *
 ******************************************************************************/


#include "navigation.h"
#include "conf_platform.h"
#include "print_util.h"
#include "time_keeper.h"

#define KP_YAW 0.2f

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief					Computes the relative position and distance to the given way point
 *
 * \param	waypoint_pos		Local coordinates of the waypoint
 * \param	rel_pos			Array to store the relative 3D position of the waypoint
 * \param	local_pos		The 3D array of the actual position
 *
 * \return					Distance to waypoint squared
 */
static float navigation_set_rel_pos_n_dist2wp(float waypoint_pos[], float rel_pos[], const float local_pos[3]);


/**
 * \brief					Sets the Robot speed to reach waypoint
 *
 * \param	rel_pos			Relative position of the waypoint
 * \param	navigation	The structure of navigation data
 */
static void navigation_set_speed_command(float rel_pos[], navigation_t* navigation);

/**
 * \brief						Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param	waypoint_input		Destination waypoint in local coordinate system
 * \param	navigation			The navigation structure
 */
static void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigation);

/**
 * \brief	Sets auto-takeoff procedure from a mavlink command message MAV_CMD_NAV_TAKEOFF
 *
 * \param	navigation			The pointer to the navigation structure
 * \param	packet				The pointer to the structure of the mavlink command message long
 */
static void navigation_set_auto_takeoff(navigation_t *navigation, mavlink_command_long_t* packet);

/**
 * \brief	Check if the nav mode is equal to the state mav mode
 *
 * \param	navigation			The pointer to the navigation structure
 *
 * \return	True if the flag STABILISE, GUIDED and ARMED are equal, false otherwise
 */
static bool navigation_mode_change(navigation_t* navigation);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float navigation_set_rel_pos_n_dist2wp(float waypoint_pos[], float rel_pos[], const float local_pos[3])
{
	float dist2wp_sqr;
	
	rel_pos[X] = (float)(waypoint_pos[X] - local_pos[X]);
	rel_pos[Y] = (float)(waypoint_pos[Y] - local_pos[Y]);
	rel_pos[Z] = (float)(waypoint_pos[Z] - local_pos[Z]);
	
	dist2wp_sqr = vectors_norm_sqr(rel_pos);
	
	return dist2wp_sqr;
}


static void navigation_set_speed_command(float rel_pos[], navigation_t* navigation)
{
	float  norm_rel_dist, v_desired;
	quat_t qtmp1, qtmp2;
	
	float dir_desired_bf[3];
	
	float rel_heading;
	
	norm_rel_dist = sqrt(navigation->waypoint_handler->dist2wp_sqr);
	
	if (norm_rel_dist < 0.0005f)
	{
		norm_rel_dist += 0.0005f;
	}

	if (((navigation->state->mav_mode.GUIDED == GUIDED_ON)&&(navigation->state->mav_mode.AUTO == AUTO_OFF))||((maths_f_abs(rel_pos[X])<=1.0f)&&(maths_f_abs(rel_pos[Y])<=1.0f))||((maths_f_abs(rel_pos[X])<=5.0f)&&(maths_f_abs(rel_pos[Y])<=5.0f)&&(maths_f_abs(rel_pos[Z])>=3.0f)))
	{
		rel_heading = 0.0f;
	}
	else
	{
		rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - navigation->position_estimator->local_position.heading);
	}

	// calculate dir_desired in local frame
	// vel = qe-1 * rel_pos * qe
	qtmp1 = quaternions_create_from_vector(rel_pos);
	qtmp2 = quaternions_global_to_local(*navigation->qe,qtmp1);
	dir_desired_bf[0] = qtmp2.v[0]; dir_desired_bf[1] = qtmp2.v[1]; dir_desired_bf[2] = qtmp2.v[2];
	
	v_desired = 0.0; //?
	
	dir_desired_bf[X] = 0.0; //?
	dir_desired_bf[Y] = 0.0; //?
	dir_desired_bf[Z] = 0.0; //?
	
	navigation->controls_nav->tvel[X] = dir_desired_bf[X];
	navigation->controls_nav->tvel[Y] = dir_desired_bf[Y];
	navigation->controls_nav->tvel[Z] = dir_desired_bf[Z];
	navigation->controls_nav->rpy[YAW] = KP_YAW * rel_heading;
}

static void navigation_run(local_coordinates_t waypoint_input, navigation_t* navigation)
{
	float rel_pos[3];
	
	// Control in translational speed of the platform
	navigation->waypoint_handler->dist2wp_sqr = navigation_set_rel_pos_n_dist2wp(waypoint_input.pos,
																					rel_pos,
																					navigation->position_estimator->local_position.pos);
	navigation_set_speed_command(rel_pos, navigation);
	
	navigation->controls_nav->theading=waypoint_input.heading;
}

static void navigation_set_auto_takeoff(navigation_t *navigation, mavlink_command_long_t* packet)
{
	print_util_dbg_print("Starting automatic take-off from button\n");
	navigation->auto_takeoff = true;

	mavlink_message_t msg;
	mavlink_msg_command_ack_pack( 	navigation->mavlink_stream->sysid,
									navigation->mavlink_stream->compid,
									&msg,
									MAV_CMD_NAV_TAKEOFF,
									MAV_RESULT_ACCEPTED);
	
	mavlink_stream_send(navigation->mavlink_stream, &msg);
}

static bool navigation_mode_change(navigation_t* navigation)
{
	mav_mode_t mode_local = navigation->state->mav_mode;
	mav_mode_t mode_nav = navigation->mode;
	
	bool result = false;
	
	if ((mode_local.STABILISE == mode_nav.STABILISE)&&(mode_local.GUIDED == mode_nav.GUIDED)&&(mode_local.AUTO == mode_nav.AUTO))
	{
		result = true;
	}
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void navigation_init(navigation_t* navigation, control_command_t* controls_nav, const quat_t* qe, mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator, state_t* state, const control_command_t* control_joystick, const remote_t* remote, mavlink_communication_t* mavlink_communication)
{
	
	navigation->controls_nav = controls_nav;
	navigation->qe = qe;
	navigation->waypoint_handler = waypoint_handler;
	navigation->position_estimator = position_estimator;
	navigation->state = state;
	navigation->mavlink_stream = &mavlink_communication->mavlink_stream;
	navigation->control_joystick = control_joystick;
	navigation->remote = remote;
	
	navigation->controls_nav->rpy[ROLL] = 0.0f;
	navigation->controls_nav->rpy[PITCH] = 0.0f;
	navigation->controls_nav->rpy[YAW] = 0.0f;
	navigation->controls_nav->tvel[X] = 0.0f;
	navigation->controls_nav->tvel[Y] = 0.0f;
	navigation->controls_nav->tvel[Z] = 0.0f;
	navigation->controls_nav->theading = 0.0f;
	navigation->controls_nav->thrust = -1.0f;
	navigation->controls_nav->control_mode = VELOCITY_COMMAND_MODE;
	navigation->controls_nav->yaw_mode = YAW_ABSOLUTE;
	
	navigation->mode.byte = state->mav_mode.byte;
	
	navigation->auto_takeoff = false;
	
	navigation->controls_nav->mavlink_stream = &mavlink_communication->mavlink_stream;
	
	navigation->dist2vel_gain = 0.7f;
	navigation->cruise_speed = 3.0f;
	navigation->max_climb_rate = 1.0f;
	
	navigation->soft_zone_size = 0.0f;
	
	navigation->loop_count = 0;
	
	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_NAV_TAKEOFF; // 22
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&navigation_set_auto_takeoff;
	callbackcmd.module_struct =									navigation;
	mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	print_util_dbg_print("Navigation initialized.\r\n");
}

task_return_t navigation_update(navigation_t* navigation)
{
	mav_mode_t mode_local = navigation->state->mav_mode;
	
	float thrust;
	
	switch (navigation->state->mav_state)
	{
		case MAV_STATE_ACTIVE:
			if (navigation->state->in_the_air)
			{
				if(mode_local.AUTO == AUTO_ON)
				{
					navigation_waypoint_navigation_handler(navigation);
						
					if (navigation->state->nav_plan_active)
					{
						navigation_run(navigation->waypoint_handler->waypoint_coordinates,navigation);
					}
					else
					{
						navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
					}
				}
				else if(mode_local.GUIDED == GUIDED_ON)
				{
					navigation_hold_position_handler(navigation);
						
					navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
					break;
				}
			}
			else
			{
				if (navigation->state->remote_active == 1)
				{
					thrust = remote_get_throttle(navigation->remote);
				}
				else
				{
					thrust = navigation->control_joystick->thrust;
				}
				
				if (thrust > -0.7f)
				{
					if ((mode_local.GUIDED == GUIDED_ON)||(mode_local.AUTO == AUTO_ON))
					{
						if (!navigation->auto_takeoff)
						{
							navigation->waypoint_handler->hold_waypoint_set = false;
						}
						navigation->auto_takeoff = true;
					}
					else
					{
						navigation->state->in_the_air = true;
					}
				}
				
				if ((mode_local.GUIDED == GUIDED_ON)||(mode_local.AUTO == AUTO_ON))
				{
					if (navigation->auto_takeoff)
					{
						navigation_waypoint_take_off_handler(navigation);
						
						navigation_run(navigation->waypoint_handler->waypoint_hold_coordinates,navigation);
					}
				}
			}
			break;

		case MAV_STATE_CRITICAL:
			// In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
			if (mode_local.STABILISE == STABILISE_ON)
			{
				navigation_critical_handler(navigation->waypoint_handler);
				navigation_run(navigation->waypoint_handler->waypoint_critical_coordinates,navigation);
			}
			break;
			
		default:
			break;
	}
	
	navigation->mode = mode_local;
	
	return TASK_RUN_SUCCESS;
}

void navigation_waypoint_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_coordinates_t local_pos)
{
	waypoint_handler->hold_waypoint_set = true;
	
	waypoint_handler->waypoint_hold_coordinates = local_pos;
	
	//waypoint_handler->waypoint_hold_coordinates.heading = coord_conventions_get_yaw(waypoint_handler->ahrs->qe);
	//waypoint_handler->waypoint_hold_coordinates.heading = local_pos.heading;
	
	print_util_dbg_print("Position hold at: (");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[X],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Y],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Z],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num((int32_t)(waypoint_handler->waypoint_hold_coordinates.heading*180.0f/3.14f),10);
	print_util_dbg_print(")\r\n");
	
}

void navigation_waypoint_take_off_init(mavlink_waypoint_handler_t* waypoint_handler)
{
	print_util_dbg_print("Automatic take-off, will hold position at: (");
	print_util_dbg_print_num(waypoint_handler->position_estimator->local_position.pos[X],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_handler->position_estimator->local_position.pos[Y],10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(-10.0f,10);
	print_util_dbg_print("), with heading of: ");
	print_util_dbg_print_num((int32_t)(waypoint_handler->position_estimator->local_position.heading*180.0f/3.14f),10);
	print_util_dbg_print("\r\n");

	waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimator->local_position;
	waypoint_handler->waypoint_hold_coordinates.pos[Z] = -10.0f;
	
	aero_attitude_t aero_attitude;
	aero_attitude=coord_conventions_quat_to_aero(waypoint_handler->ahrs->qe);
	waypoint_handler->waypoint_hold_coordinates.heading = aero_attitude.rpy[2];
	
	waypoint_handler->dist2wp_sqr = 100.0f; // same position, 10m above => dist_sqr = 100.0f
	
	waypoint_handler->hold_waypoint_set = true;
}

void navigation_waypoint_take_off_handler(navigation_t* navigation)
{
	if (!navigation->waypoint_handler->hold_waypoint_set)
	{
		navigation_waypoint_take_off_init(navigation->waypoint_handler);
	}
	if (!navigation->state->nav_plan_active)
	{
		waypoint_handler_nav_plan_init(navigation->waypoint_handler);
	}
	
	//if (navigation->mode == navigation->state->mav_mode.byte)
	if (navigation_mode_change(navigation))
	{
		if (navigation->waypoint_handler->dist2wp_sqr <= 16.0f)
		{
			//state_machine->state->mav_state = MAV_STATE_ACTIVE;
			navigation->state->in_the_air = true;
			navigation->auto_takeoff = false;
			print_util_dbg_print("Automatic take-off finised, dist2wp_sqr (10x):");
			print_util_dbg_print_num(navigation->waypoint_handler->dist2wp_sqr * 10.0f,10);
			print_util_dbg_print(".\r\n");
		}
	}
}


void navigation_hold_position_handler(navigation_t* navigation)
{
	//if (navigation->mode != navigation->state->mav_mode.byte)
	if (!navigation_mode_change(navigation))
	{
		navigation->waypoint_handler->hold_waypoint_set = false;
	}
	
	if (!navigation->state->nav_plan_active)
	{
		waypoint_handler_nav_plan_init(navigation->waypoint_handler);
	}
	
	if (!navigation->waypoint_handler->hold_waypoint_set)
	{
		navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->waypoint_handler->position_estimator->local_position);
	}
}

void navigation_waypoint_navigation_handler(navigation_t* navigation)
{
	//if (navigation->mode != navigation->state->mav_mode.byte)
	if (!navigation_mode_change(navigation))
	{
		navigation->waypoint_handler->hold_waypoint_set = false;
	}
	
	if (navigation->state->nav_plan_active)
	{
		uint8_t i;
		float rel_pos[3];
		
		for (i=0;i<3;i++)
		{
			rel_pos[i] = navigation->waypoint_handler->waypoint_coordinates.pos[i]-navigation->waypoint_handler->position_estimator->local_position.pos[i];
		}
		navigation->waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
		
		if (navigation->waypoint_handler->dist2wp_sqr < (navigation->waypoint_handler->current_waypoint.param2*navigation->waypoint_handler->current_waypoint.param2))
		{
			print_util_dbg_print("Waypoint Nr");
			print_util_dbg_print_num(navigation->waypoint_handler->current_waypoint_count,10);
			print_util_dbg_print(" reached, distance:");
			print_util_dbg_print_num(sqrt(navigation->waypoint_handler->dist2wp_sqr),10);
			print_util_dbg_print(" less than :");
			print_util_dbg_print_num(navigation->waypoint_handler->current_waypoint.param2,10);
			print_util_dbg_print(".\r\n");
			
			mavlink_message_t msg;
			mavlink_msg_mission_item_reached_pack( 	navigation->mavlink_stream->sysid,
													navigation->mavlink_stream->compid,
													&msg,
													navigation->waypoint_handler->current_waypoint_count);
			mavlink_stream_send(navigation->mavlink_stream, &msg);
			
			navigation->waypoint_handler->waypoint_list[navigation->waypoint_handler->current_waypoint_count].current = 0;
			if((navigation->waypoint_handler->current_waypoint.autocontinue == 1)&&(navigation->waypoint_handler->number_of_waypoints>1))
			{
				print_util_dbg_print("Autocontinue towards waypoint Nr");
				
				if (navigation->waypoint_handler->current_waypoint_count == (navigation->waypoint_handler->number_of_waypoints-1))
				{
					navigation->waypoint_handler->current_waypoint_count = 0;
				}
				else
				{
					navigation->waypoint_handler->current_waypoint_count++;
				}
				print_util_dbg_print_num(navigation->waypoint_handler->current_waypoint_count,10);
				print_util_dbg_print("\r\n");
				navigation->waypoint_handler->waypoint_list[navigation->waypoint_handler->current_waypoint_count].current = 1;
				navigation->waypoint_handler->current_waypoint = navigation->waypoint_handler->waypoint_list[navigation->waypoint_handler->current_waypoint_count];
				navigation->waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(navigation->waypoint_handler, navigation->waypoint_handler->position_estimator->local_position.origin);
				
				mavlink_message_t msg;
				mavlink_msg_mission_current_pack( 	navigation->mavlink_stream->sysid,
													navigation->mavlink_stream->compid,
													&msg,
													navigation->waypoint_handler->current_waypoint_count);
				mavlink_stream_send(navigation->mavlink_stream, &msg);
				
			}
			else
			{
				navigation->state->nav_plan_active = false;
				print_util_dbg_print("Stop\r\n");
				
				navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->waypoint_handler->waypoint_coordinates);
			}
		}
	}
	else
	{
		if (!navigation->waypoint_handler->hold_waypoint_set)
		{
			navigation_waypoint_hold_init(navigation->waypoint_handler, navigation->waypoint_handler->position_estimator->local_position);
		}
		waypoint_handler_nav_plan_init(navigation->waypoint_handler);
	}
}

void navigation_critical_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
	float rel_pos[3];
	uint8_t i;
	
	if (waypoint_handler->state->mav_state == MAV_STATE_STANDBY)
	{
		waypoint_handler->critical_behavior = CLIMB_TO_SAFE_ALT;
	}
	
	if (!(waypoint_handler->critical_next_state))
	{
		waypoint_handler->critical_next_state = true;
		
		aero_attitude_t aero_attitude;
		aero_attitude=coord_conventions_quat_to_aero(waypoint_handler->ahrs->qe);
		waypoint_handler->waypoint_critical_coordinates.heading = aero_attitude.rpy[2];
		
		switch (waypoint_handler->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
				waypoint_handler->waypoint_critical_coordinates.pos[X] = waypoint_handler->position_estimator->local_position.pos[X];
				waypoint_handler->waypoint_critical_coordinates.pos[Y] = waypoint_handler->position_estimator->local_position.pos[Y];
				waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
				break;
			
			case FLY_TO_HOME_WP:
				waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
				waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
				waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
				break;
			
			case CRITICAL_LAND:
				waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
				waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
				waypoint_handler->waypoint_critical_coordinates.pos[Z] = 0.0f;
				break;
		}
		
		for (i=0;i<3;i++)
		{
			rel_pos[i] = waypoint_handler->waypoint_critical_coordinates.pos[i] - waypoint_handler->position_estimator->local_position.pos[i];
		}
		waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
	}
	
	if (waypoint_handler->dist2wp_sqr < 3.0f)
	{
		waypoint_handler->critical_next_state = false;
		switch (waypoint_handler->critical_behavior)
		{
			case CLIMB_TO_SAFE_ALT:
				print_util_dbg_print("Critical State! Flying to home waypoint.\r\n");
				waypoint_handler->critical_behavior = FLY_TO_HOME_WP;
				break;
			
			case FLY_TO_HOME_WP:
				print_util_dbg_print("Critical State! Performing critical landing.\r\n");
				waypoint_handler->critical_behavior = CRITICAL_LAND;
				break;
			
			case CRITICAL_LAND:
				print_util_dbg_print("Critical State! Landed, switching off motors, Emergency mode.\r\n");
				waypoint_handler->critical_landing = true;
				break;
		}
	}
}