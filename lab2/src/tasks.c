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
 * \file tasks.c
 *
 * \author MAV'RIC Team
 *   
 * \brief Definition of the tasks executed on the autopilot
 *
 ******************************************************************************/


#include "tasks.h"
#include "central_data.h"
#include "print_util.h"
#include "stabilisation.h"
#include "gps_ublox.h"
#include "navigation.h"
#include "led.h"
#include "imu.h"
#include "delay.h"
#include "analog_monitor.h"
#include "lsm330dlc.h"
#include "hmc5883l.h"
#include "stdio_usb.h"
#include "data_logging.h"

#include "pwm_servos.h"

#include "remote.h"
#include "attitude_controller_p2.h"

central_data_t* central_data;


task_set_t* tasks_get_main_taskset() 
{
	central_data = central_data_get_pointer_to_struct();

	return central_data->scheduler.task_set;
}

task_return_t tasks_run_imu_update(void* arg)
{
	
	lsm330dlc_gyro_update(&(central_data->imu.raw_gyro));
	lsm330dlc_acc_update(&(central_data->imu.raw_accelero));
	hmc5883l_update(&(central_data->imu.raw_compass));
	
	imu_update(	&central_data->imu);
	qfilter_update(&central_data->attitude_filter);
	
	return TASK_RUN_SUCCESS;
}



task_return_t tasks_led_toggle(void* arg)
{
	LED_Toggle(LED1);
	
	return TASK_RUN_SUCCESS;
}


void tasks_create_tasks() 
{	
	central_data = central_data_get_pointer_to_struct();
	
	scheduler_t* scheduler = &central_data->scheduler;

	scheduler_add_task(scheduler, 4000,		RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGHEST, &tasks_run_imu_update												, 0															, 0);
	scheduler_add_task(scheduler, 20000, 	RUN_REGULAR, PERIODIC_RELATIVE, PRIORITY_HIGH   , (task_function_t)&remote_update 									, (task_argument_t)&central_data->remote 				, 1);
	scheduler_add_task(scheduler, 200000,   RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&state_machine_update              				, (task_argument_t)&central_data->state_machine         , 5);
	scheduler_add_task(scheduler, 4000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&mavlink_communication_update                    , (task_argument_t)&central_data->mavlink_communication , 6);
	scheduler_add_task(scheduler, 250000,	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOWEST , &tasks_led_toggle													, 0														, 10);

	scheduler_sort_tasks(scheduler);
}

