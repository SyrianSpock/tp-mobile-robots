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
 * \file sonar_i2cxl.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Driver for the sonar module using i2C communication protocol
 *
 ******************************************************************************/


#include "sonar_i2cxl.h"
#include "twim.h"
#include "delay.h"
#include "print_util.h"
#include "mavlink_communication.h"
#include "time_keeper.h"

const uint8_t SONAR_I2CXL_DEFAULT_ADDRESS			= 0x70;		///< Address of the device
const uint8_t SONAR_I2CXL_RANGE_COMMAND				= 0x51;		///< Address of the Range Command Register
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register 1
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register 2


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Send range Command for the sonar_i2cxl
 *
 * \param sonar pointer to an object containing the sonar_i2cxl's data
 */
void sonar_i2cxl_send_range_command(sonar_i2cxl_t* sonar);


/**
 * \brief Get the last measurement
 *
 * \param sonar pointer to an object containing the sonar_i2cxl's data
 */
void sonar_i2cxl_get_last_measure(sonar_i2cxl_t* sonar);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void sonar_i2cxl_send_range_command(sonar_i2cxl_t* sonar_i2cxl)
{
	uint8_t buff = SONAR_I2CXL_RANGE_COMMAND;
	twim_write(&AVR32_TWIM1, &buff, 1, sonar_i2cxl->i2c_address, false);
}


void sonar_i2cxl_get_last_measure(sonar_i2cxl_t* sonar_i2cxl)
{
	uint8_t buf[2];
	twim_read(&AVR32_TWIM1, buf, 2, sonar_i2cxl->i2c_address, false);
	sonar_i2cxl->distance_cm = (buf[0] << 8) + buf[1];
	sonar_i2cxl->distance_m  = ((float)sonar_i2cxl->distance_cm) / 100;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void sonar_i2cxl_init(sonar_i2cxl_t* sonar_i2cxl, const mavlink_stream_t* mavlink_stream)
{
	// Init dependencies
	sonar_i2cxl->mavlink_stream = mavlink_stream;

	///< Init data_struct
	sonar_i2cxl->i2c_address = SONAR_I2CXL_DEFAULT_ADDRESS;
	sonar_i2cxl->distance_cm = 0;
	sonar_i2cxl->distance_m  = 0;

	///< Init I2C bus
	static twi_options_t twi_opt = 
	{
		.pba_hz = 64000000,
		.speed  = 100000,
		.chip   = 1,
		.smbus  = false
	};

	twi_master_init(&AVR32_TWIM1, &twi_opt);
	print_util_dbg_print("i2cxl Sonar initialized\r\n");
}


void sonar_i2cxl_update(sonar_i2cxl_t* sonar_i2cxl)
{
	sonar_i2cxl_get_last_measure(sonar_i2cxl);
	sonar_i2cxl_send_range_command(sonar_i2cxl);
}


task_return_t sonar_i2cxl_send_telemetry(sonar_i2cxl_t* sonar_i2cxl)
{
	mavlink_message_t msg;

	mavlink_msg_distance_sensor_pack(	sonar_i2cxl->mavlink_stream->sysid,
										sonar_i2cxl->mavlink_stream->compid,
										&msg,
						       			time_keeper_get_millis(), 
						       			20,								// min 20cm 
						       			760,							// max 7.6m 
						       			sonar_i2cxl->distance_m * 100, 
						       			MAV_DISTANCE_SENSOR_ULTRASOUND, 
						       			0, 								// id 0
						       			0, 								// orientation 0
						       			1);								// covariance (!=0)

	mavlink_stream_send(sonar_i2cxl->mavlink_stream, &msg);
	return TASK_RUN_SUCCESS;
}