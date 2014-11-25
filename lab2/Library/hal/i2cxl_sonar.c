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
 * \file i2cxl_sonar.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Driver for the sonar module using i2C communication protocol
 *
 ******************************************************************************/


#include "i2cxl_sonar.h"
#include "twim.h"
#include "delay.h"
#include "print_util.h"
#include "mavlink_communication.h"
#include "time_keeper.h"

const uint8_t I2CXL_DEFAULT_ADDRESS				= 0x70;		///< Address of the device
const uint8_t I2CXL_RANGE_COMMAND				= 0x51;		///< Address of the Range Command Register
const uint8_t I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register 1
const uint8_t I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register 2

/**
 * \brief Send range Command for the i2cxl_sonar
 *
 * \param i2c_sonar pointer to an object containing the i2cxl_sonar's data
 */
void i2cxl_send_range_command(i2cxl_sonar_t* i2c_sonar);

/**
 * \brief Get the last measurement
 *
 * \param i2c_sonar pointer to an object containing the i2cxl_sonar's data
 */
void i2cxl_get_last_measure(i2cxl_sonar_t* i2c_sonar);


void i2cxl_sonar_init(i2cxl_sonar_t* i2cxl_sonar, const mavlink_stream_t* mavlink_stream)
{
	// Init dependencies
	i2cxl_sonar->mavlink_stream = mavlink_stream;

	///< Init data_struct
	i2cxl_sonar->i2c_address = I2CXL_DEFAULT_ADDRESS;
	i2cxl_sonar->distance_cm = 0;
	i2cxl_sonar->distance_m  = 0;

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


void i2cxl_sonar_update(i2cxl_sonar_t* i2cxl_sonar)
{
	i2cxl_get_last_measure(i2cxl_sonar);
	i2cxl_send_range_command(i2cxl_sonar);
}

void i2cxl_send_range_command(i2cxl_sonar_t* i2cxl_sonar)
{
	uint8_t buff = I2CXL_RANGE_COMMAND;
	twim_write(&AVR32_TWIM1, &buff, 1, i2cxl_sonar->i2c_address, false);
}


void i2cxl_get_last_measure(i2cxl_sonar_t* i2cxl_sonar)
{
	uint8_t buf[2];
	twim_read(&AVR32_TWIM1, buf, 2, i2cxl_sonar->i2c_address, false);
	i2cxl_sonar->distance_cm = (buf[0] << 8) + buf[1];
	i2cxl_sonar->distance_m  = ((float)i2cxl_sonar->distance_cm) / 100;
}

task_return_t i2cxl_send_sonar(i2cxl_sonar_t* i2cxl_sonar)
{
	mavlink_message_t msg;
	mavlink_msg_named_value_float_pack(	i2cxl_sonar->mavlink_stream->sysid,
										i2cxl_sonar->mavlink_stream->compid,
										&msg,
										time_keeper_get_millis(),
										"sonar(m)",
										i2cxl_sonar->distance_m);
	mavlink_stream_send(i2cxl_sonar->mavlink_stream, &msg);
	return TASK_RUN_SUCCESS;
}