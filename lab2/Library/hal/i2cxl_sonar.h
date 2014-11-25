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
 * \file i2cxl_sonar.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Driver for the sonar module using i2C communication protocol
 *
 ******************************************************************************/


#ifndef I2CXL_SONAR_H_
#define I2CXL_SONAR_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "scheduler.h"
#include "mavlink_stream.h"
#include <stdint.h>

/**
 * \brief structure of the i2cxl_sonar module
*/
typedef struct 
{
	uint8_t i2c_address;		///< address of the sonar module
	uint16_t distance_cm;		///< measured distance in centimeters
	float distance_m;			///< measured distance in meters
	const mavlink_stream_t* mavlink_stream;  ///< Pointer to mavlink stream
} i2cxl_sonar_t;

/**
 * \brief Initializes the I2CXL sonar data struct and the i2c bus
 * 
 * \param i2c_sonar pointer to the i2c_sonar Data structure
 */
void i2cxl_sonar_init(i2cxl_sonar_t* i2c_sonar, const mavlink_stream_t* mavlink_stream);

/**
 * \brief Reads last value from sensor and start new recording
 * \details This function should be called at a frequency lower than 10Hz
 * 
 * \param i2c_sonar Data struct
 */
void i2cxl_sonar_update(i2cxl_sonar_t* i2c_sonar);


/**
 * \brief	Task to send the mavlink sonar message
 * 
 * \param i2c_sonar Data struct
 *
 * \return	The status of execution of the task
 */
task_return_t i2cxl_send_sonar(i2cxl_sonar_t* i2cxl_sonar);

#ifdef __cplusplus
	}
#endif

#endif /* I2CXL_SONAR_H */