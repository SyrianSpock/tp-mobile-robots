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
 * \file boardsupport.c
 * 
 * \author MAV'RIC Team
 *   
 * \brief Initialization of all hardware related elements (communication lines, 
 * sensors devices, etc)
 *
 ******************************************************************************/


#include "boardsupport.h"
#include "uart_int.h"
#include "sysclk.h"
#include "sleepmgr.h"
#include "led.h"
#include "delay.h"
#include "user_board/user_board.h"
#include "buffer.h"

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "print_util.h"
#include "lsm330dlc.h"
#include "hmc5883l.h"
#include "piezo_speaker.h"
#include "gpio.h"

#include "xbee.h"
#include "console.h"
#include "stdio_usb.h"

void boardsupport_init(central_data_t *central_data) 
{
	irq_initialize_vectors();
	cpu_irq_enable();
	Disable_global_interrupt();
		
	// Initialize the sleep manager
	sleepmgr_init();
	sysclk_init();

	board_init();
	delay_init(sysclk_get_cpu_hz());
	time_keeper_init();
		
	INTC_init_interrupts();

	// Switch on the red LED
	LED_On(LED2);

	// Init UART 0 for XBEE communication
	xbee_init(UART0);
				
	// Init USB for wired communication
	console_init(CONSOLE_USB);
		
	// connect abstracted aliases to hardware ports
	central_data->telemetry_down_stream = xbee_get_out_stream();
	central_data->telemetry_up_stream = xbee_get_in_stream();
	central_data->debug_out_stream = console_get_out_stream();
	central_data->debug_in_stream = console_get_in_stream();
	
	// init debug output
	print_util_dbg_print_init(central_data->debug_out_stream);
	print_util_dbg_print("Debug stream initialised\r\n");
	
	// init 6V enable
	gpio_enable_gpio_pin(AVR32_PIN_PA04);
	gpio_set_gpio_pin(AVR32_PIN_PA04);
	
	Enable_global_interrupt();

	// Init piezo speaker
	piezo_speaker_init_binary();
	
	print_util_dbg_print("Board initialised\r\n");
}
