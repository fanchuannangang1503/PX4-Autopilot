/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david.sidrane@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include "boot_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include <hardware/s32k1xx_flexcan.h>
#include "nvic.h"

#include "board.h"
#include "px4_macros.h"
#include "can.h"
#include "timer.h"

#include <arch/board/board.h>

#include <lib/systemlib/crc.h>


#define CAN_TX_TIMEOUT_MS     (200 /(1000/(1000000/CONFIG_USEC_PER_TICK)))

/****************************************************************************
 * Name: can_speed2freq
 *
 * Description:
 *   This function maps a can_speed_t to a bit rate in Hz
 *
 * Input Parameters:
 *   can_speed - A can_speed_t from CAN_125KBAUD to CAN_1MBAUD
 *
 * Returned value:
 *   Bit rate in Hz
 *
 ****************************************************************************/
int can_speed2freq(can_speed_t speed)

{
	return 1000000 >> (CAN_1MBAUD - speed);
}

/****************************************************************************
 * Name: can_speed2freq
 *
 * Description:
 *   This function maps a frequency in Hz to a can_speed_t in the range
 *   CAN_125KBAUD to CAN_1MBAUD.
 *
 * Input Parameters:
 *   freq - Bit rate in Hz
 *
 * Returned value:
 *   A can_speed_t from CAN_125KBAUD to CAN_1MBAUD
 *
 ****************************************************************************/
can_speed_t can_freq2speed(int freq)
{
	return (freq == 1000000u ? CAN_1MBAUD : freq == 500000u ? CAN_500KBAUD : freq == 250000u ? CAN_250KBAUD : CAN_125KBAUD);
}

/****************************************************************************
 * Name: can_tx
 *
 * Description:
 *   This function is called to transmit a CAN frame using the supplied
 *   mailbox. It will busy wait on the mailbox if not available.
 *
 * Input Parameters:
 *   message_id - The CAN message's EXID field
 *   length     - The number of bytes of data - the DLC field
 *   message    - A pointer to 8 bytes of data to be sent (all 8 bytes will be
 *                loaded into the CAN transmitter but only length bytes will
 *                be sent.
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   The CAN_OK of the data sent or CAN_ERROR if a time out occurred
 *
 ****************************************************************************/
uint8_t can_tx(uint32_t message_id, size_t length, const uint8_t *message, uint8_t mailbox)
{
	return CAN_OK;
}

/****************************************************************************
 * Name: can_rx
 *
 * Description:
 *   This function is called to receive a CAN frame from a supplied fifo.
 *   It does not block if there is not available, but returns 0
 *
 * Input Parameters:
 *   message_id - A pointer to return the CAN message's EXID field
 *   length     - A pointer to return the number of bytes of data - the DLC field
 *   message    - A pointer to return 8 bytes of data to be sent (all 8 bytes will
 *                be written from the CAN receiver but only length bytes will be sent.
 *   fifo         A can_fifo_mailbox_t fifixxx value to choose the incoming fifo.
 *
 * Returned value:
 *   The length of the data read or 0 if the fifo was empty
 *
 ****************************************************************************/
uint8_t can_rx(uint32_t *message_id, size_t *length, uint8_t *message, uint8_t fifo)
{
	return 0;
}

/****************************************************************************
 * Name: can_autobaud
 *
 * Description:
 *   This function will attempt to detect the bit rate in use on the CAN
 *   interface until the timeout provided expires or the successful detection
 *   occurs.
 *
 *   It will initialize the CAN block for a given bit rate
 *   to test that a message can be received. The CAN interface is left
 *   operating at the detected bit rate and in CAN_Mode_Normal mode.
 *
 * Input Parameters:
 *   can_speed - A pointer to return detected can_speed_t from CAN_UNKNOWN to
 *               CAN_1MBAUD
 *   timeout   - The timer id of a timer to use as the maximum time to wait for
 *               successful bit rate detection. This timer may be not running
 *               in which case the auto baud code will try indefinitely to
 *               detect the bit rate.
 *
 * Returned value:
 *   CAN_OK - on Success or a CAN_BOOT_TIMEOUT
 *
 ****************************************************************************/
int can_autobaud(can_speed_t *can_speed, bl_timer_id timeout)
{
	*can_speed = CAN_UNKNOWN;


	return CAN_OK;
}

/****************************************************************************
 * Name: can_init
 *
 * Description:
 *   This function is used to initialize the CAN block for a given bit rate and
 *   mode.
 *
 * Input Parameters:
 *   speed - A can_speed_t from CAN_125KBAUD to CAN_1MBAUD
 *   mode  - One of the can_mode_t of Normal, LoopBack and Silent or
 *           combination thereof.
 *
 * Returned value:
 *   OK - on Success or a negate errno value
 *
 ****************************************************************************/
int can_init(can_speed_t speed, can_mode_t mode)
{

	return OK;
}

/****************************************************************************
 * Name: can_cancel_on_error
 *
 * Description:
 *   This function will test for transition completion or any error.
 *   If the is a error the the transmit will be aborted.
 *
 * Input Parameters:
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   CAN_OK - on Success or a CAN_ERROR if the cancellation was needed
 *
 ****************************************************************************/
void can_cancel_on_error(uint8_t mailbox)
{

}
