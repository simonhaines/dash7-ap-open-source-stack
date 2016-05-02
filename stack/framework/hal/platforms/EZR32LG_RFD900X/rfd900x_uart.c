/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2015 University of Antwerp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*! \file rfd900x_uart.c
 *
 *  Basic, non-DMA-driven UART controller based on the existing HAL.
 *
 *  \author simon@rallysafe.com.au
 */

#include <debug.h>
#include "hwuart.h"
#include "hwatomic.h"
#include "scheduler.h"
#include "platform_uart.h"


#define NUM_UART_BUFFERS	3
#define UART_BUFFER_SIZE	64
#define UART_QUEUE_SIZE		3

// Buffers allocated for asynchronously receiving messages over the UART
static uint8_t buffers[NUM_UART_BUFFERS][UART_BUFFER_SIZE + 1];
static size_t current_buffer;

// Callbacks for messages received over the UART
static uart_callback_t callbacks[UART_QUEUE_SIZE];
static size_t num_registered_callbacks;

static uart_handle_t *uart;
static void uart_rx_inthandler(uint8_t byte);
static void uart_task(void);

__LINK_C void __uart_init(void) {
	num_registered_callbacks = 0;

	uart = uart_init(UART_NUM, UART_BAUDRATE, UART_LOCATION);
	uart_set_rx_interrupt_callback(uart, uart_rx_inthandler);
	assert(uart_enable(uart));
}

__LINK_C error_t uart_register_callback(uart_callback_t callback)
{
	if (callback == 0x0)
		return EINVAL;

	uint8_t empty_index = UART_QUEUE_SIZE;
	for(int i = 0; i < UART_QUEUE_SIZE; i++)
	{
		if(empty_index == UART_QUEUE_SIZE && callbacks[i] == 0x0)
			empty_index = i;
		else if(callbacks[i] == callback)
			return EALREADY;
	}

	if(empty_index >= UART_QUEUE_SIZE)
		return ENOMEM;

	start_atomic();
		callbacks[empty_index] = callback;
		num_registered_callbacks++;
		if(num_registered_callbacks == 1)
		{
			// This is the first listener to register, enable the UART interrupt
			uart_rx_interrupt_enable(uart);
		}
	end_atomic();
	return SUCCESS;
}

__LINK_C error_t uart_deregister_callback(uart_callback_t callback)
{
	if (callback == 0x0)
		return EINVAL;

	uint8_t callback_index = UART_QUEUE_SIZE;
	for(int i = 0; i < UART_QUEUE_SIZE; i++)
	{
		if(callbacks[i] == callback)
		{
			callback_index = i;
			break;
		}
	}

	if(callback_index >= UART_QUEUE_SIZE)
		return EALREADY;

	start_atomic();
		callbacks[callback_index] = 0x0;
		num_registered_callbacks--;
		if(num_registered_callbacks == 0)
		{
			// This is the last listener to deregister, disable the UART interrupt
			uart_rx_interrupt_disable(uart);
		}
	end_atomic();
	return SUCCESS;
}

static void uart_rx_inthandler(uint8_t byte) {
	static enum {
		SYNC = 0x7E,
		LENGTH,
		PAYLOAD,
		CHECKSUM1,
		CHECKSUM2,
		FLUSH
	} state = SYNC;
	static size_t index;
	static uint8_t length;
	static uint8_t chkA, chkB;

	switch(state)
	{
	case SYNC:
		state = (byte == SYNC) ? LENGTH : SYNC;
		break;
	case LENGTH:
		length = byte;
		if (length < UART_BUFFER_SIZE)
		{
			if (++current_buffer == NUM_UART_BUFFERS)
				current_buffer = 0;
			index = chkA = chkB = 0;
			buffers[current_buffer][index++] = length;
			state = PAYLOAD;
		}
		else
			state = FLUSH;
		break;
	case PAYLOAD:
		buffers[current_buffer][index++] = byte;
		chkA += byte;
		chkB += chkA;
		if (index == length)
			state = CHECKSUM1;
		break;
	case CHECKSUM1:
		if (chkB != byte)
		{
			length = 1;
			state = FLUSH;
		}
		else
			state = CHECKSUM2;
		break;
	case CHECKSUM2:
		if (chkA == byte)
			sched_post_task(&uart_task);
		state = SYNC;
		break;
	case FLUSH:
		if (--length == 0)
			state = SYNC;
		break;
	}
}

static void uart_task()
{
	uint8_t *buffer = &buffers[current_buffer][1];
	size_t length = buffers[current_buffer][0];

	for (int index = 0; index < UART_QUEUE_SIZE; ++index)
	{
		uart_callback_t callback = 0x0;
		start_atomic();
			if (callbacks[index] != 0x0)
			{
				callback = callbacks[index];
				break;
			}
		end_atomic();

		if (callback != 0x0)
			callback(buffer, length);
	}
}
