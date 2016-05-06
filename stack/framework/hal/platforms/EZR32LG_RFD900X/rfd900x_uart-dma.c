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
 *  DMA-driven UART controller.
 *
 *  \author simon@rallysafe.com.au
 */


#include <debug.h>
#include "types.h"
#include "scheduler.h"
#include "platform.h"
#include "platform_uart.h"

#include "em_cmu.h"
#include "em_dma.h"
#include "dmactrl.h"

#include "hwuart.h"
#include "hwatomic.h"


#define NUM_UART_BUFFERS	3
#define UART_BUFFER_SIZE	64
#define UART_QUEUE_SIZE		3

#define DMA_CHANNELS    2
#define DMA_CHANNEL_RX  0
#define DMA_CHANNEL_TX  1

// Buffers allocated for asynchronously receiving messages over the UART
static struct {
	uint8_t data[UART_BUFFER_SIZE + 2];
	size_t length;
} buffers[NUM_UART_BUFFERS];
static size_t current_buffer = 0;

// Transmit buffers
static uint8_t tx_buffer[2][UART_BUFFER_SIZE];
static size_t current_tx_buffer = 0;


// Callbacks for messages received over the UART
static uart_callback_t callbacks[UART_QUEUE_SIZE];
static size_t num_registered_callbacks = 0;

static uart_handle_t *uart;
static void uart_rx_inthandler(uint8_t byte);
static void uart_task(void);



static DMA_CB_TypeDef dma_callbacks[DMA_CHANNELS];
static uint8_t data;
static void dma_rx_complete(unsigned int channel, bool primary, void *user) {
	static enum {
		SYNC = 0x7E,
		LENGTH,
		PAYLOAD,
		CHECKSUM1,
		CHECKSUM2
	} state = SYNC;
	static size_t index;
	static uint8_t length;
	static uint8_t chkA, chkB;

	switch(state)
	{
	case SYNC:
		state = (data == SYNC) ? LENGTH : SYNC;
		DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, &data, (void *)&(USART1->RXDATA), 0);
		break;
	case LENGTH:
		length = data;
		if (length <= UART_BUFFER_SIZE)
		{
			if (++current_buffer == NUM_UART_BUFFERS)
				current_buffer = 0;
			index = chkA = chkB = 0;
			buffers[current_buffer].length = length;
			state = PAYLOAD;
			DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, buffers[current_buffer].data, (void *)&(USART1->RXDATA), length + 1);
		}
		else
		{
			state = SYNC;
			DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, &data, (void *)&(USART1->RXDATA), 0);
		}
		break;
	case PAYLOAD:
		for (int index = 0; index < length; ++index) {
			chkA += buffers[current_buffer].data[index];
			chkB += chkA;
		}
		if (chkB == buffers[current_buffer].data[length] && chkA == buffers[current_buffer].data[length+1]) {
			sched_post_task(&uart_task);
		}
		state = SYNC;
		DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, &data, (void *)&(USART1->RXDATA), 0);
		break;
	}
}

static void dma_init(void)
{
	/* Enable DMA */
	CMU_ClockEnable(cmuClock_DMA, true);
	DMA_Init_TypeDef dma_init;
	dma_init.hprot = 0;
	dma_init.controlBlock = dmaControlBlock;
	DMA_Init(&dma_init);

	/* Rx channel */
	DMA_CfgChannel_TypeDef rx_channel_cfg;
	rx_channel_cfg.highPri = false;
	rx_channel_cfg.enableInt = true;
	rx_channel_cfg.select = DMAREQ_USART1_RXDATAV;  // TODO make configurable?
	rx_channel_cfg.cb = &dma_callbacks[DMA_CHANNEL_RX];
	rx_channel_cfg.cb->cbFunc = dma_rx_complete;
	rx_channel_cfg.cb->userPtr = NULL;
	DMA_CfgChannel(DMA_CHANNEL_RX, &rx_channel_cfg);

	DMA_CfgDescr_TypeDef rx_descr_cfg;
	rx_descr_cfg.srcInc = dmaDataIncNone;
	rx_descr_cfg.dstInc = dmaDataInc1;
	rx_descr_cfg.size = dmaDataSize1;
	rx_descr_cfg.arbRate = dmaArbitrate1;
	rx_descr_cfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL_RX, true, &rx_descr_cfg);

	/* Tx channel */
	DMA_CfgChannel_TypeDef tx_channel_cfg;
	tx_channel_cfg.highPri = false;
	tx_channel_cfg.enableInt = false;
	tx_channel_cfg.select = DMAREQ_USART1_TXBL;  // TODO make configurable?
	tx_channel_cfg.cb = &dma_callbacks[DMA_CHANNEL_TX];
	tx_channel_cfg.cb->cbFunc = NULL;
	tx_channel_cfg.cb->userPtr = NULL;
	DMA_CfgChannel(DMA_CHANNEL_TX, &tx_channel_cfg);

	DMA_CfgDescr_TypeDef tx_descr_cfg;
	tx_descr_cfg.srcInc = dmaDataInc1;
	tx_descr_cfg.dstInc = dmaDataIncNone;
	tx_descr_cfg.size = dmaDataSize1;
	tx_descr_cfg.arbRate = dmaArbitrate1;
	tx_descr_cfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL_TX, true, &tx_descr_cfg);
}

__LINK_C void __uart_init(void) {
	num_registered_callbacks = 0;
	sched_register_task(&uart_task);

	uart = uart_init(UART_NUM, UART_BAUDRATE, UART_LOCATION);
	bool result = uart_enable(uart);
	assert(result);

	dma_init();
	DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, &data, (void *)&(USART1->RXDATA), 0);
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
	end_atomic();
	return SUCCESS;
}

__LINK_C void  uart_send(uint8_t *buffer, size_t length)
{
	if (length < UART_BUFFER_SIZE && length > 0)
	{
		start_atomic();
			uint8_t *buf = tx_buffer[current_tx_buffer];
			current_tx_buffer ^= 1;
		end_atomic();
		memcpy(buf, buffer, length);

		while (DMA_ChannelEnabled(DMA_CHANNEL_TX)) /* Wait for Tx channel */;
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false, (void *)&(USART1->TXDATA), buf, length - 1);
	}
}

static void uart_task()
{
	uint8_t *data = buffers[current_buffer].data;
	size_t length = buffers[current_buffer].length;

	for (int index = 0; index < UART_QUEUE_SIZE; ++index)
	{
		uart_callback_t callback = 0x0;
		start_atomic();
		callback = callbacks[index];
		end_atomic();

		if (callback != 0x0)
			callback(data, length);
	}
}
