/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2016 University of Antwerp
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

/* \file
 *
 * Interface to the serial UART of the RFD900x module. This file is NOT a part of the
 * 'HAL' interface since not every platform will have a UART.
 *
 */

#ifndef PLATFORM_UART_H_
#define PLATFORM_UART_H_

#include "link_c.h"
#include "types.h"
#include "platform.h"

/* \brief The callback function for when an incoming message is received asynchronously over the UART.
 *
 * \param buffer	The buffer containing the received message
 * \param length	The length of the message
 * **/
typedef void (*uart_callback_t)(uint8_t *buffer, size_t length);

/* \brief Register a function to be called when an incoming message is received.
 *
 * Multiple callback functions can be registered, but the same function cannot be registered twice.
 * If a previously registered callback is re-registered, EALREADY is returned.
 *
 *  \param	callback	The function to call when an asynchronous message is received
 *  \return	error_t		SUCCESS if the callback was successfully registered
 *  					EINVAL if callback is 0x0
 *						EALREADY if the callback was already registered
 *						ENOMEM	if the callback could not be registered because there are already too many
 *								callbacks registered
 */
__LINK_C error_t uart_register_callback(uart_callback_t callback);

/* \brief Deregister a callback function previously registered using 'uart_register_callback'.
 *
 *  \param	callback	The function to deregister
 *  \return	error_t		SUCCESS if the callback was successfully deregistered
 *  					EINVAL if callback is 0x0
 *						EALREADY if the callback was not registered for this button
 */
__LINK_C error_t uart_deregister_callback(uart_callback_t callback);

/* \brief Transmit a buffer synchronously over the UART.
 *
 * \param  buffer		The buffer containing the bytes to send
 * \param  length		The number of bytes to send
 */
__LINK_C void uart_send(uint8_t *buffer, size_t length);

/* Not a user function */
__LINK_C void __uart_init(void);


#endif /* PLATFORM_UART_H_ */
