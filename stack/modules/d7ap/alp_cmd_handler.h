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

/*
 * \author	glenn.ergeerts@uantwerpen.be
 */

#ifndef ALP_CMD_HANDLER_H
#define ALP_CMD_HANDLER_H

#include "types.h"
#include "fifo.h"
#include "d7asp.h"


#define ALP_CMD_HANDLER_ID 'D'


///
/// \brief Shell command handler for ALP interface
/// \param cmd_fifo
///
void alp_cmd_handler(fifo_t* cmd_fifo);

///
/// \brief Process the ALP command against the local FS and returns the output using the shell interface.
/// \param alp_command
/// \param alp_command_length
///
void alp_cmd_handler_process_fs_itf(uint8_t* alp_command, uint8_t alp_command_length);

///
/// \brief Output received unsollicited response to the shell interface
/// \param d7asp_result
/// \param alp_command
/// \param alp_command_size
///
void alp_cmd_handler_output_unsollicited_response(d7asp_result_t d7asp_result, uint8_t *alp_command, uint8_t alp_command_size);

#endif // ALP_CMD_HANDLER_H
