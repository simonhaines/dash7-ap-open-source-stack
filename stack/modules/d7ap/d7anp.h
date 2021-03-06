/*! \file d7anp.h
 *

 *  \copyright (C) Copyright 2015 University of Antwerp and others (http://oss-7.cosys.be)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \author glenn.ergeerts@uantwerpen.be
 *
 */

#ifndef D7ANP_H_
#define D7ANP_H_

#include "stdint.h"
#include "stdbool.h"

#include "dae.h"

typedef struct packet packet_t;

/*! \brief The D7ANP CTRL header
 *
 * note: bit order is important here since this is send over the air. We explicitly reverse the order to ensure BE.
 * Although bit fields can cause portability problems it seems fine for now using gcc and the current platforms.
 * If this poses problems in the future we must resort to bit arithmetics and flags.
 */
typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t origin_access_class : 4;
            bool origin_access_id_is_vid : 1;
            bool origin_access_id_present : 1;
            bool hop_enabled : 1;
            bool nls_enabled : 1;
        };
    };
} d7anp_ctrl_t;

void d7anp_tx_foreground_frame(packet_t* packet, bool should_include_origin_template, dae_access_profile_t* access_profile);
uint8_t d7anp_assemble_packet_header(packet_t* packet, uint8_t* data_ptr);
bool d7anp_disassemble_packet_header(packet_t* packet, uint8_t* packet_idx);

#endif /* D7ANP_H_ */
