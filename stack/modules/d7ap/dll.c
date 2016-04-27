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


#include "log.h"
#include "dll.h"
#include "hwradio.h"
#include "packet_queue.h"
#include "packet.h"
#include "crc.h"
#include "debug.h"
#include "fs.h"
#include "ng.h"
#include "hwdebug.h"
#include "random.h"
#include "MODULE_D7AP_defs.h"

#if defined(FRAMEWORK_LOG_ENABLED) && defined(MODULE_D7AP_DLL_LOG_ENABLED)
#define DPRINT(...) log_print_stack_string(LOG_STACK_DLL, __VA_ARGS__)
#else
#define DPRINT(...)
#endif



typedef enum
{
    DLL_STATE_IDLE,
    DLL_STATE_SCAN_AUTOMATION,
    DLL_STATE_CSMA_CA_STARTED,
	DLL_STATE_CSMA_CA_RETRY,
    DLL_STATE_CCA1,
    DLL_STATE_CCA2,
    DLL_STATE_CCA_FAIL,
    DLL_STATE_FOREGROUND_SCAN,
    DLL_STATE_BACKGROUND_SCAN,
    DLL_STATE_TX_FOREGROUND,
    DLL_STATE_TX_FOREGROUND_COMPLETED
} dll_state_t;

static dae_access_profile_t* NGDEF(_current_access_profile);
#define current_access_profile NG(_current_access_profile)

static dae_access_profile_t NGDEF(_scan_access_profile);
#define scan_access_profile NG(_scan_access_profile)

#define NO_ACTIVE_ACCESS_CLASS 0xFF
static uint8_t NGDEF(_active_access_class);
#define active_access_class NG(_active_access_class)

static dll_state_t NGDEF(_dll_state);
#define dll_state NG(_dll_state)

static packet_t* NGDEF(_current_packet);
#define current_packet NG(_current_packet)

// CSMA-CA parameters
static int16_t NGDEF(_dll_tca);
#define dll_tca NG(_dll_tca)

static int16_t NGDEF(_dll_tca0);
#define dll_tca0 NG(_dll_tca0)

static int16_t NGDEF(_dll_to);
#define dll_to NG(_dll_to)

static uint16_t NGDEF(_dll_slot_duration);
#define dll_slot_duration NG(_dll_slot_duration)

static uint16_t NGDEF(_dll_rigd_n);
#define dll_rigd_n NG(_dll_rigd_n)

static uint32_t NGDEF(_dll_cca_started);
#define dll_cca_started NG(_dll_cca_started)

// TODO defined somewhere?
#define t_g	5

static hw_radio_packet_t* alloc_new_packet(uint8_t length)
{
    // note we don't use length because in the current implementation the packets in the queue are of
    // fixed (maximum) size
    return &(packet_queue_alloc_packet()->hw_radio_packet);
}

static void release_packet(hw_radio_packet_t* hw_radio_packet)
{
    packet_queue_free_packet(packet_queue_find_packet(hw_radio_packet));
}

static void switch_state(dll_state_t next_state)
{
    switch(next_state)
    {
    case DLL_STATE_CSMA_CA_STARTED:
        assert(dll_state == DLL_STATE_IDLE || dll_state == DLL_STATE_SCAN_AUTOMATION
               || dll_state == DLL_STATE_FOREGROUND_SCAN);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_CSMA_CA_STARTED");
        break;
    case DLL_STATE_CSMA_CA_RETRY:
        assert(dll_state == DLL_STATE_CCA1 || dll_state == DLL_STATE_CCA2);
		dll_state = next_state;
		DPRINT("Switched to DLL_STATE_CSMA_CA_RETRY");
		break;
    case DLL_STATE_CCA1:
        assert(dll_state == DLL_STATE_CSMA_CA_STARTED || dll_state == DLL_STATE_CSMA_CA_RETRY);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_CCA1");
        break;
    case DLL_STATE_CCA2:
        assert(dll_state == DLL_STATE_CCA1);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_CCA2");
        break;
    case DLL_STATE_FOREGROUND_SCAN:
        assert(dll_state == DLL_STATE_IDLE || dll_state == DLL_STATE_SCAN_AUTOMATION
               || dll_state == DLL_STATE_TX_FOREGROUND_COMPLETED);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_FOREGROUND_SCAN");
        break;
    case DLL_STATE_IDLE:
        assert(dll_state == DLL_STATE_FOREGROUND_SCAN || dll_state == DLL_STATE_CCA_FAIL
               || dll_state == DLL_STATE_TX_FOREGROUND_COMPLETED);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_IDLE");
        break;
    case DLL_STATE_SCAN_AUTOMATION:
        assert(dll_state == DLL_STATE_FOREGROUND_SCAN
               || dll_state == DLL_STATE_IDLE
               || dll_state == DLL_STATE_TX_FOREGROUND_COMPLETED
               || dll_state == DLL_STATE_SCAN_AUTOMATION);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_SCAN_AUTOMATION");
        break;
    case DLL_STATE_TX_FOREGROUND:
        assert(dll_state == DLL_STATE_CCA2);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_TX_FOREGROUND");
        break;
    case DLL_STATE_TX_FOREGROUND_COMPLETED:
        assert(dll_state == DLL_STATE_TX_FOREGROUND);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_TX_FOREGROUND_COMPLETED");
        break;
    case DLL_STATE_CCA_FAIL:
        assert(dll_state == DLL_STATE_CCA1 || dll_state == DLL_STATE_CCA2
        		|| dll_state == DLL_STATE_CSMA_CA_STARTED || dll_state == DLL_STATE_CSMA_CA_RETRY);
        dll_state = next_state;
        DPRINT("Switched to DLL_STATE_CCA_FAIL");
        break;
    default:
        assert(false);
    }
}

static void process_received_packets()
{
    packet_t* packet = packet_queue_get_received_packet();
    assert(packet != NULL);
    DPRINT("Processing received packet");
    packet_queue_mark_processing(packet);
    packet_disassemble(packet);

    return;

    // TODO check if more received packets are pending
}

void packet_received(hw_radio_packet_t* packet)
{
    assert(dll_state == DLL_STATE_FOREGROUND_SCAN || dll_state == DLL_STATE_SCAN_AUTOMATION);

    // we are in interrupt context here, so mark packet for further processing,
    // schedule it and return
    DPRINT("packet received @ %i , RSSI = %i", packet->rx_meta.timestamp, packet->rx_meta.rssi);
    packet_queue_mark_received(packet);
    sched_post_task(&process_received_packets);
}

static void packet_transmitted(hw_radio_packet_t* hw_radio_packet)
{
    assert(dll_state == DLL_STATE_TX_FOREGROUND);
    switch_state(DLL_STATE_TX_FOREGROUND_COMPLETED);
    DPRINT("Transmitted packet with length = %i", hw_radio_packet->length);
    packet_t* packet = packet_queue_find_packet(hw_radio_packet);
    d7atp_signal_packet_transmitted(packet);
}

static void execute_cca();
static void execute_csma_ca();

static void cca_rssi_valid(int16_t cur_rssi)
{
    assert(dll_state == DLL_STATE_CCA1 || dll_state == DLL_STATE_CCA2);
    DEBUG_PIN_SET(2); // TODO tmp
    if (cur_rssi <= E_CCA)
    {
        if(dll_state == DLL_STATE_CCA1)
        {
            DPRINT("CCA1 RSSI: %d", cur_rssi);
            switch_state(DLL_STATE_CCA2);
            timer_post_task_delay(&execute_cca, 5);
            return;
        }
        else if(dll_state == DLL_STATE_CCA2)
        {
            // OK, send packet
            DPRINT("CCA2 RSSI: %d", cur_rssi);
            DPRINT("CCA2 succeeded, transmitting ...");
            // log_print_data(current_packet->hw_radio_packet.data, current_packet->hw_radio_packet.length + 1); // TODO tmp

            switch_state(DLL_STATE_TX_FOREGROUND);

            error_t err = hw_radio_send_packet(&current_packet->hw_radio_packet, &packet_transmitted);
            assert(err == SUCCESS);

            hw_radio_set_idle(); // ensure radio goes back to IDLE after transmission instead of to RX (which was previous state because of CCA)

            d7atp_signal_packet_csma_ca_insertion_completed(true);
            return;
        }
    }
    else
    {
        DPRINT("Channel not clear, RSSI: %i", cur_rssi);
        switch_state(DLL_STATE_CSMA_CA_RETRY);
        execute_csma_ca();

        //switch_state(DLL_STATE_CCA_FAIL);
        //d7atp_signal_packet_csma_ca_insertion_completed(false);
    }
}

static void execute_cca()
{
    assert(dll_state == DLL_STATE_CCA1 || dll_state == DLL_STATE_CCA2);
    DEBUG_PIN_CLR(2); // TODO tmp

    hw_rx_cfg_t rx_cfg =(hw_rx_cfg_t){
        .channel_id.channel_header = current_access_profile->subbands[0].channel_header,
        .channel_id.center_freq_index = current_access_profile->subbands[0].channel_index_start,
        .syncword_class = PHY_SYNCWORD_CLASS1,
    };

    hw_radio_set_rx(&rx_cfg, NULL, &cca_rssi_valid);
}

static uint16_t calculate_tx_duration()
{
    int data_rate = 6; // Normal rate: 6.9 bytes/tick
    // TODO select correct subband
    switch (current_access_profile->subbands[0].channel_header.ch_class)
    {
    case PHY_CLASS_LO_RATE:
        data_rate = 1; // Lo Rate: 1.2 bytes/tick
        break;
    case PHY_CLASS_HI_RATE:
        data_rate = 20; // High rate: 20.83 byte/tick
    }

    uint16_t duration = (current_packet->hw_radio_packet.length / data_rate) + 1;
    return duration;
}

static void execute_csma_ca()
{
    // TODO generate random channel queue
    DEBUG_PIN_SET(2); // TODO tmp
    //hw_radio_set_rx(NULL, NULL, NULL); // put radio in RX but disable callbacks to make sure we don't receive packets when in this state
                                        // TODO use correct rx cfg + it might be interesting to switch to idle first depending on calculated offset
    uint16_t tx_duration = calculate_tx_duration();
    switch (dll_state)
    {
        case DLL_STATE_CSMA_CA_STARTED:
        {
            dll_tca = current_access_profile->transmission_timeout_period - tx_duration;
            dll_cca_started = timer_get_counter_value();
            DPRINT("Tca= %i = %i - %i", dll_tca, current_access_profile->transmission_timeout_period, tx_duration);

            if (dll_tca <= 0)
            {
                // TODO how to handle this? signal upper layer?
                DPRINT("Tca negative, CCA failed");
                assert(false);
//				switch_state(DLL_STATE_CCA_FAIL);
//				sched_post_task(&execute_csma_ca);
                break;
            }

            uint16_t t_offset = 0;

            csma_ca_mode_t csma_ca_mode = current_access_profile->control_csma_ca_mode;
            // TODO overrule mode to UNC for subsequent requests by the requester, or a single response to a unicast request

            switch(csma_ca_mode)
            {
                case CSMA_CA_MODE_UNC:
                    // no delay
                    dll_slot_duration = 0;
                    break;
                case CSMA_CA_MODE_AIND: // TODO implement AIND
                {
                    dll_slot_duration = tx_duration;
                    // no initial delay
                    break;
                }
                case CSMA_CA_MODE_RAIND: // TODO implement RAIND
                {
                    dll_slot_duration = tx_duration;
                    uint16_t max_nr_slots = dll_tca / tx_duration;
                    uint16_t slots_wait = get_rnd() % max_nr_slots;
                    t_offset = slots_wait * tx_duration;
                    break;
                }
                case CSMA_CA_MODE_RIGD: // TODO implement RAIND
                {
                    dll_rigd_n = 0;
                    dll_tca0 = dll_tca;
                    dll_slot_duration = (uint16_t) ((double)dll_tca0) / (2 << (dll_rigd_n));
                    t_offset = get_rnd() % dll_slot_duration;
                    break;
                }
            }

            DPRINT("slot duration: %i t_offset: %i csma ca mode: %i", dll_slot_duration, t_offset, csma_ca_mode);

            dll_to = dll_tca - t_offset;

            if (t_offset > 0)
            {
                switch_state(DLL_STATE_CCA1);
                timer_post_task_delay(&execute_cca, t_offset);
            }
            else
            {
                switch_state(DLL_STATE_CCA1);
                sched_post_task(&execute_cca);
            }

            break;
        }
        case DLL_STATE_CSMA_CA_RETRY:
        {
        	int32_t cca_duration = timer_get_counter_value() - dll_cca_started;
        	dll_to -= cca_duration;


            DPRINT("RETRY dll_to = %i < %i ", dll_to, t_g);

            if (dll_to < t_g)
            {
                switch_state(DLL_STATE_CCA_FAIL);
                sched_post_task(&execute_csma_ca);
                break;
            }

            dll_tca = dll_to;
            dll_cca_started = timer_get_counter_value();
            uint16_t t_offset = 0;

            switch(current_access_profile->control_csma_ca_mode)
            {
                case CSMA_CA_MODE_AIND:
                case CSMA_CA_MODE_RAIND:
                {
                    uint16_t max_nr_slots = dll_tca / tx_duration;
                    uint16_t slots_wait = get_rnd() % max_nr_slots;
                    t_offset = slots_wait * tx_duration;
                    break;
                }
                case CSMA_CA_MODE_RIGD:
                {
                    dll_rigd_n++;
                    dll_slot_duration = (uint16_t) ((double)dll_tca0) / (2 << (dll_rigd_n+1));
                    if(dll_slot_duration != 0) // TODO can be 0, validate
                        t_offset = get_rnd() % dll_slot_duration;
                    else
                        t_offset = 0;

                    DPRINT("slot duration: %i", dll_slot_duration);
                    break;
                }
            }

            DPRINT("t_offset: %i", t_offset);

            dll_to = dll_tca - t_offset;

            if (t_offset > 0)
            {
                timer_post_task_delay(&execute_csma_ca, t_offset);
            }
            else
            {
                switch_state(DLL_STATE_CCA1);
                sched_post_task(&execute_cca);
            }

            break;
        }
        case DLL_STATE_CCA_FAIL:
        {
            // TODO hw_radio_set_idle();
        	switch_state(DLL_STATE_IDLE);
            d7atp_signal_packet_csma_ca_insertion_completed(false);
            break;
        }
    }
}

void dll_execute_scan_automation()
{
    uint8_t scan_access_class = fs_read_dll_conf_active_access_class();
    if(active_access_class != scan_access_class)
    {
        fs_read_access_class(scan_access_class, &scan_access_profile);
        active_access_class = scan_access_class;
    }

    current_access_profile = &scan_access_profile;

    if(current_access_profile->control_scan_type_is_foreground && current_access_profile->control_number_of_subbands > 0) // TODO background scan
    {
        assert(current_access_profile->control_number_of_subbands == 1); // TODO multiple not supported
        switch_state(DLL_STATE_SCAN_AUTOMATION);
        hw_rx_cfg_t rx_cfg = {
            .channel_id = {
                .channel_header = current_access_profile->subbands[0].channel_header,
                .center_freq_index = current_access_profile->subbands[0].channel_index_start
            },
            .syncword_class = PHY_SYNCWORD_CLASS1
        };

        hw_radio_set_rx(&rx_cfg, &packet_received, NULL);

        assert(current_access_profile->scan_automation_period == 0); // scan automation period, assuming 0 for now
    }
    else
    {
        // TODO should already be idle, remove?
        hw_radio_set_idle();

        // TODO wait until radio idle
        if(dll_state != DLL_STATE_IDLE)
            switch_state(DLL_STATE_IDLE);
    }
}

void dll_notify_dll_conf_file_changed()
{
    // when doing scan automation restart this
    if(dll_state == DLL_STATE_SCAN_AUTOMATION)
    {
        dll_execute_scan_automation();
    }
}

void dll_init()
{
    sched_register_task(&process_received_packets);
    sched_register_task(&dll_start_foreground_scan);
    sched_register_task(&execute_cca);
    sched_register_task(&execute_csma_ca);
    sched_register_task(&dll_execute_scan_automation);

    hw_radio_init(&alloc_new_packet, &release_packet);

    dll_state = DLL_STATE_IDLE;
    active_access_class = NO_ACTIVE_ACCESS_CLASS;
    sched_post_task(&dll_execute_scan_automation);
}

void dll_tx_frame(packet_t* packet, dae_access_profile_t* access_profile)
{
    current_access_profile = access_profile;
    dll_header_t* dll_header = &(packet->dll_header);
    dll_header->subnet = access_profile->subnet;
    dll_header->control_eirp_index = 0; // TODO hardcoded for now
    if(packet->d7atp_addressee != NULL)
    {
        dll_header->control_target_address_set = packet->d7atp_addressee->addressee_ctrl_has_id;
        dll_header->control_vid_used = packet->d7atp_addressee->addressee_ctrl_virtual_id;
    }

    packet_assemble(packet);

    packet->hw_radio_packet.tx_meta.tx_cfg = (hw_tx_cfg_t){
        .channel_id.channel_header = current_access_profile->subbands[0].channel_header,
        .channel_id.center_freq_index = current_access_profile->subbands[0].channel_index_start,
        .syncword_class = PHY_SYNCWORD_CLASS1,
        .eirp = 10
    };

    current_packet = packet;

    switch_state(DLL_STATE_CSMA_CA_STARTED);
    execute_csma_ca();
}

void dll_start_foreground_scan()
{
    switch_state(DLL_STATE_FOREGROUND_SCAN);
    // TODO handle Tscan timeout

    // TODO only access class using 1 subband which contains 1 channel index is supported for now

    hw_rx_cfg_t rx_cfg = {
        .channel_id = {
            .channel_header = current_access_profile->subbands[0].channel_header,
            .center_freq_index = current_access_profile->subbands[0].channel_index_start
        },
        .syncword_class = PHY_SYNCWORD_CLASS1
    };

    hw_radio_set_rx(&rx_cfg, &packet_received, NULL);
}

void dll_stop_foreground_scan()
{
    assert(dll_state == DLL_STATE_FOREGROUND_SCAN);
    dll_execute_scan_automation();
}

uint8_t dll_assemble_packet_header(packet_t* packet, uint8_t* data_ptr)
{
    uint8_t* dll_header_start = data_ptr;
    *data_ptr = packet->dll_header.subnet; data_ptr += sizeof(packet->dll_header.subnet);
    *data_ptr = packet->dll_header.control; data_ptr += sizeof(packet->dll_header.control);
    if(packet->dll_header.control_target_address_set)
    {
        uint8_t addr_len = packet->dll_header.control_vid_used? 2 : 8;
        memcpy(data_ptr, packet->d7atp_addressee->addressee_id, addr_len); data_ptr += addr_len;
    }

    return data_ptr - dll_header_start;
}

bool dll_disassemble_packet_header(packet_t* packet, uint8_t* data_idx)
{
    packet->dll_header.subnet = packet->hw_radio_packet.data[(*data_idx)]; (*data_idx)++;
    if(packet->dll_header.subnet != current_access_profile->subnet)
    {
        DPRINT("Subnet does not match current access profile, skipping packet");
        return false;
    }

    packet->dll_header.control = packet->hw_radio_packet.data[(*data_idx)]; (*data_idx)++;
    if(packet->dll_header.control_target_address_set)
    {
        uint8_t address_len;
        uint8_t id[8];
        if(!packet->dll_header.control_vid_used)
        {
            fs_read_uid(id);
            address_len = 8;
        }
        else
        {
            fs_read_vid(id);
            address_len = 2;
        }

        if(memcmp(packet->hw_radio_packet.data + (*data_idx), id, address_len) != 0)
        {
            DPRINT("Device ID filtering failed, skipping packet");
            return false;
        }

        (*data_idx) += address_len;
    }
    // TODO filter LQ
    // TODO pass to upper layer
    // TODO Tscan -= Trx
    return true;
}

