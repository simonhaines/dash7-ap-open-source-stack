/*! \file fs.c
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
 *  \author glenn.ergeerts@uantwerpen.be
 *  \author maarten.weyn@uantwerpen.be
 *
 */

#include "string.h"
#include "assert.h"
#include "fs.h"
#include "ng.h"
#include "hwsystem.h"
#include "alp.h"
#include "d7asp.h"

#define FILE_COUNT 0x42 // TODO define from cmake (D7AP module specific)
#define FILE_DATA_SIZE 60 // TODO define from cmake (D7AP module specific)

static fs_file_header_t NGDEF(_file_headers)[FILE_COUNT] = { 0 };
#define file_headers NG(_file_headers)

static uint8_t NGDEF(_data)[FILE_DATA_SIZE] = { 0 };
#define data NG(_data)

static uint8_t NGDEF(_current_data_offset); // TODO we are using offset here instead of pointer because NG does not support pointers, fix later when NG is replaced
#define current_data_offset NG(_current_data_offset)

static uint16_t NGDEF(_file_offsets)[FILE_COUNT] = { 0 };
#define file_offsets NG(_file_offsets)

static uint16_t NGDEF(_is_fs_init_completed);
#define is_fs_init_completed NG(_is_fs_init_completed)

#define D7A_FILE_UID_FILE_ID 0x00
#define D7A_FILE_UID_SIZE 8

#define D7A_FILE_DLL_CONF_FILE_ID	0x0A
#define D7A_FILE_DLL_CONF_SIZE		6

#define D7A_FILE_ACCESS_PROFILE_ID 0x20 // the first access class file
#define D7A_FILE_ACCESS_PROFILE_SIZE 12 // TODO assuming 1 subband

#define ACTION_FILE_ID_BROADCAST_COUNTER 0x41

static inline bool is_file_defined(uint8_t file_id)
{
    return file_headers[file_id].length != 0;
}

static void execute_alp_command(uint8_t command_file_id)
{
    assert(is_file_defined(command_file_id));
    uint8_t* data_ptr = (uint8_t*)(data + file_offsets[command_file_id]);
    uint8_t* file_start = data_ptr;

    // parse ALP command
    d7asp_fifo_config_t fifo_config;
    assert((*data_ptr) == ALP_ITF_ID_D7ASP); // only D7ASP supported for now
    data_ptr++;
    fifo_config.fifo_ctrl = (*data_ptr); data_ptr++;
    fifo_config.qos.qos_ctrl = (*data_ptr); data_ptr++;
    fifo_config.qos.qos_ack_period = (*data_ptr); data_ptr++;
    fifo_config.qos.qos_retry_single = (*data_ptr); data_ptr++;
    fifo_config.qos.qos_retry_total = (*data_ptr); data_ptr++;
    fifo_config.dormant_timeout = (*data_ptr); data_ptr++;
    fifo_config.start_id = (*data_ptr); data_ptr++;
    fifo_config.addressee.addressee_ctrl = (*data_ptr); data_ptr++;
    memcpy(&(fifo_config.addressee.addressee_id), data_ptr, 8); data_ptr += 8; // TODO assume 8 for now

    d7asp_queue_alp_actions(&fifo_config, data_ptr, file_headers[command_file_id].length - (uint8_t)(data_ptr - file_start));
}

static void write_access_class(uint8_t access_class_index, dae_access_profile_t* access_class)
{
    assert(access_class_index < 16);
    assert(access_class->control_number_of_subbands == 1); // TODO only one supported for now
    data[current_data_offset] = access_class->control; current_data_offset++;
    data[current_data_offset] = access_class->subnet; current_data_offset++;
    data[current_data_offset] = access_class->scan_automation_period; current_data_offset++;
    data[current_data_offset] = access_class->transmission_timeout_period; current_data_offset++;
    data[current_data_offset] = 0x00; current_data_offset++; // RFU
    // subbands, only 1 supported for now
    memcpy(data + current_data_offset, &(access_class->subbands[0].channel_header), 1); current_data_offset++;
    memcpy(data + current_data_offset, &(access_class->subbands[0].channel_index_start), 2); current_data_offset += 2;
    memcpy(data + current_data_offset, &(access_class->subbands[0].channel_index_end), 2); current_data_offset += 2;
    data[current_data_offset] = access_class->subbands[0].eirp; current_data_offset++;
    data[current_data_offset] = access_class->subbands[0].ccao; current_data_offset++;
}

void fs_init(fs_init_args_t* init_args)
{
    // TODO store as big endian!
    is_fs_init_completed = false;
    current_data_offset = 0;

    // UID
    file_offsets[D7A_FILE_UID_FILE_ID] = current_data_offset;
    file_headers[D7A_FILE_UID_FILE_ID] = (fs_file_header_t){
        .file_properties.action_protocol_enabled = 0,
        .file_properties.storage_class = FS_STORAGE_PERMANENT,
        .file_properties.permissions = 0, // TODO
        .length = D7A_FILE_UID_SIZE
    };

    uint64_t id = hw_get_unique_id();
    uint64_t id_be = __builtin_bswap64(id);
    memcpy(data + current_data_offset, &id_be, D7A_FILE_UID_SIZE);
    current_data_offset += D7A_FILE_UID_SIZE;

    file_offsets[D7A_FILE_DLL_CONF_FILE_ID] = current_data_offset;
	file_headers[D7A_FILE_DLL_CONF_FILE_ID] = (fs_file_header_t){
		.file_properties.action_protocol_enabled = 0,
		.file_properties.storage_class = FS_STORAGE_RESTORABLE,
		.file_properties.permissions = 0, // TODO
		.length = D7A_FILE_UID_SIZE
	};

	memset(data + current_data_offset, 0, D7A_FILE_DLL_CONF_SIZE);
	current_data_offset += D7A_FILE_DLL_CONF_SIZE;

    // access profiles
    assert(init_args->access_profiles_count > 0 && init_args->access_profiles_count < 16);
    dae_access_profile_t* access_class = init_args->access_profiles;
    for(uint8_t i = 0; i < init_args->access_profiles_count; i++)
    {
        file_offsets[D7A_FILE_ACCESS_PROFILE_ID + i] = current_data_offset;
        write_access_class(i, access_class);
        file_headers[D7A_FILE_ACCESS_PROFILE_ID + i] = (fs_file_header_t){
            .file_properties.action_protocol_enabled = 0,
            .file_properties.storage_class = FS_STORAGE_PERMANENT,
            .file_properties.permissions = 0, // TODO
            .length = D7A_FILE_ACCESS_PROFILE_SIZE
        };

        access_class += sizeof(dae_access_profile_t);
    }

    // init user files
    if(init_args->fs_user_files_init_cb)
        init_args->fs_user_files_init_cb();

    assert(current_data_offset <= FILE_DATA_SIZE);
    is_fs_init_completed = true;
}

void fs_init_file(uint8_t file_id, const fs_file_header_t* file_header, const uint8_t* initial_data)
{
    assert(!is_fs_init_completed); // initing files not allowed after fs_init() completed (for now?)
    file_offsets[file_id] = current_data_offset;
    memcpy(file_headers + file_id, file_header, sizeof(fs_file_header_t));
    memset(data + current_data_offset, 0, file_header->length);
    current_data_offset += file_header->length;
    if(initial_data != NULL)
        fs_write_file(file_id, 0, initial_data, file_header->length);
}

void fs_init_file_with_D7AActP(uint8_t file_id, const d7asp_fifo_config_t* fifo_config, const alp_control_t* alp_ctrl, const uint8_t* alp_operand)
{
    uint8_t alp_command_buffer[40] = { 0 };
    uint8_t* ptr = alp_command_buffer;
    (*ptr) = ALP_ITF_ID_D7ASP; ptr++;
    (*ptr) = fifo_config->fifo_ctrl; ptr++;
    (*ptr) = fifo_config->qos.qos_ctrl; ptr++;
    (*ptr) = fifo_config->qos.qos_ack_period; ptr++;
    (*ptr) = fifo_config->qos.qos_retry_single; ptr++;
    (*ptr) = fifo_config->qos.qos_retry_total; ptr++;
    (*ptr) = fifo_config->dormant_timeout; ptr++;
    (*ptr) = fifo_config->start_id; ptr++;
    (*ptr) = fifo_config->addressee.addressee_ctrl; ptr++;
    memcpy(ptr, &(fifo_config->addressee.addressee_id), 8); ptr += 8; // TODO assume 8 for now

    (*ptr) = alp_ctrl->raw; ptr++;

    uint8_t alp_operand_len = 0;
    switch(alp_ctrl->operation)
    {
        case ALP_OP_READ_FILE_DATA:
            alp_operand_len = 3; // File Offset Operand + requested Data Length
                //TODO File Offset Operand can be 2-5 bytes actually, depending on File Offset Field Length, see spec
            break;
        default:
            assert(false);
    }

    memcpy(ptr, alp_operand, alp_operand_len); ptr += alp_operand_len;

    // TODO fixed header implemented here, or should this be configurable by app?
    fs_file_header_t action_file_header = (fs_file_header_t){
        .file_properties.action_protocol_enabled = 0,
        .file_properties.storage_class = FS_STORAGE_PERMANENT,
        .file_properties.permissions = 0, // TODO
        .length = ptr - alp_command_buffer
    };

    fs_init_file(file_id, &action_file_header, alp_command_buffer);
}

void fs_read_file(uint8_t file_id, uint8_t offset, uint8_t* buffer, uint8_t length)
{
    assert(is_file_defined(file_id));
    assert(file_headers[file_id].length >= offset + length);
    memcpy(buffer, data + file_offsets[file_id] + offset, length);
}

void fs_write_file(uint8_t file_id, uint8_t offset, const uint8_t* buffer, uint8_t length)
{
    assert(is_file_defined(file_id));
    assert(file_headers[file_id].length >= offset + length);
    memcpy(data + file_offsets[file_id] + offset, buffer, length);

    if(file_headers[file_id].file_properties.action_protocol_enabled == true
            && file_headers[file_id].file_properties.action_condition == ALP_ACT_COND_WRITE) // TODO ALP_ACT_COND_WRITEFLUSH?
    {
        execute_alp_command(file_headers[file_id].file_properties.action_file_id);
    }
}

void fs_read_uid(uint8_t *buffer)
{
    fs_read_file(D7A_FILE_UID_FILE_ID, 0, buffer, D7A_FILE_UID_SIZE);
}

void fs_read_access_class(uint8_t access_class_index, dae_access_profile_t *access_class)
{
    assert(access_class_index < 16);
    assert(is_file_defined(D7A_FILE_ACCESS_PROFILE_ID + access_class_index));
    uint8_t* data_ptr = data + file_offsets[D7A_FILE_ACCESS_PROFILE_ID + access_class_index];
    access_class->control = (*data_ptr); data_ptr++;
    access_class->subnet = (*data_ptr); data_ptr++;
    access_class->scan_automation_period = (*data_ptr); data_ptr++;
    access_class->transmission_timeout_period = (*data_ptr); data_ptr++;
    data_ptr++; // RFU
    // subbands, only 1 supported for now
    memcpy(&(access_class->subbands[0].channel_header), data_ptr, 1); data_ptr++;
    memcpy(&(access_class->subbands[0].channel_index_start), data_ptr, 2); data_ptr += 2;
    memcpy(&(access_class->subbands[0].channel_index_end), data_ptr, 2); data_ptr += 2;
    access_class->subbands[0].eirp = (*data_ptr); data_ptr++;
    access_class->subbands[0].ccao = (*data_ptr); data_ptr++;
}