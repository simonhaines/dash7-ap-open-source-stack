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

/* \file
 *
 * Driver for si4460
 *
 * @author maarten.weyn@uantwerpen.be
 *
 * Currently implemented for 26 MHZ RF Xtal
 * TODO: SYNTH_LPFILTx is dependent on hardware define in platform
 *
 * TODO: PKT_RX_THRESHOLD -> IRQ when Rx fifo almost full
 * TODO: PKT_TX_THRESHOLD
 */

#include "debug.h"
#include "string.h"

#include "log.h"
#include "hwradio.h"
#include "hwsystem.h"
#include "hwdebug.h"
#include "hwgpio.h"
#include "types.h"
#include "fec.h"


// NOTE not really needed, here to get configure_channel() to compile
#include "si4460_registers.h"


/***** EZRadioDrv definitions *****/
#include "ecode.h"
#include "ezradio_cmd.h"
#include "ezradio_hal.h"
#include "ezradio_api_lib.h"
#include "ezradio_api_lib_add.h"

#define ECODE_EMDRV_EZRADIODRV_OK                       ( ECODE_OK )                                   ///< Success return value.
#define ECODE_EMDRV_EZRADIODRV_ILLEGAL_HANDLE           ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000001 )   ///< Illegal SPI handle.
#define ECODE_EMDRV_EZRADIODRV_TRANSMIT_PLUGIN_BASE             ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000100 )   ///< Transmit plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_RECEIVE_PLUGIN_BASE              ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000200 )   ///< Receive plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_CRC_ERROR_PLUGIN_BASE            ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000300 )   ///< CRC error plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_AUTO_ACK_PLUGIN_BASE             ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000400 )   ///< Receive plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_UNMOD_CARRIER_PLUGIN_BASE        ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000500 )   ///< Receive plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_PN9_PLUGIN_BASE                  ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000600 )   ///< Receive plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_DIRECT_TRANSMIT_PLUGIN_BASE      ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000700 )   ///< Transmit plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_DIRECT_RECEIVE_PLUGIN_BASE       ( ECODE_EMDRV_EZRADIODRV_BASE | 0x00000800 )   ///< Receive plugin base error code.
#define ECODE_EMDRV_EZRADIODRV_TRANSMIT_FAILED          		( ECODE_EMDRV_EZRADIODRV_TRANSMIT_PLUGIN_BASE | 0x00000001 )   ///< Unable to start transmission.


#include "ezradio_prop.h"
#include "rfd900x_pwm.h"
#include "rfd900x_amp.h"

#include "915_HR.H"

#if defined(FRAMEWORK_LOG_ENABLED) && defined(FRAMEWORK_PHY_LOG_ENABLED) // TODO more granular (LOG_PHY_ENABLED)
#define DPRINT(...) log_print_stack_string(LOG_STACK_PHY, __VA_ARGS__)
#define DPRINT_PACKET(...) log_print_raw_phy_packet(__VA_ARGS__)
#define DPRINT_DATA(...) log_print_data(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINT_PACKET(...)
#define DPRINT_DATA(...)
#endif

#define RSSI_OFFSET 64 // if this is changed also change radio register 0x20,0x4e

#if DEBUG_PIN_NUM >= 2
    #define DEBUG_TX_START() hw_debug_set(0);
    #define DEBUG_TX_END() hw_debug_clr(0);
    #define DEBUG_RX_START() hw_debug_set(1);
    #define DEBUG_RX_END() hw_debug_clr(1);
#else
    #define DEBUG_TX_START()
    #define DEBUG_TX_END()
    #define DEBUG_RX_START()
    #define DEBUG_RX_END()
#endif

#ifdef HAL_RADIO_USE_HW_CRC
static bool has_hardware_crc = true;
#else
static bool has_hardware_crc = false;
#endif

/** \brief The possible states the radio can be in
 */
typedef enum
{
    HW_RADIO_STATE_IDLE,
    HW_RADIO_STATE_TX,
    HW_RADIO_STATE_RX,
    HW_RADIO_STATE_OFF,
    HW_RADIO_STATE_UNKOWN
} hw_radio_state_t;



static alloc_packet_callback_t alloc_packet_callback;
static release_packet_callback_t release_packet_callback;
static rx_packet_callback_t rx_packet_callback;
static tx_packet_callback_t tx_packet_callback;
static rssi_valid_callback_t rssi_valid_callback;
static hw_radio_state_t current_state;
static hw_radio_packet_t* current_packet;
static channel_id_t current_channel_id = {
    .channel_header_raw = 0xFF,
    .center_freq_index = 0xFF
};

static uint8_t ez_channel_id = 0;
static hw_radio_packet_t* rx_packet;

static eirp_t current_eirp = 20;

static bool should_rx_after_tx_completed = false;
static uint16_t tx_fifo_data_length = 0;
static uint16_t rx_fifo_data_lenght = 0;

static hw_rx_cfg_t current_rx_cfg = {0x0000, PHY_SYNCWORD_CLASS0};
static syncword_class_t current_syncword_class = PHY_SYNCWORD_CLASS0;

static inline int16_t convert_rssi(uint8_t rssi_raw);
static void start_rx(hw_rx_cfg_t const* rx_cfg);
static void ezradio_int_callback();
static void report_rssi();


// RFD900x power levels
#define RFD900_INT_TX_POW 26           // TX power level into amp (0-127)not linear, 10dbm out, 6.8 after atten
#define NUM_POWER_LEVELS 16
#define POWER_LEVEL_STEP 2
static const uint8_t power_levels[NUM_POWER_LEVELS] = { 235, 230, 224, 218, 211, 206, 201, 196, 190, 184, 178, 171, 164, 150, 136, 80 };
static void radio_set_transmit_power(uint8_t power);
static uint8_t calibration_get(uint8_t level);

static void radio_set_transmit_power(uint8_t power)
{
	uint8_t i;
	ezradio_set_property(EZRADIO_PROP_GRP_ID_PA, 1u, EZRADIO_PROP_GRP_INDEX_PA_PWR_LVL, RFD900_INT_TX_POW);
	i = calibration_get(power);
	if (i != 0xFF)
	{
		pwm_set_duty(i);
	}
	else
	{
		i = power / POWER_LEVEL_STEP;
		pwm_set_duty(power_levels[i]);
	}
}

static uint8_t calibration_get(uint8_t level)
{
	// In parameters.c, this pulls a calibration value out of flash
	// (that is set through an AT command), for now return the default;
#if 0
	uint8_t *calibration = (uint8_t *)(FLASH_CALIBRATION_AREA);
	if(((/*calibration_crc != 0xFF && calibration_crc == crc && */(calibration[level] != 0xff) && level <= BOARD_MAXTXPOWER))||
		 ((level >= CalParam_BAND)&&(level < CalParam_LAST)) )
	{
		return calibration[level];
	}
#endif
	return 0xFF;
}


/***** ezradio driver interface *****/
#define RADIO_CONFIG_DATA_RADIO_DELAY_AFTER_RESET_US (10000)
/* Radio configuration data array. */
static const uint8_t Radio_Configuration_Data_Array[]  = RADIO_CONFIGURATION_DATA_ARRAY;
/* Radio interrupt receive flag */
static bool ezradioIrqReceived = false;
static task_t int_callback = NULL;
static void ezradioInit(task_t cb);
static void ezradioPowerUp(void);
static void ezradioResetTRxFifo(void);
static Ecode_t ezradioStartRx(uint8_t channel, bool packet_handler);
static Ecode_t ezradioStartTx(hw_radio_packet_t* packet, uint8_t channel_id, bool rx_after, uint8_t data_lenght);
void GPIO_EZRadio_INT_IRQHandler(uint8_t pin);

static void ezradioInit(task_t cb)
{
	// Most of this is from ezradio_plugin_manager/ezradioInit()
  uint16_t wDelay;

  /* Initialize radio GPIOs and SPI port */
  ezradio_hal_GpioInit( GPIO_EZRadio_INT_IRQHandler, true );
  ezradio_hal_SpiInit();

  /* Power Up the radio chip */
  ezradioPowerUp();

  /* Load radio configuration */
  while (EZRADIO_CONFIG_SUCCESS != ezradio_configuration_init(Radio_Configuration_Data_Array))
  {
    /* Error hook */
#ifdef ERROR_HOOK
    ERROR_HOOK;
#else
    DPRINT("ERROR: Radio configuration failed!\n");
#endif
    for (wDelay = 0x7FFF; wDelay--; ) ;

    /* Power Up the radio chip */
    ezradioPowerUp();
  }

  /* Read ITs, clear pending ones */
  ezradio_get_int_status(0u, 0u, 0u, NULL);
  if (cb != NULL)
    int_callback = cb;

}

static void ezradioPowerUp(void)
{
	// Same as ezradio_plugin_manager/ezradioPowerUp()
	// except uses debug clock to delay after reset

	/* Hardware reset the chip */
	ezradio_reset();

	/* Delay for preconfigured time */
	hw_busy_wait(RADIO_CONFIG_DATA_RADIO_DELAY_AFTER_RESET_US);
}

static void ezradioResetTRxFifo(void)
{
	uint8_t fifo=0;
	fifo = EZRADIO_CMD_FIFO_INFO_ARG_FIFO_RX_BIT|
			EZRADIO_CMD_FIFO_INFO_ARG_FIFO_TX_BIT;
	ezradio_fifo_info_fast_reset(fifo);
}

static Ecode_t ezradioStartRx(uint8_t channel, bool packet_handler)
{
	ezradio_get_int_status(0u, 0u, 0u, NULL);

	ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_RX_BIT, NULL);

	// reset length of first field (can be corrupted by TX)
	ezradio_set_property(0x12, 0x02, 0x0D, 0x00, 0x01);
	if (packet_handler)
	{
		// packet handler mode: end of packet and CRC
		ezradio_set_property(0x12, 0x01, 0x06, 0x02);
		ezradio_start_rx(channel, 0u, 0u,
				  EZRADIO_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
				  EZRADIO_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
				  EZRADIO_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_READY);
	} else {
		// Direct RX mode - used in FEC: no end of packet, no CRC
		ezradio_set_property(0x12, 0x01, 0x06, 0x2);
		ezradio_start_rx(channel, 0u, 0xFF,
				  EZRADIO_CMD_START_RX_ARG_NEXT_STATE1_RXTIMEOUT_STATE_ENUM_NOCHANGE,
				  EZRADIO_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_READY,
				  EZRADIO_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_READY);
	}

    return ECODE_OK;
}

static Ecode_t ezradioStartTx(hw_radio_packet_t* packet, uint8_t channel_id, bool rx_after, uint8_t data_lenght)
{
	ezradio_cmd_reply_t ezradioReply;

	/* Request and check radio device state */
	ezradio_request_device_state(&ezradioReply);

	//TODO: what to do if already in TX
	if (ezradioReply.REQUEST_DEVICE_STATE.CURR_STATE == EZRADIO_CMD_REQUEST_DEVICE_STATE_REP_CURR_STATE_MAIN_STATE_ENUM_TX) {
	    return ECODE_EMDRV_EZRADIODRV_TRANSMIT_FAILED;
	}

	ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_READY);

	// clear pending interrupts
	ezradio_get_int_status_fast_clear();


	//Reset TX FIFO
	ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_TX_BIT, NULL);

	uint16_t chunck_lenght = data_lenght;
	if (chunck_lenght > 64) chunck_lenght = 64;

	/* Fill the TX fifo with data, CRC is added by HW*/
	ezradio_write_tx_fifo(chunck_lenght, packet->data);

	/* Start sending packet*/
	// RX state or idle state
	uint8_t next_state = rx_after ? 8 << 4 : 1 << 4;
	ezradio_start_tx(channel_id, next_state,  data_lenght);


	while (chunck_lenght < data_lenght)
	{
		ezradio_fifo_info(0, &ezradioReply);
		DPRINT("TX FIFO Space: %d", ezradioReply.FIFO_INFO.TX_FIFO_SPACE);
		uint16_t new_length = data_lenght - chunck_lenght;
		if (new_length > ezradioReply.FIFO_INFO.TX_FIFO_SPACE) new_length = ezradioReply.FIFO_INFO.TX_FIFO_SPACE;
		ezradio_write_tx_fifo(new_length, &(packet->data[chunck_lenght]));
		chunck_lenght += new_length;
		DPRINT ("%d added -> %d", chunck_lenght, data_lenght);
	}

	return ECODE_EMDRV_EZRADIODRV_OK;
}

void GPIO_EZRadio_INT_IRQHandler( uint8_t pin )
{
  (void)pin;

  /* Sign radio interrupt received */
  ezradioIrqReceived = true;

  if (int_callback)
	  int_callback();
}

/***** Hardware interface (hw_* functions, see /hal/chips/si4460/si4460.c) *****/

static void configure_channel(const channel_id_t* channel_id)
{

	// only change settings if channel_id changed compared to current config
	// TODO: check if only channel number changes

	if((channel_id->channel_header_raw != current_channel_id.channel_header_raw))
	{
		// DPRINT("configure_channel: %s", byte_to_binary(channel_id->channel_header_raw));
		if (channel_id->channel_header.ch_coding != current_channel_id.channel_header.ch_coding)
		{
			if (has_hardware_crc && channel_id->channel_header.ch_coding != PHY_CODING_FEC_PN9)
			{
				// use HW CRC
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PKT_LEN_ADJUST_HW_CRC);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_CRC_CONFIG_HW_CRC);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_CRC_CONFIG_HW_CRC);
			} else {
				// use SW CRC
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PKT_LEN_ADJUST_SW_CRC);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_1_CRC_CONFIG_SW_CRC);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PKT_FIELD_2_CRC_CONFIG_SW_CRC);
			}
		}
		// TODO assert valid center freq index

		memcpy(&current_channel_id, channel_id, sizeof(channel_id_t)); // cache new settings

		// TODO preamble size depends on channel class

		// set freq band
		DPRINT("Set frequency band index: %d", channel_id->channel_header.ch_freq_band);

		/*
		switch(channel_id->channel_header.ch_class)
		{
		case PHY_CLASS_LO_RATE:
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPFF_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPINT_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT3_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT2_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT1_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DATA_RATE_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_TX_NCO_MODE_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PA_TC_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_WAIT_LR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_SPIKE_DET_LR);

			break;
		case PHY_CLASS_NORMAL_RATE:
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPFF_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPINT_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT3_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT2_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT1_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DATA_RATE_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_TX_NCO_MODE_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PA_TC_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_WAIT_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_SPIKE_DET_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG1_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_OSR_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GAIN_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_LIMITER_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_RFPD_DECAY_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_IFPD_DECAY_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_EYE_NR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_QUAL_NR);

			break;
		case PHY_CLASS_HI_RATE:
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPFF_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_PFDCP_CPINT_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT3_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT2_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNTH_LPFILT1_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_TX_NCO_MODE_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PA_TC_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_WAIT_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_SPIKE_DET_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG1_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_OSR_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GAIN_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_LIMITER_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_RFPD_DECAY_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_IFPD_DECAY_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_EYE_HR);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_QUAL_HR);
			//assert(false);
			break;
		default:
			break;
		}
		*/

		// TODO validate
		switch(channel_id->channel_header.ch_freq_band)
		{
		// TODO calculate depending on rate and channr
		case PHY_BAND_433:
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CLKGEN_BAND_433);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_VCOCNT_RX_ADJ_433);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_IF_FREQ_433);

			if(channel_id->channel_header.ch_class == PHY_CLASS_LO_RATE)
			{
				assert(channel_id->center_freq_index <= 68);
				ez_channel_id = channel_id->center_freq_index;
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG1_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_OSR_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GAIN_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_LIMITER_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_RFPD_DECAY_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_IFPD_DECAY_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_EYE_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_QUAL_433_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_433_LR);

			}
			else if(channel_id->channel_header.ch_class == PHY_CLASS_NORMAL_RATE)
			{
				assert(channel_id->center_freq_index % 8 == 0 && channel_id->center_freq_index <= 270);
				ez_channel_id = channel_id->center_freq_index / 8;

				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_433_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_433_NR);
			}
			else if(channel_id->channel_header.ch_class == PHY_CLASS_HI_RATE)
			{
				assert(channel_id->center_freq_index % 8 == 0 && channel_id->center_freq_index <= 56);
				ez_channel_id = channel_id->center_freq_index / 8;
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DATA_RATE_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_433_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_433_HR);
			}

			DPRINT("Set channel freq index: %d", channel_id->center_freq_index);
			break;
		case PHY_BAND_868:
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CLKGEN_BAND_868);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_VCOCNT_RX_ADJ_868);
			ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_IF_FREQ_868);
			if(channel_id->channel_header.ch_class == PHY_CLASS_LO_RATE)
			{
				assert(channel_id->center_freq_index <= 279);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_868_LR);

				ez_channel_id = channel_id->center_freq_index % 255;
				if ((uint8_t) (channel_id->center_freq_index / 255) == 0)
				{
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_868_LR_01);
				} else
				{
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_868_LR_02);
				}

				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DECIMATION_CFG1_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_OSR_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_GAIN_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_LIMITER_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_RFPD_DECAY_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AGC_IFPD_DECAY_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_RAW_EYE_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DSA_QUAL_868_LR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_868_LR);
			}
			else if(channel_id->channel_header.ch_class == PHY_CLASS_NORMAL_RATE)
			{
				assert(channel_id->center_freq_index % 8 == 0 && channel_id->center_freq_index <= 270);
				ez_channel_id = channel_id->center_freq_index / 8;

				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_868_NR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_868_NR);
			} else {
				assert(channel_id->center_freq_index % 8 == 0 && channel_id->center_freq_index <= 270);
				ez_channel_id = channel_id->center_freq_index / 8;

				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_DATA_RATE_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_FREQ_DEV_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_BCR_NCO_OFFSET_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_AFC_GAIN_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_0_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE_1_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_0_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE_1_868_HR);
				ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_868_HR);
			}

			DPRINT("Set channel freq index: %d", channel_id->center_freq_index);
			break;
		case PHY_BAND_915:
			//ez_channel_id = channel_id->center_freq_index / 8;
			// Assume all other properties set in rfd900x_si4460_interface/ezradioInit()
			break;
		}
	} else 	if (channel_id->center_freq_index != current_channel_id.center_freq_index)
	{
		DPRINT("configure center_freq_index: %d", channel_id->center_freq_index);
		switch(channel_id->channel_header.ch_freq_band)
		{
		// TODO calculate depending on rate and channr
		case PHY_BAND_433:
			if(channel_id->channel_header.ch_class == PHY_CLASS_LO_RATE)
			{
				assert(channel_id->center_freq_index <= 68);
				ez_channel_id = channel_id->center_freq_index;

			}
			else
			{
				assert(channel_id->center_freq_index % 8 == 0 && channel_id->center_freq_index <= 56);
				ez_channel_id = channel_id->center_freq_index / 8;
			}

			DPRINT("Set channel freq index: %d", channel_id->center_freq_index);
			break;
		case PHY_BAND_868:
			if(channel_id->channel_header.ch_class == PHY_CLASS_LO_RATE)
			{
				assert(channel_id->center_freq_index <= 279);
				ez_channel_id = channel_id->center_freq_index % 255;
				if ((uint8_t) (channel_id->center_freq_index / 255) == 0)
				{
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_868_LR_01);
				} else
				{
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_FREQ_CONTROL_FRAC_868_LR_02);
				}

			}
			else  {
				assert(channel_id->center_freq_index % 8 == 0 && channel_id->center_freq_index <= 272);
				ez_channel_id = channel_id->center_freq_index / 8;
			}

			DPRINT("Set channel freq index: %d", channel_id->center_freq_index);
			break;
		case PHY_BAND_915:
			//ez_channel_id = channel_id->center_freq_index / 8;
			// Assume all other properties set in ezradioInit()
			break;
		}

		current_channel_id.center_freq_index = channel_id->center_freq_index;
	}
}


static void configure_eirp(const eirp_t eirp)
{
	if (eirp == current_eirp)
		return;
	eirp_t  pow = eirp;

	DPRINT("configure_eirp not implemented using max");
	if(pow < 0){pow=0;}                                                           // min 0
	else if(pow > 30){pow=30;}                                                     // max 30
	radio_set_transmit_power(pow);                                                // set tx power
	//uint8_t ddac = 0x7F; // max
	//ezradio_set_property(0x22, 0x01, 0x01, ddac);
	current_eirp = pow;
}

static void configure_syncword_class(syncword_class_t syncword_class, phy_coding_t coding)
{
	if(syncword_class == current_syncword_class)
		return;

	current_syncword_class = syncword_class;
	switch (coding)
	{
		case PHY_CODING_PN9:
			switch (syncword_class)
			{
				case PHY_SYNCWORD_CLASS0:
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNC_BITS_CS0_0);
					break;
				case PHY_SYNCWORD_CLASS1:
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNC_BITS_CS0_1);
					break;
			}
			break;
		case PHY_CODING_FEC_PN9:
			switch (syncword_class)
			{
				case PHY_SYNCWORD_CLASS0:
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNC_BITS_CS1_0);
					break;
				case PHY_SYNCWORD_CLASS1:
					ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_SYNC_BITS_CS1_1);
					break;
			}
			break;
	}
}

static void switch_to_idle_mode()
{
	if (current_state == HW_RADIO_STATE_IDLE)
		return;

	ezradio_change_state(EZRADIO_CMD_CHANGE_STATE_ARG_NEXT_STATE1_NEW_STATE_ENUM_SLEEP);
	current_state = HW_RADIO_STATE_IDLE;
}

error_t hw_radio_init(alloc_packet_callback_t alloc_packet_cb,
                      release_packet_callback_t release_packet_cb)
{
	/* EZRadio response structure union */
	ezradio_cmd_reply_t ezradioReply;

	alloc_packet_callback = alloc_packet_cb;
	release_packet_callback = release_packet_cb;

	current_state = HW_RADIO_STATE_UNKOWN;


	/* Initialize EZRadio device. */
	DPRINT("INIT ezradioInit");
	ezradioInit(&ezradio_int_callback);

	/* Set Tx power */
	radio_set_transmit_power(current_eirp);

	/* Print EZRadio device number. */
	DPRINT("INIT ezradio_part_info");
	ezradio_part_info(&ezradioReply);
	DPRINT("   Device: Si%04x\n\n", ezradioReply.PART_INFO.PART);

	/* Enable Packet Trace Interface */
	//ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_PTI_ENABLE);

	/* Fix registers not correct set by radio configurator */
	// latch on sync word detect - currently no threshold comparison (todo: optimize)
	//ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_CONTROL);

	// disable jump detection (todo: optimize)
	//ezradio_set_property(RADIO_CONFIG_SET_PROPERTY_MODEM_RSSI_CONTROL2);

	/* Reset radio fifos. */
	DPRINT("INIT ezradioResetTRxFifo");
	ezradioResetTRxFifo();



	// configure default channel, eirp and syncword
	configure_channel(&current_channel_id);
	configure_eirp(current_eirp);
	configure_syncword_class(current_rx_cfg.syncword_class, current_channel_id.channel_header.ch_coding);

	sched_register_task((&report_rssi));

	switch_to_idle_mode();
}

error_t hw_radio_set_rx(hw_rx_cfg_t const* rx_cfg, rx_packet_callback_t rx_cb, rssi_valid_callback_t rssi_valid_cb)
{
	DPRINT("hw_radio_set_rx rx_cb %p rssi_valid_cb %p", rx_cb, rssi_valid_cb);
	if(rx_cb != NULL)
	{
		assert(alloc_packet_callback != NULL);
		assert(release_packet_callback != NULL);
	}

	// assert(rx_cb != NULL || rssi_valid_cb != NULL); // at least one callback should be valid

	// TODO error handling EINVAL, EOFF
	rx_packet_callback = rx_cb;
	rssi_valid_callback = rssi_valid_cb;

	// if we are currently transmitting wait until TX completed before entering RX
	// we return now and go into RX when TX is completed
	if(current_state == HW_RADIO_STATE_TX)
	{
		should_rx_after_tx_completed = true;

		return SUCCESS;
	} else {

	}

	if (rx_cfg != NULL)
	{

		start_rx(rx_cfg);
		memcpy(&current_rx_cfg, rx_cfg, sizeof(hw_rx_cfg_t));
	}
	else
	{
		start_rx(&current_rx_cfg);
	}


	return SUCCESS;
}


error_t hw_radio_send_packet(hw_radio_packet_t* packet, tx_packet_callback_t tx_cb)
{
	// TODO error handling EINVAL, ESIZE, EOFF
	if(current_state == HW_RADIO_STATE_TX)
		return EBUSY;

	uint8_t data_length = packet->length + 1;

	if (packet->tx_meta.tx_cfg.channel_id.channel_header.ch_coding == PHY_CODING_FEC_PN9)
	{

		DPRINT(LOG_STACK_PHY, "Original packet: %d", data_length);
		DPRINT_DATA(packet->data, data_length);

		data_length = fec_encode(packet->data, data_length);
		DPRINT(LOG_STACK_PHY, "Encoded packet: %d", data_length);
		DPRINT_DATA(packet->data, data_length);

	} else {
		if (has_hardware_crc)
			data_length -= 2;
	}

	tx_packet_callback = tx_cb;

	if(current_state == HW_RADIO_STATE_RX)
	{
		//pending_rx_cfg.channel_id = current_channel_id;
		//pending_rx_cfg.syncword_class = current_syncword_class;
		should_rx_after_tx_completed = true;
	}

	current_state = HW_RADIO_STATE_TX;
	current_packet = packet;


	DPRINT(LOG_STACK_PHY, "Data to TX Fifo:");
	DPRINT_DATA(packet->data, data_length);


	configure_channel((channel_id_t*)&(packet->tx_meta.tx_cfg.channel_id));
	configure_eirp(packet->tx_meta.tx_cfg.eirp);
	configure_syncword_class(current_packet->tx_meta.tx_cfg.syncword_class, current_packet->tx_meta.tx_cfg.channel_id.channel_header.ch_coding);

	DEBUG_TX_START();
	DEBUG_RX_END();

    // Turn on the power and low-noise amplifiers
    amp_on(0);
    amp_on(1);

	ezradioStartTx(packet, ez_channel_id, should_rx_after_tx_completed, data_length);
	return SUCCESS;
}

int16_t hw_radio_get_rssi()
{
	ezradio_cmd_reply_t ezradioReply;
	ezradio_get_modem_status(0, &ezradioReply);

	//return -120;

//	DPRINT("CURR_RSSI    %d", convert_rssi(ezradioReply.GET_MODEM_STATUS.CURR_RSSI));
//	DPRINT("LATCH_RSSI    %d", convert_rssi(ezradioReply.GET_MODEM_STATUS.LATCH_RSSI));
//	DPRINT("ANT1_RSSI    %d", convert_rssi(ezradioReply.GET_MODEM_STATUS.ANT1_RSSI));
//	DPRINT("ANT2_RSSI    %d", convert_rssi(ezradioReply.GET_MODEM_STATUS.ANT2_RSSI));
//	ezradio_frr_d_read(1, &ezradioReply);
//	DPRINT("FRR_D_VALUE    %d", convert_rssi(ezradioReply.FRR_D_READ.FRR_D_VALUE));

	uint8_t rss = ezradioReply.GET_MODEM_STATUS.CURR_RSSI > ezradioReply.GET_MODEM_STATUS.LATCH_RSSI ? ezradioReply.GET_MODEM_STATUS.CURR_RSSI : ezradioReply.GET_MODEM_STATUS.LATCH_RSSI;
	return convert_rssi(rss);
}

int16_t hw_radio_get_latched_rssi()
{
	ezradio_cmd_reply_t ezradioReply;
	//ezradio_get_modem_status(0, &ezradioReply);
	//return convert_rssi(ezradioReply.GET_MODEM_STATUS.LATCH_RSSI);

	ezradio_frr_d_read(1, &ezradioReply);
	return convert_rssi(ezradioReply.FRR_D_READ.FRR_D_VALUE);
}

error_t hw_radio_set_idle()
{
	// if we are currently transmitting wait until TX completed before entering IDLE
	// we return now and go into IDLE when TX is completed
	if(current_state == HW_RADIO_STATE_TX)
	{
	  should_rx_after_tx_completed = false;
	  return SUCCESS;
	}

	switch_to_idle_mode();
	return SUCCESS;
}

error_t hw_radio_poweroff()
{
	ezradio_hal_AssertShutdown();
	current_state = HW_RADIO_STATE_OFF;
	return SUCCESS;
}

static void report_rssi()
{
	int16_t rss = hw_radio_get_rssi();
	rssi_valid_callback(rss);
}

static void start_rx(hw_rx_cfg_t const* rx_cfg)
{
	DPRINT("start_rx");

	if (current_state == HW_RADIO_STATE_OFF)
		ezradio_hal_DeassertShutdown();

    current_state = HW_RADIO_STATE_RX;

    configure_channel(&(rx_cfg->channel_id));
    configure_syncword_class(rx_cfg->syncword_class, rx_cfg->channel_id.channel_header.ch_coding);

    // Turn off the power and low-noise amplifiers
    amp_off(0);
    amp_off(1);

    rx_fifo_data_lenght = 0;
    if (rx_cfg->channel_id.channel_header.ch_coding == PHY_CODING_FEC_PN9)
    {
    	ezradioStartRx(ez_channel_id, false);
    } else {

    	ezradioStartRx(ez_channel_id, true);
    }

    DEBUG_RX_START();


    if(rssi_valid_callback != 0)
    {
    	timer_post_task_delay(&report_rssi, TIMER_TICKS_PER_SEC / 5000);
    }
}

static inline int16_t convert_rssi(uint8_t rssi_raw)
{
	return ((int16_t)(rssi_raw >> 1)) - (70 + RSSI_OFFSET);
}

static void ezradio_handle_end_of_packet()
{
	// fill rx_meta
	rx_packet->rx_meta.rssi = hw_radio_get_latched_rssi();
	rx_packet->rx_meta.lqi = 0;
	memcpy(&(rx_packet->rx_meta.rx_cfg.channel_id), &current_channel_id, sizeof(channel_id_t));
	if (has_hardware_crc && current_rx_cfg.channel_id.channel_header.ch_coding != PHY_CODING_FEC_PN9)
		rx_packet->rx_meta.crc_status = HW_CRC_VALID;
	else
		rx_packet->rx_meta.crc_status = HW_CRC_UNAVAILABLE;

	rx_packet->rx_meta.timestamp = timer_get_counter_value();
	//memcpy((void*)rx_packet->rx_meta.rx_cfg, (void*)&current_rx_cfg, sizeof(hw_rx_metadata_t));

	ezradio_fifo_info(EZRADIO_CMD_FIFO_INFO_ARG_FIFO_RX_BIT, NULL);


	DPRINT_DATA(rx_packet->data, rx_packet->length+1);


	if (current_rx_cfg.channel_id.channel_header.ch_coding == PHY_CODING_FEC_PN9)
	{
		fec_decode_packet(rx_packet->data, rx_packet->length, rx_packet->length);
		//assert length and data[0] can only differ 1
		rx_packet->length =  rx_packet->data[0];
	}

	DPRINT_PACKET(rx_packet, false);


	DEBUG_RX_END();

//					if(rx_packet_callback != NULL) // TODO this can happen while doing CCA but we should not be interrupting here (disable packet handler?)
	rx_packet_callback(rx_packet);
//					else
//						release_packet_callback(packet);

	if(current_state == HW_RADIO_STATE_RX)
	{
		start_rx(&current_rx_cfg);
	}
}

static void ezradio_int_callback()
{
	//DPRINT("ezradio ISR");

	ezradio_cmd_reply_t ezradioReply;
	ezradio_cmd_reply_t radioReplyLocal;
	ezradio_get_int_status(0x0, 0x0, 0x0, &ezradioReply);
	//ezradio_frr_a_read(3, &ezradioReply);

	//DPRINT(" - INT_PEND     %s", byte_to_binary(ezradioReply.FRR_A_READ.FRR_A_VALUE));
//	DPRINT(" - INT_PEND     %s", byte_to_binary(ezradioReply.GET_INT_STATUS.INT_PEND));
//	DPRINT(" - INT_STATUS   %s", byte_to_binary(ezradioReply.GET_INT_STATUS.INT_STATUS));
//	DPRINT(" - PH_PEND      %s", byte_to_binary(ezradioReply.GET_INT_STATUS.PH_PEND));
	//DPRINT(" - PH_STATUS    %s", byte_to_binary(ezradioReply.GET_INT_STATUS.PH_STATUS));
	//DPRINT(" - PH_STATUS    %s", byte_to_binary(ezradioReply.FRR_A_READ.FRR_B_VALUE));
//	DPRINT(" - MODEM_PEND   %s", byte_to_binary(ezradioReply.GET_INT_STATUS.MODEM_PEND));
//	DPRINT(" - MODEM_STATUS %s", byte_to_binary(ezradioReply.GET_INT_STATUS.MODEM_STATUS));
//	DPRINT(" - CHIP_PEND    %s", byte_to_binary(ezradioReply.GET_INT_STATUS.CHIP_PEND));
//	DPRINT(" - CHIP_STATUS  %s", byte_to_binary(ezradioReply.GET_INT_STATUS.CHIP_STATUS));

	//if (ezradioReply.FRR_A_READ.FRR_A_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_INT_PEND_PH_INT_PEND_BIT)
	if (ezradioReply.GET_INT_STATUS.INT_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_INT_PEND_PH_INT_PEND_BIT)
	{
		//DPRINT("PH ISR");
		switch(current_state)
		{
			case HW_RADIO_STATE_RX:
				if (ezradioReply.GET_INT_STATUS.PH_STATUS & EZRADIO_CMD_GET_INT_STATUS_REP_PH_STATUS_PACKET_RX_BIT)
				//if (ezradioReply.FRR_A_READ.FRR_B_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_PH_STATUS_PACKET_RX_BIT)
				{
					DPRINT("PACKET_RX IRQ");

					if(rx_packet_callback != NULL)
					{
						/* Check how many bytes we received. */
						ezradio_fifo_info(0, &radioReplyLocal);

						DPRINT("RX ISR packetLength: %d", radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT);

						//ezradio_cmd_reply_t radioReplyLocal2;
						//ezradio_get_packet_info(0, 0, 0, &radioReplyLocal2);

						if (rx_fifo_data_lenght == 0)
						{
							rx_packet = alloc_packet_callback(radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT);
						}

			//            hw_radio_packet_t* packet = alloc_packet_callback(radioReplyLocal2.PACKET_INFO.LENGTH);
			//            packet->length = radioReplyLocal2.PACKET_INFO.LENGTH;
						/* Read out the RX FIFO content. */
						ezradio_read_rx_fifo(radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT, &(rx_packet->data[rx_fifo_data_lenght]));
						//ezradio_read_rx_fifo(radioReplyLocal2.PACKET_INFO.LENGTH, packet->data);

						ezradio_handle_end_of_packet();

					}
				} else if (ezradioReply.GET_INT_STATUS.PH_STATUS & EZRADIO_CMD_GET_INT_STATUS_REP_PH_STATUS_RX_FIFO_ALMOST_FULL_BIT)
						//} else if (ezradioReply.FRR_A_READ.FRR_B_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_PH_STATUS_RX_FIFO_ALMOST_FULL_BIT)
					{
						//DPRINT("- RX FIFO almost full IRQ");

						ezradio_fifo_info(0, &radioReplyLocal);
						if (rx_fifo_data_lenght == 0)
						{
							DPRINT("RX FIFO: %d", radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT);
							uint8_t buffer[4];
							ezradio_read_rx_fifo(4, buffer);
							uint8_t length = buffer[0];
							if (current_rx_cfg.channel_id.channel_header.ch_coding == PHY_CODING_FEC_PN9)
							{
								uint8_t fec_buffer[4];
								memcpy(fec_buffer, buffer, 4);
								fec_decode_packet(fec_buffer, 4, 4);
								length = fec_calculated_decoded_length(fec_buffer[0]+1);
								DPRINT("RX Packet Length: %d / %d", fec_buffer[0], length);
							}
							rx_packet = alloc_packet_callback(length+1);
							rx_packet->length = length;
							memcpy(rx_packet->data, buffer, 4);
							rx_fifo_data_lenght += 4;
							radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT-=4;
						}
						while (radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT > 0)
						{
							DPRINT("RX FIFO: %d", radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT);
							/* Read out the FIFO Count bytes of RX FIFO */
							ezradio_read_rx_fifo(radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT, &(rx_packet->data[rx_fifo_data_lenght]));
							rx_fifo_data_lenght += radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT;
							//DPRINT("%d of %d bytes collected", rx_fifo_data_lenght, rx_packet->data[0]+1);
							ezradio_fifo_info(0, &radioReplyLocal);

							if (rx_fifo_data_lenght >= rx_packet->length + 1)
							{
								ezradio_handle_end_of_packet();
								return;
							}
						}

						ezradio_int_callback();

						return;
				} else if (ezradioReply.GET_INT_STATUS.PH_STATUS & EZRADIO_CMD_GET_INT_STATUS_REP_PH_STATUS_CRC_ERROR_BIT)

				//} else if ( ezradioReply.FRR_A_READ.FRR_B_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_PH_STATUS_CRC_ERROR_BIT)
				{
					DPRINT("- PACKET_RX CRC_ERROR IRQ");
					/* Check how many bytes we received. */
					ezradio_fifo_info(0, &radioReplyLocal);
					DPRINT("RX ISR packetLength: %d", radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT);

					//ezradio_cmd_reply_t radioReplyLocal2;
					//ezradio_get_packet_info(0, 0, 0, &radioReplyLocal2);

					if (rx_fifo_data_lenght == 0)
					{
						rx_packet = alloc_packet_callback(radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT);
					}

					/* Read out the RX FIFO content. */
					ezradio_read_rx_fifo(radioReplyLocal.FIFO_INFO.RX_FIFO_COUNT, &(rx_packet->data[rx_fifo_data_lenght]));

					rx_packet->rx_meta.crc_status = HW_CRC_INVALID;
					rx_packet->length = rx_packet->data[0] + 1;


					DPRINT_PACKET(rx_packet, false);


					DEBUG_RX_END();

					if(rx_packet_callback != NULL)
						rx_packet_callback(rx_packet);
					else
						release_packet_callback(rx_packet);

					if(current_state == HW_RADIO_STATE_RX)
					{
						start_rx(&current_rx_cfg);
					}

				} else {
					DPRINT((" - OTHER RX IRQ"));
				}



				break;
			case HW_RADIO_STATE_TX:
				if (ezradioReply.GET_INT_STATUS.PH_PEND & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
				//if (ezradioReply.FRR_A_READ.FRR_C_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_PACKET_SENT_PEND_BIT)
				{
					DPRINT("PACKET_SENT IRQ");

					DEBUG_TX_END();
					if(!should_rx_after_tx_completed)
						switch_to_idle_mode();

					current_packet->tx_meta.timestamp = timer_get_counter_value();


					DPRINT_PACKET(current_packet, true);

					if(tx_packet_callback != 0)
						tx_packet_callback(current_packet);

					if(should_rx_after_tx_completed)
					{
						// RX requested while still in TX ...
						// TODO this could probably be further optimized by not going into IDLE
						// after RX by setting TXOFF_MODE to RX (if the cfg is the same at least)
						should_rx_after_tx_completed = false;
						start_rx(&current_rx_cfg);
					}
//				} else if (ezradioReply.FRR_A_READ.FRR_C_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_PH_PEND_TX_FIFO_ALMOST_EMPTY_PEND_BIT)
//				{
//					DPRINT(" - TX FIFO Almost empty IRQ ");
//
//					ezradio_fifo_info(0, &radioReplyLocal);
//					DPRINT("TX FIFO Space: %d", radioReplyLocal.FIFO_INFO.TX_FIFO_SPACE);
//
//					//Fill fifo
//					#ifdef HAL_RADIO_USE_HW_CRC
//						int16_t new_length = current_packet->length-1 - tx_fifo_data_length;
//					#else
//						int16_t new_length = current_packet->length+1 - tx_fifo_data_length;
//					#endif
//					if (new_length > radioReplyLocal.FIFO_INFO.TX_FIFO_SPACE) new_length = radioReplyLocal.FIFO_INFO.TX_FIFO_SPACE;
//
//					while (new_length > 0)
//					{
//						ezradio_write_tx_fifo(new_length, &(current_packet->data[tx_fifo_data_length]));
//						tx_fifo_data_length += new_length;
//						DPRINT ("%d added -> %d", new_length, tx_fifo_data_length);
//
//					#ifdef HAL_RADIO_USE_HW_CRC
//						new_length = current_packet->length-1 - tx_fifo_data_length;
//					#else
//						new_length = current_packet->length+1 - tx_fifo_data_length;
//					#endif
//						if (new_length > radioReplyLocal.FIFO_INFO.TX_FIFO_SPACE) new_length = radioReplyLocal.FIFO_INFO.TX_FIFO_SPACE;
//					}
//
//					if (new_length == 0)
//					{
//						DPRINT("reprocess callback");
//						ezradio_int_callback();
//						return;
//					}
//
//					DPRINT ("%d added -> %d", new_length, tx_fifo_data_length);
				} else {
					DPRINT(" - OTHER IRQ");
				}
				break;
			default:
				//assert(false);
				DPRINT("State: %d", current_state);
		}
	}

//	if (ezradioReply.FRR_A_READ.FRR_A_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_INT_PEND_MODEM_INT_PEND_BIT)
//	{
//		DPRINT("MODEM ISR");
//	}

//	if (ezradioReply.FRR_A_READ.FRR_A_VALUE & EZRADIO_CMD_GET_INT_STATUS_REP_INT_PEND_CHIP_INT_PEND_BIT)
//	{
//		DPRINT("CHIP ISR");
//
//		if (current_state != HW_RADIO_STATE_IDLE)
//		{
//			if (ezradioReply.GET_INT_STATUS.CHIP_STATUS & EZRADIO_CMD_GET_INT_STATUS_REP_CHIP_STATUS_STATE_CHANGE_BIT)
//			{
//				ezradio_request_device_state(&radioReplyLocal);
//				DPRINT(" - Current State %d", radioReplyLocal.REQUEST_DEVICE_STATE.CURR_STATE);
//				DPRINT(" - Current channel %d", radioReplyLocal.REQUEST_DEVICE_STATE.CURRENT_CHANNEL);
//			}else {
//				DPRINT(" - OTHER IRQ");
//			}
//		}
//	}


}

