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

#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include "platform_defs.h"

#ifndef PLATFORM_EZR32LG_RFD900X
    #error Mismatch between the configured platform and the actual platform. Expected PLATFORM_EZR32LG_RFD900X to be defined
#endif

#include <ezr32lg_chip.h>


/**************************
 * OSCILLATOR DEFINITIONS *
 **************************/
// The RFD900x doesn't have any on-board crystals
#undef HW_USE_HFXO
#undef HW_USE_LFXO


/*******************
 * LED DEFINITIONS *
 *******************/
// Red LED is GPIO F.11, green LED is GPIO F.10
#define HW_NUM_LEDS  2
#define LED0         F10
#define	LED1         F11


/***********************
 * CONSOLE DEFINITIONS *
 ***********************/
#define NO_CONSOLE
#undef  CONSOLE_UART


/********************
 * UART DEFINITIONS *
 ********************/
// Use USART 1, location 2
#define UART_NUM       2
#define UART_LOCATION  2
#define UART_BAUDRATE  PLATFORM_EZR32LG_RFD900X_UART_BAUDRATE


/**********************
 * SPI RF DEFINITIONS *
 **********************/
#define si4455_GDO0_PIN  A15
#define si4455_GDO1_PIN  E14
#define si4455_SDN_PIN   E8


/*************************
 * DEBUG PIN DEFINITIONS *
 *************************/
#define DEBUG_PIN_NUM  0
/* Define DEBUG0, DEBUG1... */


/**************************
 * USERBUTTON DEFINITIONS *
 **************************/
#define NUM_USERBUTTONS  0
/* Define BUTTON0, BUTTON1... */


/*******************
 * LCD DEFINITIONS *
 *******************/
#undef HAS_LCD


/*******************
 * PWM DEFINITIONS *
 *******************/
#define PWM_TIMER     TIMER3
#define PWM_CLOCK     cmuClock_TIMER3
#define PWM_PORT      gpioPortE
#define PWM_PIN       2
#define PWM_CCX       2
#define PWM_LOCATION  1


/*************************
 * AMPLIFIER DEFINITIONS *
 *************************/
#define HW_NUM_AMPS  2
#define AMP_PA       F3
#define AMP_LNA      F7


#endif
