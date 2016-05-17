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

/*! \file rfd900x_pwm.c
 *
 *  Pulse width modulator for the RFD900X power amplifier.
 *
 *  \author simon@rallysafe.com.au
 */

#include "platform.h"
#include <stdbool.h>
#include <em_device.h>
#include <em_cmu.h>
#include <em_gpio.h>
#include <em_system.h>
#include <em_timer.h>

#include "rfd900x_pwm.h"

// Translate platform definitions
#if PWM_CCX == 2
#define PWMTIMER_ROUTE_CCXPEN TIMER_ROUTE_CC2PEN
#else
#error "Compare/Capture register must be defined"
#endif

#if PWM_LOCATION == 1
#define PWMTIMER_ROUTE_LOCATION TIMER_ROUTE_LOCATION_LOC1
#else
#error "PWM location must be defined"
#endif

void __rfd900x_pwm_init(void)
{
	CMU_ClockEnable(PWM_CLOCK, true);
	GPIO_PinModeSet(PWM_PORT, PWM_PIN, gpioModePushPull, 0);
	uint32_t Top;
	TIMER_InitCC_TypeDef timerCCInit =
	{
		.eventCtrl  = timerEventEveryEdge,
		.edge       = timerEdgeBoth,
		.prsSel     = 0,
		.cufoa      = timerOutputActionNone,
		.cofoa      = timerOutputActionNone,
		.cmoa       = timerOutputActionToggle,
		.mode       = timerCCModePWM,
		.filter     = false,
		.prsInput   = false,
		.coist      = false,
		.outInvert  = false,
	};
	/* Configure CC channel 0 */
	TIMER_InitCC(PWM_TIMER, PWM_CCX, &timerCCInit);

	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale1,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};
	/* Route CC0 to location 3 (PD1) and enable pin */
	PWM_TIMER->ROUTE |= (PWMTIMER_ROUTE_CCXPEN | PWMTIMER_ROUTE_LOCATION);
	Top = 0xff;
	// Freq = (28000000UL/256) = 109375 := 100Khz
	TIMER_TopSet(PWM_TIMER, Top);
	TIMER_CompareSet(PWM_TIMER, PWM_CCX,0x00);			// set on time
	TIMER_Init(PWM_TIMER, &timerInit);
}

void pwm_set_duty(uint8_t Duty8Bit)
{
	TIMER_CompareBufSet(PWM_TIMER, PWM_CCX,(~Duty8Bit)&0xFF);			// set on time, use buffered so no glitches
}
