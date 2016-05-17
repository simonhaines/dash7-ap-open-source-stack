/*
 * rfd900x_mcu.c
 *
 *  Created on: 17/05/2016
 *      Author: simon
 */

#include "em_cmu.h"
#include "em_chip.h"
#include "platform.h"

void __rfd900x_mcu_init()
{
	  CHIP_Init();

	  /* HFXO 48MHz, divided by 1, rco is only 28Mhz   TODO , calibrate RCO from radio XO??*/
	  // calibration tables exist in ROM , data sheet has temp comp curves
	  // use this info to calibrate on the fly
	  CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
	  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
	  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

	  /* NOTE! The following line causes an assert violation.
	   * By default, Dash7 is compiled with DEBUG_EFM set,
	   * whereas the SiK project does not set this.
	   */
	  //CMU_ClockEnable(cmuClock_HF, true);

	  CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_1);
	  CMU_ClockEnable(cmuClock_HFPER, true);
	  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
}
