/*
 * rfd900x_amp.c
 *
 *  Created on: 17/05/2016
 *      Author: simon
 */

#include <debug.h>
#include "types.h"
#include "platform.h"
#include "em_gpio.h"
#include "ezr32lg_pins.h"
#include "rfd900x_amp.h"

// Not defined in ezr32lg_pins.h (for some reason)
extern pin_id_t const F3;

#if HW_NUM_AMPS != 2
	#error HW_NUM_AMPS does not match the expected value. Update platform.h or rfd900x_amp.c
#endif
static pin_id_t amps[HW_NUM_AMPS];

void __rfd900x_amp_init(void)
{
	amps[0] = AMP_PA;
	amps[1] = AMP_LNA;
	for(int i = 0; i < HW_NUM_AMPS; i++)
	{
		error_t err = hw_gpio_configure_pin(amps[i], false, gpioModePushPull, 0);
		assert(err == SUCCESS);
	}
}

void amp_on(unsigned char amp_no)
{
    if(amp_no < HW_NUM_AMPS)
    	hw_gpio_set(amps[amp_no]);
}

void amp_off(unsigned char amp_no)
{
    if(amp_no < HW_NUM_AMPS)
    	hw_gpio_clr(amps[amp_no]);
}
