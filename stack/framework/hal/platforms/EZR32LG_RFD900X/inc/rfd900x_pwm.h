/*
 * rfd900x_pwm.h
 *
 *  Created on: 13/05/2016
 *      Author: simon
 */

#ifndef RFD900X_PWM_H_
#define RFD900X_PWM_H_

#include "link_c.h"
#include "types.h"


/* \brief Set the duty cycle of the PWM driver.
 *
 * \param  duty		The duty cycle (0 - 222 where 255 = 100%).
 */
__LINK_C void pwm_set_duty(uint8_t duty);

/* Not a user function */
__LINK_C void __rfd900x_pwm_init(void);


#endif /* RFD900X_PWM_H_ */
