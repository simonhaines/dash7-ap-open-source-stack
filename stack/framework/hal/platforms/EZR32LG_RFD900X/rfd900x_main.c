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

#include "scheduler.h"
#include "bootstrap.h"
#include "hwgpio.h"
#include "hwuart.h"
#include "hwleds.h"
#include "hwlcd.h"
#include "hwusb.h"
#include "ezr32lg_mcu.h"
#include "hwdebug.h"
#include "hwwatchdog.h"
#include "platform.h"
#include "platform_uart.h"
#include "em_gpio.h"
#include <debug.h>

#include "em_cmu.h"
#include "em_chip.h"

#include "rfd900x_mcu.h"
#include "rfd900x_amp.h"
#include "rfd900x_pwm.h"
#include "rfd900x_si4460.h"

// Not defined in ezr32lg_pins.h
extern pin_id_t const F3;


void SWO_SetupForPrint(void)
{
  /* Enable GPIO clock. */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_LEOPARD_FAMILY) || defined(_EFM32_WONDER_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) |GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  /* Wait until clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR  = 0xC5ACCE55;
  ITM->TER  = 0x0;
  ITM->TCR  = 0x0;
  TPI->SPPR = 2;
  TPI->ACPR = 0xf;
  ITM->TPR  = 0x0;
  DWT->CTRL = 0x400003FF;
  ITM->TCR  = 0x0001000D;
  TPI->FFCR = 0x00000100;
  ITM->TER  = 0x1;
}


void __platform_init()
{
	__rfd900x_mcu_init();     // Set clocking
    //__gpio_init();            // Start GPIO clock and enable interrupt
    //__led_init();             // Configure LEDs
    //__rfd900x_pwm_init();     // Start PWM clock and drive to pin
    //__rfd900x_amp_init();     // Configure low-noise and power amplifiers
    //__rfd900x_si4460_init();  // Initialise the transceiver

    //__hw_debug_init();        // Don't use any debug pins for now
    //__watchdog_init();        // Wake up the dog
}

void __platform_post_framework_init()
{
#ifdef PLATFORM_USE_SWO
    SWO_SetupForPrint();
#endif

    // The UART registers tasks, so needs the framework in place
    __uart_init();
}
void main2(void);
int main()
{
    // Only when using bootloader
	//SCB->VTOR=0x4000;
    main2();
    //initialise the platform itself
	__platform_init();
    //do not initialise the scheduler, this is done by __framework_bootstrap()
    __framework_bootstrap();
    //initialise platform functionality that depends on the framework
    __platform_post_framework_init();

    scheduler_run();
    return 0;
}


#include "em_timer.h"
#define TDMTIMER2                  TIMER2
#define TDMTIMER2_cmuClock cmuClock_TIMER2

uint16_t timer2_tick2(void)
{
  return(TIMER_CounterGet(TDMTIMER2));
}
#define TDMSHIFT 0
#define TDMFREQ (54687U*(1U<<TDMSHIFT))                                         // 16uS to match tdm calculations

#define usec2Ticks(usec) ((usec+(1000000UL/TDMFREQ)-1)/(1000000UL/TDMFREQ))

void timer_init2()                                                                 // initialise timers
{
  CMU_ClockDivSet(cmuClock_HFPER,cmuClkDiv_1);                                  // set divide for hfper clk
  CMU_ClockEnable(TDMTIMER2_cmuClock, true);                                    /* Enable clock for TDMTIMER2 module */

  TIMER_Init_TypeDef tdmtimerInit2 =               /* Select TDMTIMER parameters */
  {
    .enable     = true,
    .debugRun   = false,
    .prescale   = timerPrescale512,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  TIMER_TopSet(TDMTIMER2, 0xffff);            /* Set TIMER Top value */
  TIMER_Init(TDMTIMER2, &tdmtimerInit2);            /* Configure TIMER */

}

void USTIMER_Delay2( uint32_t usec )
{
  uint16_t Ticks = usec2Ticks(usec);
  uint16_t tickStart = timer2_tick2();
  while(((uint16_t)(timer2_tick2() - tickStart)) <Ticks);
}


#include "ezradio_cmd.h"
//#include "ezradio_plugin_manager.h"
#include "ezradio_api_lib.h"
#include "ezradio_api_lib_add.h"
#include "ezradio_hal.h"
#include "radio-config-2gfsk-64-96.h"

void ezradio_reset2(void)
{
    /* Put radio in shutdown */
    ezradio_hal_AssertShutdown();
    /* Delay for 20us */
    USTIMER_Delay2( 10u );
    /* Release radio from shutdown */
    ezradio_hal_DeassertShutdown();
    /* Delay for 100us */
    USTIMER_Delay2( 100u );
    /* Clear CTS signal. */
    ezradio_comm_ClearCTS();
    //GPIO_PinModeSet(PAEN_PORT, PAEN, gpioModePushPull, 0);
    //GPIO_PinModeSet(LNAEN_PORT, LNAEN  , gpioModePushPull, 1);
}

#define RADIO_CONFIG_DATA_RADIO_DELAY_AFTER_RESET_US (10000)
void ezradioPowerUp2(void)
{
  /* Hardware reset the chip */
  ezradio_reset2();
  /* Delay for preconfigured time */
  USTIMER_Delay2( RADIO_CONFIG_DATA_RADIO_DELAY_AFTER_RESET_US );
}
void DummyCB(uint8_t pin)
{

}

void ezradioInit2(const uint8_t *Radio_Config_Data)
{
  uint16_t wDelay;
  const uint8_t * Config_Data;
  Config_Data = Radio_Config_Data;
  /* Initialize radio GPIOs and SPI port */

  ezradio_hal_GpioInit(DummyCB, true );
  ezradio_hal_SpiInit();
  /* Power Up the radio chip */
  ezradioPowerUp2();
  /* Load radio configuration */
  while (EZRADIO_CONFIG_SUCCESS != ezradio_configuration_init(Config_Data))
  {
    for (wDelay = 0x7FFF; wDelay--; ) ;
    /* Power Up the radio chip */
    ezradioPowerUp2();
  }
  /* Read ITs, clear pending ones */
  //ezradio_get_int_status(0u, 0u, 0u, NULL);
}

#include "915_HR.h"
void main2(void)
{
#define LED_PORT gpioPortF
#define GREENLED 10
#define REDLED   11
  CMU_OscillatorEnable(cmuOsc_HFRCO,true,true);
  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  CMU_ClockDivSet(cmuClock_HF, cmuClkDiv_1);
  //CMU_ClockEnable(cmuClock_HF, true);
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);

  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_DriveModeSet(LED_PORT, gpioDriveModeHigh);
  GPIO_PinModeSet(LED_PORT, GREENLED, gpioModePushPullDrive, 0);
  GPIO_PinModeSet(LED_PORT, REDLED  , gpioModePushPullDrive, 0);
  /* Initialize GPIO interrupt */
  GPIOINT_Init();

  /* Enable GPIO Clock. */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;
  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;
  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));
  /* Enable trace in core debug */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR  = 0xC5ACCE55;
    ITM->TER  = 0x0;
    ITM->TCR  = 0x0;
    TPI->SPPR = 2;
    TPI->ACPR = 0xf;
    ITM->TPR  = 0x0;
    DWT->CTRL = 0x400003FE;
    ITM->TCR  = 0x0001000D;
    TPI->FFCR = 0x00000100;
    ITM->TER  = 0x1;

    timer_init2();                                                                 // initialise timers
    static const uint8_t Radio_Configuration_Data_Array_2G6496[] = RADIO_CONFIGURATION_DATA_ARRAY;//RADIO_2G6496_CONFIGURATION_DATA_ARRAY;
    static ezradio_cmd_reply_t ezradioReply;

    ezradioInit2(Radio_Configuration_Data_Array_2G6496);
    ezradio_part_info(&ezradioReply);
}
