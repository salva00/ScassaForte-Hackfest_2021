/*
    NeaPolis Innovation Summer Campus 2021 Examples
    Copyright (C) 2021 Ciro Mazzocchi
    [ciro.mazzocchi@outlook.com]

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * Demo Test for PWM Driver.
 * PWM Signal, generated on PB4 / ARD D5 pin.
 */

#include "ch.h"
#include "hal.h"

#include "stdio.h"

static uint16_t open_flag = 0;

/*
 * PWM Driver Configuration.
 */
static void pwmPeriodCb(PWMDriver *pwmp){
  (void)pwmp;

  // This sets to HIGH the pin of green led
  palSetPad(GPIOA, GPIOA_LED_GREEN);
};

static void pwmPulseCb(PWMDriver *pwmp){
  (void) pwmp;

  palClearPad(GPIOA, GPIOA_LED_GREEN);
}

static PWMConfig pwmcfg = {
  10000,
  200,
  pwmPeriodCb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmPulseCb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

/*
 * Duty cycle switcher Thread.
 */
static THD_WORKING_AREA(waSwitcherThread, 128);
static THD_FUNCTION(SwitcherThread, arg) {
  (void)arg;
  chRegSetThreadName("switcher");

  // This would define out sequence
  int lock_status[] = {750,1200};

  while(true)
  {
    //In base allo stato del flag, decido se aprire o meno la cassaforte
      if(open_flag)
      {
        pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, lock_status[0]));
      }
      else
      {
        pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, lock_status[1]));
      }

      // This waits 1 second
      chThdSleepMilliseconds(1000);
  }
}

int main(void) {
  halInit();
  chSysInit();

  //Configurazione GPIOC13 (BLUE_BUTTON)
  palSetPadMode(GPIOC,GPIOC_BUTTON,PAL_MODE_INPUT);

  //LED di DEBUG
  palSetPadMode(GPIOA,GPIOA_ARD_D8,PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOA,GPIOA_ARD_D8);

  // It configures PWM related PIN.
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2));

  // It stars PWM driver.
  pwmStart(&PWMD3, &pwmcfg);

  // It enables the periodic callback at the beginning of period
  pwmEnablePeriodicNotification(&PWMD3);

  // It enables the periodic callback at the end of pulse
  pwmEnableChannelNotification(&PWMD3,0);

  // It creates the switcher's thread
  chThdCreateStatic(waSwitcherThread, sizeof(waSwitcherThread), NORMALPRIO+2, SwitcherThread, &open_flag);

  while (true) {

    if(!palReadPad(GPIOC,GPIOC_BUTTON))
    {
      while(!palReadPad(GPIOC,GPIOC_BUTTON))
      {
        open_flag = 1;
        palWritePad(GPIOA,GPIOA_ARD_D8,PAL_HIGH);
        chThdSleepMilliseconds(20);//Debouncing time
      }
    }
    open_flag = 0;
    palWritePad(GPIOA,GPIOA_ARD_D8,PAL_LOW);

    chThdSleepMilliseconds(1000);
  }
}