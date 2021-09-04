#include "ch.h"

#define IRTX_LINE PAL_LINE(GPIOC, 1U)
#define IRRX_LINE PAL_LINE(GPIOA, 4U)
#define LED_LINE PAL_LINE(GPIOC,GPIOA_LED_GREEN)

static THD_WORKING_AREA(waThread1, 512);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  palSetLineMode(IRRX_LINE, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(IRTX_LINE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LED_LINE, PAL_MODE_OUTPUT_PUSHPULL);
  palClearLine(LED_LINE);

  //palSetLine(IRTX_LINE);

  while (true) {
    chThdSleepMilliseconds(10);
    if (palReadLine(IRRX_LINE)==1) {

         palSetLine(LED_LINE);

         //palSetLine(IRTX_LINE);
         chThdSleepMilliseconds(250);
         palClearLine(LED_LINE);
         chThdSleepMilliseconds(250);
       }
       chThdSleepMilliseconds(10);
     }
  }

