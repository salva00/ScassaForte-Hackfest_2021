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
#include "chprintf.h"
#include "ssd1306.h"


#define CODE 7
#define BUFF_SIZE 20

BaseSequentialStream * chp = (BaseSequentialStream*) &SD2;
static uint16_t open_flag = 0;

//*********SHELL working area*********//

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

//*********Numero canali ADC e numero misurazioni*********//

#define ADC_GRP1_NUM_CHANNELS       1
#define ADC_GRP1_BUF_DEPTH          8

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];


//*********ADC Driver configuration************//

/*size_t nx = 0, ny = 0;
static void adccallback(ADCDriver *adcp) {

  if (adcIsBufferComplete(adcp)) {
    nx += 1;
  }
  else {
    ny += 1;
  }
}*/

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}


static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,                            /* circular     : Circular buffer mode  */
  ADC_GRP1_NUM_CHANNELS,            /* num_channels : Number of channels    */
  NULL,                             /* end_cb       : End Callback          */
  adcerrorcallback,                 /* error_cb     : Error Callback        */
  0,                                /* CR1      */
  ADC_CR2_SWSTART,                  /* CR2      */
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3), /* SMPR1    */
  0,                                /* SMPR2    */
  0,                                /* HTR      */
  0,                                /* LTR      */
  0,                                /* SQR1     */
  0,                                /* SQR2     */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)  /* SQR3     */
};


//*********PWM Driver Configuration *********//

static PWMConfig pwmcfg = {
  10000,
  200,
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};


//**********ssd1306 configuration**********//

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};

static const SSD1306Config ssd1306cfg = {
  &I2CD1,
  &i2ccfg,
  SSD1306_SAD_0X78,
};

static SSD1306Driver SSD1306D1;

//*********TIMx configuration ICU *********//

static int rotation_number = 0;
char buff[BUFF_SIZE];

/* Callback called at the end of period*/
static void cbIcuPeriod(ICUDriver *icup) {
  (void)icup;

  if(!palReadPad(GPIOA,1U)) rotation_number += 1; //Clock wise --> increment counter
  else rotation_number -= 1;
}

/* Callback called at the end of pulse*/
/*static void cbIcuWidth(ICUDriver *icup) {
  last_width = icuGetWidthX(icup);                                              // Gets duration of last width in ticks
}*/

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,                                                        // ICU is configured on active level
  1000,                                                                         // Frequency is 1 kHz
  NULL,                                                                   // This callback is called when input signal pass from the high level to low level
  cbIcuPeriod,                                                                  // This callback is called when input signal pass from the low level to high level
  NULL,                                                                         // There is no callback when the counter goes in overflow
  ICU_CHANNEL_1,                                                                // It configures the first channel of ICU Driver
};


//*********SERVO control Thread************//

static THD_WORKING_AREA(waServoThread, 128);
static THD_FUNCTION(ServoThread, arg) {
  (void)arg;
  chRegSetThreadName("Servo");

  int lock_status[] = {750,1200};

  while(true)
  {
    //In base allo stato del flag di corretezza del codice, decido se aprire o meno la cassaforte
      if(open_flag)
      {
        pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, lock_status[0]));
      }
      else
      {
        pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, lock_status[1]));
      }

      // This waits 1 second
      chThdSleepMilliseconds(500);
  }
}


//*********Led RGB management by Photoresistor************//

static THD_WORKING_AREA(waPhotoResistorThread, 128);
static THD_FUNCTION(PhotoResistorThread, arg) {
  (void)arg;
  chRegSetThreadName("PhotoResistor");
  
  float ave = 0.0;
  
  adcStart(&ADCD1, NULL);

  while(true)
  {
    adcConvert(&ADCD1, &adcgrpcfg1, (adcsample_t*) samples1, ADC_GRP1_BUF_DEPTH);
    
    ave = 0;
    //calcolo il valor medio di tensione in ingresso e decido se la cassaforte è aperta o meno
    for(int i = 0;i < ADC_GRP1_BUF_DEPTH;++i)
    {
      ave += (float) samples1[i];
    }

    if(ave/ADC_GRP1_BUF_DEPTH <= 3600) //Locker aperto --> accendo LED Verde
    {
      palSetPad(GPIOA,8U);
      palClearPad(GPIOA,9U);
    }
    else           //Locker chiuso --> accendo LED Giallo e metto il flag di apertura porta a 0
    {
      palSetPad(GPIOA,8U);
      palSetPad(GPIOA,9U);
      open_flag = 0;
    }

      // This waits 1 second
      chThdSleepMilliseconds(500);
  }
}

//*********Inserimento codice PIN************//

static THD_WORKING_AREA(waCode, 128);
static THD_FUNCTION(Code, arg) {
  (void)arg;
  chRegSetThreadName("Code");

  //Start OLED
  ssd1306ObjectInit(&SSD1306D1);
  ssd1306Start(&SSD1306D1, &ssd1306cfg);
  ssd1306FillScreen(&SSD1306D1, 0x00);

  ssd1306GotoXy(&SSD1306D1, 0, 1);
  chsnprintf(buff, BUFF_SIZE, "ScassaForte");
  ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

  ssd1306GotoXy(&SSD1306D1, 0, 20);
  chsnprintf(buff, BUFF_SIZE, "Insert PIN");
  ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_7x10, SSD1306_COLOR_BLACK);

  ssd1306UpdateScreen(&SSD1306D1);

  while(true)
  {

    //Upoad Display
    ssd1306FillScreen(&SSD1306D1, 0x00);//pulizia dello schermo

    ssd1306GotoXy(&SSD1306D1,55,27);
    chsnprintf(buff, BUFF_SIZE, "%d",rotation_number);
    ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

    ssd1306UpdateScreen(&SSD1306D1);

    //if(!palReadPad(GPIOB,3U))
    if(!palReadPad(GPIOC,GPIOC_BUTTON))
    {
      while(!palReadPad(GPIOC,GPIOC_BUTTON))
      {
        chThdSleepMilliseconds(20);

      }
      switch (rotation_number)
      {
      case CODE:
        open_flag = 1;

        ssd1306FillScreen(&SSD1306D1, 0x00); //Pulizia dello schermo
        ssd1306GotoXy(&SSD1306D1,0,27);
        chsnprintf(buff, BUFF_SIZE, "Codice corretto");
        ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

         ssd1306UpdateScreen(&SSD1306D1);
        break;
      default:

        ssd1306FillScreen(&SSD1306D1, 0x00); //Pulizia dello schermo
        ssd1306GotoXy(&SSD1306D1,0,27);
        chsnprintf(buff, BUFF_SIZE, "Codice errato");
        ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
        ssd1306UpdateScreen(&SSD1306D1);

      }
      rotation_number = 0;

      /*if(rotation_number > 10)
      {
        open_flag = 1;
      }
      rotation_number = 0;*/
    }
      // This waits 1 second
      chThdSleepMilliseconds(500);
  }
}


int main(void) {
  halInit();
  chSysInit();

  sdStart(&SD2, NULL);

  //PIN SDA/SCL
  /* Configuring I2C related PINs */
     palSetLineMode(LINE_ARD_D15, PAL_MODE_ALTERNATE(4) |
                    PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUPDR_PULLUP);
     palSetLineMode(LINE_ARD_D14, PAL_MODE_ALTERNATE(4) |
                    PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |
                    PAL_STM32_PUPDR_PULLUP);


  //Collego A0 al TIM5
   palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(2));

   //Collego A1 per la direzione dell'encoder
   palSetPadMode(GPIOA, 1, PAL_MODE_INPUT);

   //Abilito ICU su TIM5
   icuStart(&ICUD5, &icucfg);
   icuStartCapture(&ICUD5);
   icuEnableNotifications(&ICUD5);

  //ADC connection --> PINC0
  palSetPadMode(GPIOC,0U,PAL_MODE_INPUT_ANALOG);

  //GPIOB3 connesso allo SW dell'encoder
  palSetPadMode(GPIOB,3U,PAL_MODE_INPUT);

  //Configurazione BLUE_BUTTON
  palSetPadMode(GPIOC,GPIOC_BUTTON,PAL_MODE_INPUT);

  //Configurazione LED RGB --> PA9,PA8,PB5
  palSetPadMode(GPIOB,5U,PAL_MODE_OUTPUT_PUSHPULL); //Blue
  palSetPadMode(GPIOA,8U,PAL_MODE_OUTPUT_PUSHPULL); //Green
  palSetPadMode(GPIOA,9U,PAL_MODE_OUTPUT_PUSHPULL); //Red

  palClearPad(GPIOB,5U);
  palClearPad(GPIOA,8U);
  palClearPad(GPIOA,9U);

  // It configures PWM related PIN.
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2));

  // It stars PWM driver.
  pwmStart(&PWMD3, &pwmcfg);

  // It enables the periodic callback at the beginning of period
  pwmEnablePeriodicNotification(&PWMD3);

  // It enables the periodic callback at the end of pulse
  pwmEnableChannelNotification(&PWMD3,0);



  //************Chiamate ai Thread**********//

  // Thread segnale PWM per controllo SERVO
  chThdCreateStatic(waServoThread, sizeof(waServoThread), NORMALPRIO+1, ServoThread, &open_flag);

  //Thread convertitore ADC per lettura fotoresistore
  chThdCreateStatic(waPhotoResistorThread, sizeof(waPhotoResistorThread), NORMALPRIO+1, PhotoResistorThread, NULL);

  //Thread convertitore ADC per lettura fotoresistore
  chThdCreateStatic(waCode, sizeof(waCode), NORMALPRIO+2, Code, NULL);


  while (true) {


    /*if(!palReadPad(GPIOC,GPIOC_BUTTON))
    {
      while(!palReadPad(GPIOC,GPIOC_BUTTON)) //Fino a quando il pulsante viene premuto apro la porta
      {
        open_flag = 1;
        chThdSleepMilliseconds(20);//Debouncing time
      }
    }
    open_flag = 0;*/

    chThdSleepMilliseconds(1000);
  }
  adcStop(&ADCD1);
}
