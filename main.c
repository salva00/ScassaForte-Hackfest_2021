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
#include "shell.h"
#include "images.c"

#define BUFF_SIZE 20
#define CODE_DIGIT 4

//**********DEFINE DEI VARI PIN USATI***********//

#define ENCODER_CLK_port GPIOA //ENCODER CLK per conteggio dei tick
#define ENCODER_CLK_pin 0U

#define ENCODER_DT_port GPIOA //ENCODER DT per analisi verso di rotazione
#define ENCODER_DT_pin 1U

#define RX_IR_port GPIOA //RX IR per controllo presenza oggetto
#define RX_IR_pin 4U

#define BUTTON_EXT_port GPIOA //BUTTON per conferma insrimento codice/digit
#define BUTTON_EXT_pin 5U

#define LED_TX_RX_port GPIOA //LED BLU per segnalare presenza
#define LED_TX_RX_pin 7U

#define LED_RGB_GREEN_port GPIOA //LED RGB, ramo LED VERDE
#define LED_RGB_GREEN_pin 8U

#define LED_RGB_RED_port GPIOA //LED RGB, ramo LED ROSSO
#define LED_RGB_RED_pin 9U

#define BUZZER_port GPIOA  //BUZZER per segnalare inserimento errato del codice e conferma del digit
#define BUZZER_pin 10U

#define LED_ERR_CODE_port GPIOB  //LED ROSSO per segnalare inserimento errato del codice
#define LED_ERR_CODE_pin 3U

#define SERVO_port GPIOB  //PWM per pilotaggio Servo motore
#define SERVO_pin 4U

#define LED_RGB_BLUE_port GPIOB  //LED RGB, ramo LED BLU
#define LED_RGB_BLUE_pin 5U

#define PHOTORESISTOR_port GPIOC //FOTORESISTORE per capire stato del locker da luce
#define PHOTORESISTOR_pin 0U

//*********SHELL working area*********//

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

//*********Numero canali ADC e numero misurazioni*********//

#define ADC_GRP1_NUM_CHANNELS       1
#define ADC_GRP1_BUF_DEPTH          8

BaseSequentialStream *chp = (BaseSequentialStream*)&SD2;

//***********VARIABILI*************//

static int CODE[4] = {0, 0, 0, 0}, INSERT_CODE[4] = {0, 0, 0, 0}, digit_count =
    0;
static uint16_t open_flag = 0;
static int error_count = 0;
static int rotation_number = 0;
int lock_box = 0, press_button_flag = 0;
int encoder_dt_status;
char buff[BUFF_SIZE];


static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

//*********ADC Driver configuration************//

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

static const ADCConversionGroup adcgrpcfg1 = {
 FALSE, /* circular     : Circular buffer mode  */
 ADC_GRP1_NUM_CHANNELS, /* num_channels : Number of channels    */
 NULL, /* end_cb       : End Callback          */
 adcerrorcallback, /* error_cb     : Error Callback        */
 0, /* CR1      */
 ADC_CR2_SWSTART, /* CR2      */
 ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3), /* SMPR1    */
 0, /* SMPR2    */
 0, /* HTR      */
 0, /* LTR      */
 0, /* SQR1     */
 0, /* SQR2     */
 ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10) /* SQR3     */
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
 0, 0};

//**********ssd1306 configuration**********//

static const I2CConfig i2ccfg = {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2, };

static const SSD1306Config ssd1306cfg = {&I2CD1, &i2ccfg, SSD1306_SAD_0X78, };

static SSD1306Driver SSD1306D1;

//*********TIMx configuration ICU *********//


/* Callback called at the end of period*/
static void cbIcuPeriod(ICUDriver *icup) {
  (void)icup;

  encoder_dt_status = palReadPad(ENCODER_DT_port, ENCODER_DT_pin);

  if (!encoder_dt_status)
    rotation_number += 1; //Clock wise --> increment counter

  else
    rotation_number -= 1; //Se l'encoder è a 0, ripartiamo da 9 (catena chiusa)

}

static ICUConfig icucfg = {ICU_INPUT_ACTIVE_HIGH, // ICU is configured on active level
    1000,                                                  // Frequency is 1 kHz
    NULL, // This callback is called when input signal pass from the high level to low level
    cbIcuPeriod, // This callback is called when input signal pass from the low level to high level
    NULL,              // There is no callback when the counter goes in overflow
    ICU_CHANNEL_1,             // It configures the first channel of ICU Driver
    };

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

static void cmd_unlock(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: unlock\r\n");
  }
  else {
    open_flag = 1;
  }
}

static void cmd_setcode(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  int x = 0;
  if (argc == 4) {
    for (int i = 0; i < CODE_DIGIT; i++) {
      x = atoi(argv[i]);
      CODE[i] = x;
    }

  }
  else {
    chprintf(chp, "Usage: setcode [|CODE]\r\n");
  }
}

static void cmd_exitlock(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: exitlock\r\n");
  }
  else {
    lock_box = 0;
    error_count = 0;
  }
}

static const ShellCommand commands[] =
    { {"unlock", cmd_unlock}, {"setcode", cmd_setcode}, {"exitlock",
                                                         cmd_exitlock},
     {NULL, NULL}};

static const ShellConfig shell_cfg1 = {(BaseSequentialStream*)&SD2, commands};

//*********THREAD SERVO************//
/*
 * Se il flag di apertura vale 1, allora il servo ruota di 90° sbloccando il locker.
 */

static THD_WORKING_AREA(waServoThread, 128);
static THD_FUNCTION( ServoThread, arg) {
  (void)arg;
  chRegSetThreadName("Servo");

  int lock_status[] = {750, 1200};

  while (true) {
    //In base allo stato del flag di corretezza del codice, decido se aprire o meno la cassaforte
    if (open_flag) {
      pwmEnableChannel(&PWMD3, 0,
                       PWM_PERCENTAGE_TO_WIDTH(&PWMD3, lock_status[0]));
      chThdSleepMilliseconds(5000); // Elapsed time needed to open the locker

    }
    else {
      pwmEnableChannel(&PWMD3, 0,
                       PWM_PERCENTAGE_TO_WIDTH(&PWMD3, lock_status[1]));
    }

    // This waits 1 second
    chThdSleepMilliseconds(500);
  }
}

//*********THREAD PHOTORESISTOR************//
/*
 * Tramite l'ADC misuro la tensione in ingresso. All'aumentare della luminosità, questa diminuisce.
 * Dal valore dei campioni, calcolo il valor medio e valuto se il locker sia aperto o meno.
 */

static THD_WORKING_AREA(waPhotoResistorThread, 128);
static THD_FUNCTION( PhotoResistorThread, arg) {
  (void)arg;
  chRegSetThreadName("PhotoResistor");

  float ave = 0.0;

  adcStart(&ADCD1, NULL);

  while (true) {
    adcConvert(&ADCD1, &adcgrpcfg1, (adcsample_t*)samples1, ADC_GRP1_BUF_DEPTH);

    ave = 0;
    //calcolo il valor medio di tensione in ingresso e decido se la cassaforte è aperta o meno
    for (int i = 0; i < ADC_GRP1_BUF_DEPTH; i++) {
      ave += (float)samples1[i];
    }

    if (ave / ADC_GRP1_BUF_DEPTH <= 3600) //Locker aperto --> accendo LED Verde
        {
      palSetPad(LED_RGB_GREEN_port, LED_RGB_GREEN_pin);
      palClearPad(LED_RGB_RED_port, LED_RGB_RED_pin);
    }
    else //Locker chiuso --> accendo LED Giallo e metto il flag di apertura porta a 0
    {
      palSetPad(LED_RGB_GREEN_port, LED_RGB_GREEN_pin);
      palSetPad(LED_RGB_RED_port, LED_RGB_RED_pin);
      open_flag = 0;
    }

    // This waits 1 second
    chThdSleepMilliseconds(500);
  }
}

//*********ISR BUTTON************//

static void confirmation_button(void *arg) {
  (void)arg;
  press_button_flag = 1;
}

//*********THREAD CODICE PIN & OLED************//
/*
 * Si fa inserire all'utente 4 numeri che rappresentano il suo codice. Ad ogni inserimento suona il buzzer.
 * Se l codice è corretto abilito il flag di apertura locker,
 * altrimenti mostro su OLED segnale di errore e conto il numero di tentativi.
 * Dopo 3 tentativi si blocca l'inserimento.
 */

static THD_WORKING_AREA(waCode, 1024);
static THD_FUNCTION( Code, arg) {
  (void)arg;
  chRegSetThreadName("Code");

  int c = 0, correct_code_flag = 1;

  //Start OLED
  ssd1306ObjectInit(&SSD1306D1);
  ssd1306Start(&SSD1306D1, &ssd1306cfg);
  ssd1306FillScreen(&SSD1306D1, 0x00);

  ssd1306UpdateScreen(&SSD1306D1);

  //Pulisco LED di errore
  palClearPad(LED_ERR_CODE_port, LED_ERR_CODE_pin);

  while (true) {
    if (!lock_box) {
      if (!open_flag) {
        //Upoad Display
        ssd1306FillScreen(&SSD1306D1, 0x00);           //pulizia dello schermo

        //******Stampa a video della frase: ScassaForte\nInsert PIN...

        ssd1306GotoXy(&SSD1306D1, 5, 1);
        chsnprintf(buff, BUFF_SIZE, "sCassaForte");
        ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

        ssd1306GotoXy(&SSD1306D1, 23, 24);
        chsnprintf(buff, BUFF_SIZE, "Insert PIN");
        ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_7x10, SSD1306_COLOR_BLACK);

        ssd1306GotoXy(&SSD1306D1, 55, 38);
        chsnprintf(buff, BUFF_SIZE, "%d", rotation_number);
        ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

        ssd1306UpdateScreen(&SSD1306D1);

      }

      if (press_button_flag) {

        palSetPad(BUZZER_port, BUZZER_pin);           //Insert code sound
        chThdSleepMilliseconds(50);
        palClearPad(BUZZER_port, BUZZER_pin);

        if (digit_count != CODE_DIGIT) {

          INSERT_CODE[digit_count] = rotation_number;
          digit_count++;
          rotation_number = 0;
          if (digit_count != 4)
            press_button_flag = 0; //Nell'inserimento dell'uktima cifra bisogna lasciare il flag del bottone a 1

        }
        else {
          digit_count = 0;
          correct_code_flag = 1;
          for (int i = 0; i < CODE_DIGIT && correct_code_flag; i++) {
            if (INSERT_CODE[i] != CODE[i])
              correct_code_flag = 0;
          }

          if (correct_code_flag) {
            open_flag = 1; //Abilito flag di apertura locker
            error_count = 0;

            //Stampa V
            for (int y = 0; y < 64; y++) {
              for (int x = 0; x < 128; x++) {
                ssd1306DrawPixel(&SSD1306D1, x, y, image_correct[c]);
                c++;
              }
            }
            c = 0;
            ssd1306UpdateScreen(&SSD1306D1);
          }
          else {

            //stampa X
            for (int y = 0; y < 64; y++) {
              for (int x = 0; x < 128; x++) {
                ssd1306DrawPixel(&SSD1306D1, x, y, image_wrong[c]);
                c++;
              }
            }
            c = 0;
            ssd1306UpdateScreen(&SSD1306D1);

            palSetPad(LED_ERR_CODE_port, LED_ERR_CODE_pin); //Accendo LED rosso per segnalare errore
            palSetPad(BUZZER_port, BUZZER_pin); //Accensione buzzer per segnare errore

            chThdSleepMilliseconds(1000);

            palClearPad(LED_ERR_CODE_port, LED_ERR_CODE_pin); //Spengo LED rosso
            palClearPad(BUZZER_port, BUZZER_pin); //Spengo buzzer

            if (error_count < 2) //Se ci sono ancora tentativi:
                {
              error_count++; //Numero di errori

              ssd1306FillScreen(&SSD1306D1, 0x00); //Pulizia dello schermo
              ssd1306GotoXy(&SSD1306D1, 0, 0);
              chsnprintf(buff, BUFF_SIZE, "Tentativi rimasti: ");
              ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_7x10,
                          SSD1306_COLOR_WHITE);

              ssd1306GotoXy(&SSD1306D1, 55, 35);
              chsnprintf(buff, BUFF_SIZE, "%d ", 3 - error_count); //Numero di tentativi rimasti stampati a video
              ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18,
                          SSD1306_COLOR_WHITE);
              ssd1306UpdateScreen(&SSD1306D1);
            }
            else //Se non ci sono più tentativi:
            {
              lock_box = 1;

              //stampa Lock
              for (int y = 0; y < 64; y++) {
                for (int x = 0; x < 128; x++) {
                  ssd1306DrawPixel(&SSD1306D1, x, y, image_lock[c]);
                  c++;
                }
              }
              c = 0;
              ssd1306UpdateScreen(&SSD1306D1);
            }

          }
          press_button_flag = 0; //Il pulsante è stato rilasciato e pulisco il flag per le misurazioni future
          rotation_number = 0; //Rimetto a zero il numero di ticks per la futura misurazione
        }
      }
    }
    else {

      ssd1306FillScreen(&SSD1306D1, 0x00); //Pulizia dello schermo
      ssd1306GotoXy(&SSD1306D1, 24, 0);
      chsnprintf(buff, BUFF_SIZE, "Contact"); //Messaggio di locker bloccato
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

      ssd1306GotoXy(&SSD1306D1, 41, 20);
      chsnprintf(buff, BUFF_SIZE, "the"); //Messaggio di locker bloccato
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);

      ssd1306GotoXy(&SSD1306D1, 32, 45);
      chsnprintf(buff, BUFF_SIZE, "Admin"); //Messaggio di locker bloccato
      ssd1306Puts(&SSD1306D1, buff, &ssd1306_font_11x18, SSD1306_COLOR_WHITE);
      ssd1306UpdateScreen(&SSD1306D1);

    }
    // This waits 1 second
    chThdSleepMilliseconds(1000);
  }
}

//************THREAD SENSORE IR***********//
/*
 * Quando il RX non ricevealcun segnale, il PIN ha un valore basso
 * e qesto indica la presenza di un oggeto => accensione LED BLUE
 */

static THD_WORKING_AREA(waIR, 128);
static THD_FUNCTION( IR, arg) {
  (void)arg;
  chRegSetThreadName("IR");

  palSetPadMode(RX_IR_port, RX_IR_pin, PAL_MODE_INPUT_PULLDOWN);

  while (true) {

    if (palReadPad(RX_IR_port, RX_IR_pin) == PAL_LOW) //Accendo il LED BLU se c'è un oggeto, altrimenti lo spengo
      palSetPad(LED_TX_RX_port, LED_TX_RX_pin);
    else
      palClearPad(LED_TX_RX_port, LED_TX_RX_pin);

    chThdSleepMilliseconds(50);
  }

}

int main(void) {
  halInit();
  chSysInit();
  shellInit();

  //*********CONFIGURAZIONE PIN*************//

  palSetPadMode(LED_TX_RX_port, LED_TX_RX_pin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LED_ERR_CODE_port, LED_ERR_CODE_pin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(BUZZER_port, BUZZER_pin, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(BUZZER_port, BUZZER_pin);

  palSetLineMode(
      LINE_ARD_D15,
      PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN
          | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);
  palSetLineMode(
      LINE_ARD_D14,
      PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN
          | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLUP);

  palSetPadMode(ENCODER_CLK_port, ENCODER_CLK_pin, PAL_MODE_ALTERNATE(2));
  palSetPadMode(ENCODER_DT_port, ENCODER_DT_pin, PAL_MODE_INPUT);
  palSetPadMode(PHOTORESISTOR_port, PHOTORESISTOR_pin, PAL_MODE_INPUT_ANALOG);
  palEnablePadEvent(BUTTON_EXT_port, BUTTON_EXT_pin,
                    PAL_EVENT_MODE_FALLING_EDGE);
  palSetPadCallback(BUTTON_EXT_port, BUTTON_EXT_pin, confirmation_button, NULL);
  palSetPadMode(LED_RGB_BLUE_port, LED_RGB_BLUE_pin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LED_RGB_GREEN_port, LED_RGB_GREEN_pin,
                PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LED_RGB_RED_port, LED_RGB_RED_pin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(SERVO_port, SERVO_pin, PAL_MODE_ALTERNATE(2));

  //************ABILITAZIONE PERIFERICI***********//

  //ICU --> TIM5
  icuStart(&ICUD5, &icucfg);
  icuStartCapture(&ICUD5);
  icuEnableNotifications(&ICUD5);

  //USART2
  sdStart(&SD2, NULL);

  //Turn-off LED RGB
  palClearPad(LED_RGB_BLUE_port, LED_RGB_BLUE_pin);
  palClearPad(LED_RGB_GREEN_port, LED_RGB_GREEN_pin);
  palClearPad(LED_RGB_RED_port, LED_RGB_RED_pin);

  // It stars PWM driver.
  pwmStart(&PWMD3, &pwmcfg);

  // It enables the periodic callback at the beginning of period
  pwmEnablePeriodicNotification(&PWMD3);

  // It enables the periodic callback at the end of pulse
  pwmEnableChannelNotification(&PWMD3, 0);

  //************Chiamate ai Thread**********//

  // Thread SERVO
  chThdCreateStatic(waServoThread, sizeof(waServoThread), NORMALPRIO + 1,
                    ServoThread, &open_flag);

  //Thread ADC per fotoresistore
  chThdCreateStatic(waPhotoResistorThread, sizeof(waPhotoResistorThread),
                    NORMALPRIO + 2, PhotoResistorThread, NULL);

  //Thread CODICE & OLED
  chThdCreateStatic(waCode, sizeof(waCode), NORMALPRIO, Code, NULL);

  //Thread IR-RX
  chThdCreateStatic(waIR, sizeof(waIR), NORMALPRIO - 1, IR, NULL);

  while (true) {
    thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell",
                                            NORMALPRIO + 1, shellThread,
                                            (void*)&shell_cfg1);

    chThdWait(shelltp);

    chThdSleepMilliseconds(1000);
  }
  adcStop(&ADCD1);
}
