/***************************************************************************
# SDU UAS Center TX firmware 
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# This library is based on the source example Generate_PPM_signal_V0.2 obtained
# from https://code.google.com/archive/p/generate-ppm-signal/downloads
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
This firmware is developed for the v03-2018 version of the SDU UAS Transmitter

Revision
2018-05-01 KJ First released test version
2018-06-04 KJ Major revision including joystick calibration and AutoQuad
              specific outputs
****************************************************************************/
/* parameters */

#define DEBUG_TO_SERIAL
#define FIRMWARE_VERSION "v2018-06-06"

/* #define DEBUG_LED_ONBOARD_RED_SAME */
/* #define DEBUG_LED_ONBOARD_GREEN_SAME */

/****************************************************************************/
/* main state defines */
#define MAIN_STATE_CAL_MINMAX 0
#define MAIN_STATE_CAL_NEUTRAL_L 1
#define MAIN_STATE_CAL_NEUTRAL_R 2 
#define MAIN_STATE_RUN 3

/* input defines */
#define PIN_LEFT_X A5
#define PIN_LEFT_Y A4
#define PIN_RIGHT_X A3
#define PIN_RIGHT_Y A2
#define PIN_LEFT_3_POS_SW A7
#define PIN_RIGHT_3_POS_SW A6
#define PIN_LEFT_BUTTON 3
#define PIN_RIGHT_BUTTON 4
#define PIN_LEFT_2_POS_SW 7
#define PIN_RIGHT_2_POS_SW 8
#define PIN_POT A1

/* output defines */
#define PIN_TX 6
#define PIN_AUDIO 11
#define PIN_BATT_VOLT A0
#define PIN_BUZZER 5
#define PIN_LED_RED 9
#define PIN_LED_GREEN 10

/* transmitter types (number corresponds to green blinks while in run mode) */
#define TX_TYPE_AUTOQUAD 1
#define TX_TYPE_CRRCSIM 2

/* joystick_defines */
#define JOY_AXIS_RESET_MIN 0xffff
#define JOY_AXIS_RESET_MAX 0
#define JOY_AXIS_MIN_OK 70
#define JOY_AXIS_MAX_OK 954
#define JOY_AXIS_NEUTRAL_MIN 462
#define JOY_AXIS_NEUTRAL_MAX 562

/* led defines */
#define LED_STATE_ON 1
#define LED_STATE_OFF 0
#define LED_TIME_FLIP 200 /* [ms] */
#define LED_TIME_CNT 1000 /* [ms] */

/* ppm defines */
#define PPM_NUMBER 8  //set the number of ppm chanels
#define DEFAULT_SERVO_VALUE 1500  //set the default servo value
#define PPM_FRAME_LEN 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PULSE_LEN 300  //set the pulse length
#define PPM_PULSE_ON_STATE 0  //set polarity of the pulses: 1 is positive, 0 is negative

/* serial debug defines */
#define SER_DEBUG_INTERVAL 500 /* [ms] */

/****************************************************************************/
/* variables */
char main_state;
long time_ms;

/* inputs */
uint16_t input_tmp;
uint16_t input_left_y;
uint16_t input_left_x;
uint16_t input_right_y;
uint16_t input_right_x;
uint8_t input_left_3_pos_sw;
uint8_t input_right_3_pos_sw;
uint16_t input_pot;
boolean input_left_button;
boolean input_right_button;
boolean input_left_2_pos_sw;
boolean input_right_2_pos_sw;
uint8_t batt_volt;

/* joystick calibration values */
uint16_t input_left_y_min;
uint16_t input_left_y_max;
uint16_t input_left_y_neutral;

uint16_t input_left_x_min;
uint16_t input_left_x_max;
uint16_t input_left_x_neutral;

uint16_t input_right_y_min;
uint16_t input_right_y_max;
uint16_t input_right_y_neutral;

uint16_t input_right_x_min;
uint16_t input_right_x_max;
uint16_t input_right_x_neutral;

/* leds */
char led_red_signal;
unsigned long led_red_tout;
boolean led_red_state;
char led_red_count;

char led_green_signal;
unsigned long led_green_tout;
boolean led_green_state;
char led_green_count;

/* tx */
char tx_type;
uint16_t ppm[PPM_NUMBER]; /* [1000;2000] */
uint32_t ppm_calc;

/* other */
boolean button_pressed;
boolean buzzer_state;
unsigned long ser_debug_next;

/* dronecage */

uint16_t cmd[6];
uint16_t cmd_old[6];

uint16_t thr_high;
uint16_t thr_low;
uint16_t pitch_high;
uint16_t pitch_low;
uint16_t roll_high;
uint16_t roll_low;

boolean first_time;


/****************************************************************************/
ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state)
  {  //start pulse
    digitalWrite(PIN_TX, PPM_PULSE_ON_STATE);
    digitalWrite(PIN_AUDIO, PPM_PULSE_ON_STATE);
    OCR1A = PPM_PULSE_LEN * 2;
    state = false;
  }
  else
  {  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(PIN_TX, !PPM_PULSE_ON_STATE);
    digitalWrite(PIN_AUDIO, !PPM_PULSE_ON_STATE);
    state = true;

    if(cur_chan_numb >= PPM_NUMBER)
    {
      digitalWrite(PIN_TX, !PPM_PULSE_ON_STATE);
      digitalWrite(PIN_AUDIO, !PPM_PULSE_ON_STATE);
       cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PULSE_LEN;// 
      OCR1A = (PPM_FRAME_LEN - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PPM_PULSE_LEN) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
/****************************************************************************/
void read_inputs (void)
{
  /* read analog inputs */
  input_left_y = analogRead(PIN_LEFT_Y);
  input_left_x = 1023 - analogRead(PIN_LEFT_X);
  input_right_y = analogRead(PIN_RIGHT_Y);
  input_right_x = 1023 - analogRead(PIN_RIGHT_X);
  
  input_tmp = analogRead(PIN_LEFT_3_POS_SW);
  if (input_tmp < 300)
    input_left_3_pos_sw = 0;
  else if (input_tmp < 700)
    input_left_3_pos_sw = 1;
  else
    input_left_3_pos_sw = 2;

  input_tmp  = analogRead(PIN_RIGHT_3_POS_SW);
  if (input_tmp < 300)
    input_right_3_pos_sw = 0;
  else if (input_tmp < 700)
    input_right_3_pos_sw = 1;
  else
    input_right_3_pos_sw = 2;
  
  input_pot = analogRead(PIN_POT);

  /* read digital inputs */
  input_left_button = digitalRead(PIN_LEFT_BUTTON);
  input_right_button = digitalRead(PIN_RIGHT_BUTTON);
  input_left_2_pos_sw = digitalRead(PIN_LEFT_2_POS_SW);
  input_right_2_pos_sw = digitalRead(PIN_RIGHT_2_POS_SW);  

  /* read battery voltage */
  batt_volt = (uint32_t) 100 * analogRead(PIN_BATT_VOLT)/1024;
}
/***************************************************************************/
void led_update(void)
{
  if (led_red_signal > 0)
  {
    if (time_ms > led_red_tout)
    {
      switch (led_red_state)
      {
        case LED_STATE_ON:
          led_red_state = LED_STATE_OFF;
          digitalWrite (PIN_LED_RED, LOW);
          #ifdef DEBUG_LED_ONBOARD_RED_SAME
            digitalWrite (LED_BUILTIN, LOW);
          #endif
          led_red_tout = time_ms + LED_TIME_FLIP;
          break;

        case LED_STATE_OFF:
          if (led_red_count < led_red_signal)
          {
            led_red_count++;
            digitalWrite (PIN_LED_RED, HIGH);
            #ifdef DEBUG_LED_ONBOARD_RED_SAME
              digitalWrite (LED_BUILTIN, HIGH);
            #endif
            led_red_state = LED_STATE_ON;
            led_red_tout = time_ms + LED_TIME_FLIP;
          }
          else if (time_ms > led_red_tout + LED_TIME_CNT)
          {
            led_red_count = 0;
          }
          break;
      }    
    }
  }
  else
  {
    digitalWrite(PIN_LED_RED, LOW);
  }

  if (led_green_signal > 0)
  {
    if (time_ms > led_green_tout)
    {
      switch (led_green_state)
      {
        case LED_STATE_ON:
          led_green_state = LED_STATE_OFF;
          digitalWrite (PIN_LED_GREEN, LOW);
          #ifdef DEBUG_LED_ONBOARD_GREEN_SAME
            digitalWrite (LED_BUILTIN, LOW);
          #endif
          led_green_tout = time_ms + LED_TIME_FLIP;
          break;

        case LED_STATE_OFF:
          if (led_green_count < led_green_signal)
          {
            led_green_count++;
            digitalWrite (PIN_LED_GREEN, HIGH);
            #ifdef DEBUG_LED_ONBOARD_GREEN_SAME
              digitalWrite (LED_BUILTIN, HIGH);
            #endif
            led_green_state = LED_STATE_ON;
            led_green_tout = time_ms + LED_TIME_FLIP;
          }
          else if (time_ms > led_green_tout + LED_TIME_CNT)
          {
            led_green_count = 0;
          }
          break;
      }    
    }
  }
}
/****************************************************************************/
void serialFlush(){
  while(Serial.available()){
    char tmp = Serial.read();
  }
}
/****************************************************************************/
void reset_leds (void)
{
  /* turn off all LED pins */
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  led_red_signal = 0;
  led_red_state = LED_STATE_OFF;
  led_red_count = 99;
  led_red_tout = time_ms + LED_TIME_FLIP;
  
  led_green_signal = 0;
  led_green_state = LED_STATE_OFF;
  led_green_count = 99;
  led_green_tout = 0;
}
/****************************************************************************/
void setup_run(void)
{
  main_state = MAIN_STATE_RUN;
  reset_leds();
  led_green_signal = tx_type;

  #ifdef DEBUG_TO_SERIAL
    Serial.println ("Entering run mode");
  #endif 
  
  /* initiallize default ppm values */
  for(int i=0; i<PPM_NUMBER; i++)
  {
    ppm[i]= DEFAULT_SERVO_VALUE;
  }

  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_AUDIO, OUTPUT);
  digitalWrite(PIN_TX, !PPM_PULSE_ON_STATE);  //set the PPM signal pin to the default state (off)
  digitalWrite(PIN_AUDIO, !PPM_PULSE_ON_STATE);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}
/****************************************************************************/
void loop_run(void)
{
  unsigned long startTime = millis();
  /* read current input */
  read_inputs();

    if(Serial.available() > 0){
    
    /*char ch = Serial.read();
    if(ch == 'P'){
      cmd[0] = Serial.parseInt();//Serial.read() + 0;
      cmd[1] = Serial.parseInt();//Serial.read() + 0;
      cmd[2] = Serial.parseInt();//Serial.read() + 0;
      cmd[3] = Serial.parseInt();//Serial.read() + 0;
      cmd[4] = Serial.parseInt();//Serial.read() + 0;
      cmd[5] = Serial.parseInt();//Serial.read() + 0;

      

      if(cmd[0] == -1)
        cmd[0] = cmd_old[0];
      if(cmd[1] == -1)
        cmd[1] = cmd_old[1];
      if(cmd[2] == -1)
        cmd[2] = cmd_old[2];
      if(cmd[3] == -1)
        cmd[3] = cmd_old[3];
      if(cmd[4] == -1)
        cmd[4] = cmd_old[4];
      if(cmd[5] == -1)
        cmd[5] = cmd_old[5];

        serialFlush();
      }*/

      const byte numBytes = 6;
      byte Buffer[numBytes];

      int byteCount = Serial.readBytesUntil('\0', Buffer, sizeof(Buffer));
      
      if(byteCount == 6){
        cmd[0] = Buffer[0]-1;
        cmd[1] = Buffer[1]-1;
        cmd[2] = Buffer[2]-1;
        cmd[3] = Buffer[3]-1;
        cmd[4] = Buffer[4]-1;
        cmd[5] = Buffer[5]-1;
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }

  /* set red led according to battery voltage */
  if (batt_volt <= 70)
    led_red_signal = 2;
  else if (batt_volt <= 73)
    led_red_signal = 1;
  else
  {
    led_red_signal = 0;

  }

  /* limit input to calibrated minimum and maximum */
  if (input_left_y > input_left_y_max)
    input_left_y = input_left_y_max;
  else if (input_left_y < input_left_y_min)
    input_left_y = input_left_y_min;

  if (input_left_x > input_left_x_max)
    input_left_x = input_left_x_max;
  else if (input_left_x < input_left_x_min)
    input_left_x = input_left_x_min;

  if (input_right_y > input_right_y_max)
    input_right_y = input_right_y_max;
  else if (input_right_y < input_right_y_min)
    input_right_y = input_right_y_min;

  if (input_right_x > input_right_x_max)
    input_right_x = input_right_x_max;
  else if (input_right_x < input_right_x_min)
    input_right_x = input_right_x_min;


  /* CRRCsim TX */
  if (tx_type == TX_TYPE_CRRCSIM)
  {
    /* throttle */
    if (input_left_y > input_left_y_neutral)
      ppm[0] = 1500 + (uint32_t) 350 * (input_left_y - input_left_y_neutral) / (input_left_y_max - input_left_y_neutral);
    else
      ppm[0] = 1500 - (uint32_t) 350 * (input_left_y_neutral - input_left_y) / (input_left_y_neutral - input_left_y_min);

    /* yaw (rudder) */
    if (input_left_x > input_left_x_neutral)
      ppm[3] = 1500 + (uint32_t) 350 * (input_left_x - input_left_x_neutral) / (input_left_x_max - input_left_x_neutral);
    else
      ppm[3] = 1500 - (uint32_t) 350 * (input_left_x_neutral - input_left_x) / (input_left_x_neutral - input_left_x_min);

    /* roll (aileron) */
    if (input_right_x > input_right_x_neutral)
      ppm[1] = 1500 + (uint32_t) 350 * (input_right_x - input_right_x_neutral) / (input_right_x_max - input_right_x_neutral);
    else
      ppm[1] = 1500 - (uint32_t) 350 * (input_right_x_neutral - input_right_x) / (input_right_x_neutral - input_right_x_min);

    /* pitch (elevator) */
    if (input_right_y > input_right_y_neutral)
      ppm[2] = 1500 + (uint32_t) 350 * (input_right_y - input_right_y_neutral) / (input_right_y_max - input_right_y_neutral);
    else
      ppm[2] = 1500 - (uint32_t) 350 * (input_right_y_neutral - input_right_y) / (input_right_y_neutral - input_right_y_min);

    // unused for now
    ppm[4] = DEFAULT_SERVO_VALUE;
    ppm[5] = DEFAULT_SERVO_VALUE;
    ppm[6] = DEFAULT_SERVO_VALUE;
    ppm[7] = DEFAULT_SERVO_VALUE;
  } /* end CRRCsim TX */

  /* AutoQuad TX */
  else if (tx_type == TX_TYPE_AUTOQUAD)
  {
    /* throttle */
    if (input_left_y > input_left_y_neutral){
      input_left_y = ((input_left_y - input_left_y_neutral) * cmd[0]/100) + input_left_y_neutral;
      ppm[0] = (1500 + (uint32_t) 350 * ((input_left_y) - input_left_y_neutral) / (input_left_y_max - input_left_y_neutral));
    }
    else{
      input_left_y = input_left_y_neutral - ((input_left_y_neutral - input_left_y) * cmd[1]/100);
      ppm[0] = (1500 - (uint32_t) 350 * (input_left_y_neutral - (input_left_y)) / (input_left_y_neutral - input_left_y_min));
    }

    /* yaw (rudder) */
    if (input_left_x > input_left_x_neutral)
      ppm[3] = 1500 + (uint32_t) 350 * (input_left_x - input_left_x_neutral) / (input_left_x_max - input_left_x_neutral);
    else
      ppm[3] = 1500 - (uint32_t) 350 * (input_left_x_neutral - input_left_x) / (input_left_x_neutral - input_left_x_min);

    /* handle special case for arming/disarming Autouad */
    if (ppm[0] < 1300)
    {
      if (ppm[3] > 1700) /* arm */
      {
        ppm[0] = 1150;
        ppm[3] = 1850;
      }
      else if (ppm[3] < 1300) /* disarm */
      {
        ppm[0] = 1150;
        ppm[3] = 1150;
      }
    }

    /* roll (aileron) */
    if (input_right_x > input_right_x_neutral){
      input_right_x = ((input_right_x - input_right_x_neutral) * cmd[4]/100) + input_right_x_neutral;
      ppm[1] = (1500 + (uint32_t) 350 * ((input_right_x) - input_right_x_neutral) / (input_right_x_max - input_right_x_neutral));
    }
    else{
      input_right_x = input_right_x_neutral - ((input_right_x_neutral - input_right_x) * cmd[5]/100);
      ppm[1] = (1500 - (uint32_t) 350 * (input_right_x_neutral - (input_right_x)) / (input_right_x_neutral - input_right_x_min));
    }

    /* pitch (elevator)  INVERTED */
    if (input_right_y > input_right_y_neutral){
      input_right_y = ((input_right_y - input_right_y_neutral) * cmd[2]/100) + input_right_y_neutral;
      ppm[2] = (1500 - (uint32_t) 350 * ((input_right_y) - input_right_y_neutral) / (input_right_y_max - input_right_y_neutral));
    }
    else{
      input_right_y = input_right_y_neutral - ((input_right_y_neutral - input_right_y) * cmd[3]/100);
      ppm[2] = (1500 + (uint32_t) 350 * (input_right_y_neutral - (input_right_y)) / (input_right_y_neutral - input_right_y_min));
    }

    /* attidue hold/position hold/mission mode */
    if (input_left_3_pos_sw == 0)
      ppm[5] = 1150;
    else if (input_left_3_pos_sw == 1)
      ppm[5] = 1500;
    else
      ppm[5] = 1850;

    /* home set/neutral/home go */
    if (input_right_3_pos_sw == 0)
      ppm[6] = 1150;
    else if (input_right_3_pos_sw == 1)
      ppm[6] = 1500;
    else
      ppm[6] = 1850;

    // unused for now
    ppm[4] = DEFAULT_SERVO_VALUE;
    ppm[7] = DEFAULT_SERVO_VALUE;
  } /* end AutoQuad TX */



  if(ppm[1] > 1600)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);

  if(ppm[2] > 1600)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
  unsigned long currentTime = millis();
  unsigned long elapsedTime = elapsedTime - currentTime;

  #ifdef DEBUG_TO_SERIAL
    if(time_ms >= ser_debug_next)
    {
      ser_debug_next = time_ms + SER_DEBUG_INTERVAL;
      Serial.print ("PPM: ");
      Serial.print (ppm[0]);
      Serial.print (" ");
      Serial.print (ppm[1]);
      Serial.print (" ");
      Serial.print (ppm[2]);
      Serial.print (" ");
      Serial.print (ppm[3]);
      Serial.print (" ");
      Serial.print (ppm[4]);
      Serial.print (" ");
      Serial.print (ppm[5]);
      Serial.print (" ");
      Serial.print (ppm[6]);
      Serial.print (" ");
      Serial.print (ppm[7]);
      Serial.print ("  Battery: ");
      Serial.print (batt_volt);
      Serial.println (" Volt*10");
      Serial.print("Elapsed Time: ");
      Serial.print(elapsedTime);
      Serial.println();
    }
  #endif
      


      //serialFlush();
}
/****************************************************************************/
void setup_cal_neutral_left(void)
{
  main_state = MAIN_STATE_CAL_NEUTRAL_L;
  reset_leds();
  led_red_signal = 1;

  #ifdef DEBUG_TO_SERIAL
    Serial.println ("Calibrating left joystick neutral position");
  #endif
  button_pressed = false;
}
/****************************************************************************/
void loop_cal_neutral_left(void)
{
  digitalWrite(LED_BUILTIN, LOW);
  read_inputs();

  /* check if user wishes to perform left neutral calibration */
  if (button_pressed == true && input_right_button == false)
  {
    if (input_left_y >= JOY_AXIS_NEUTRAL_MIN && input_left_y <= JOY_AXIS_NEUTRAL_MAX
      && input_left_x >= JOY_AXIS_NEUTRAL_MIN && input_left_x <= JOY_AXIS_NEUTRAL_MAX
      && input_right_y >= JOY_AXIS_NEUTRAL_MIN && input_right_y <= JOY_AXIS_NEUTRAL_MAX
      && input_right_x >= JOY_AXIS_NEUTRAL_MIN && input_right_x <= JOY_AXIS_NEUTRAL_MAX)
    {
      input_left_y_neutral = input_left_y;
      input_left_x_neutral = input_left_x;

      #ifdef DEBUG_TO_SERIAL
        if(time_ms >= ser_debug_next)
        {
          ser_debug_next = time_ms + SER_DEBUG_INTERVAL;
          Serial.print ("Left Y neutral: ");
          Serial.print (input_left_y_neutral);
          Serial.print (" Left Y neutral: ");
          Serial.print (input_left_x_neutral);
          Serial.print ("\n");
        }
      #endif
      
      setup_run();
    }
    else
    {
       button_pressed = false;
    }
  }
  else if (input_right_button == true)
    button_pressed = true;
  
}
/****************************************************************************/
void setup_cal_minmax(void)
{
  main_state = MAIN_STATE_CAL_MINMAX;
  #ifdef DEBUG_TO_SERIAL
    Serial.println ("Calibrating joystick minimum and maximum");
  #endif
  reset_leds();
  led_red_signal = 2;
  button_pressed = false;

  input_left_y_min = JOY_AXIS_RESET_MIN;
  input_left_y_max = JOY_AXIS_RESET_MAX;
  input_left_x_min = JOY_AXIS_RESET_MIN;
  input_left_x_max = JOY_AXIS_RESET_MAX;
  input_right_y_min = JOY_AXIS_RESET_MIN;
  input_right_y_max = JOY_AXIS_RESET_MAX;
  input_right_x_min = JOY_AXIS_RESET_MIN;
  input_right_x_max = JOY_AXIS_RESET_MAX;
}
/****************************************************************************/
void loop_cal_minmax(void)
{
  read_inputs();

  if (input_left_y < input_left_y_min)
    input_left_y_min = input_left_y;
  else if (input_left_y > input_left_y_max)
    input_left_y_max = input_left_y;

  if (input_left_x < input_left_x_min)
    input_left_x_min = input_left_x;
  else if (input_left_x > input_left_x_max)
    input_left_x_max = input_left_x;

  if (input_right_y < input_right_y_min)
    input_right_y_min = input_right_y;
  else if (input_right_y > input_right_y_max)
    input_right_y_max = input_right_y;

  if (input_right_x < input_right_x_min)
    input_right_x_min = input_right_x;
  else if (input_right_x > input_right_x_max)
    input_right_x_max = input_right_x;

  #ifdef DEBUG_TO_SERIAL
    if(time_ms >= ser_debug_next)
    {
      ser_debug_next = time_ms + SER_DEBUG_INTERVAL;
      Serial.print ("Left Y: ");
      Serial.print (input_left_y_min);
      Serial.print (" ");
      Serial.print (input_left_y_max);
      Serial.print (" Left X: ");
      Serial.print (input_left_x_min);
      Serial.print (" ");
      Serial.print (input_left_x_max);
      Serial.print (" ");
      Serial.print ("Right Y: ");
      Serial.print (input_right_y_min);
      Serial.print (" ");
      Serial.print (input_right_y_max);
      Serial.print (" Right X: ");
      Serial.print (input_right_x_min);
      Serial.print (" ");
      Serial.print (input_right_x_max);
      Serial.print ("\n");
    }
  #endif

  /* check if user wishes to conclude minmax calibration */
  if (button_pressed == true && input_left_button == false)
  {
    if (input_left_y_min <= JOY_AXIS_MIN_OK && input_left_y_max >= JOY_AXIS_MAX_OK
      && input_left_x_min <= JOY_AXIS_MIN_OK && input_left_x_max >= JOY_AXIS_MAX_OK
      && input_right_y_min <= JOY_AXIS_MIN_OK && input_right_y_max >= JOY_AXIS_MAX_OK
      && input_right_x_min <= JOY_AXIS_MIN_OK && input_right_x_max >= JOY_AXIS_MAX_OK

      && input_right_y >= JOY_AXIS_NEUTRAL_MIN && input_right_y <= JOY_AXIS_NEUTRAL_MAX
      && input_right_x >= JOY_AXIS_NEUTRAL_MIN && input_right_x <= JOY_AXIS_NEUTRAL_MAX)
    {

      input_right_y_neutral = input_right_y;
      input_right_x_neutral = input_right_x;

      #ifdef DEBUG_TO_SERIAL
        Serial.println ("Calibrated right joystick neutral position");
        Serial.print ("Right Y neutral: ");
        Serial.print (input_right_y_neutral);
        Serial.print (" Right X neutral: ");
        Serial.print (input_right_x_neutral);
        Serial.print ("\n");
      #endif

      setup_cal_neutral_left();
    }
    else
    {
      button_pressed = false;

      #ifdef DEBUG_TO_SERIAL
        Serial.print ("Calibration not completed correctly.");
        Serial.print ("\n");
      #endif
    }
  }
  else if (input_left_button == true)
    button_pressed = true;
}
/****************************************************************************/
void setup()
{ 
  first_time = true; // used for first time remapping
  cmd[0] = 100;
  cmd[1] = 100;
  cmd[2] = 100;
  cmd[3] = 100;
  cmd[4] = 100;
  cmd[5] = 100;
  cmd_old[0] = 100;
  cmd_old[1] = 100;
  cmd_old[2] = 100;
  cmd_old[3] = 100;
  cmd_old[4] = 100;
  cmd_old[5] = 100;


  pinMode(LED_BUILTIN, OUTPUT);
  // setup digital output pins
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(PIN_LED_RED, OUTPUT); 
  pinMode(PIN_LED_GREEN, OUTPUT); 
  pinMode(PIN_BUZZER, OUTPUT); 

  // enable pull-up resistor on digital input pins 
  digitalWrite (PIN_LEFT_BUTTON, HIGH);
  digitalWrite (PIN_RIGHT_BUTTON, HIGH);
  digitalWrite (PIN_LEFT_2_POS_SW, HIGH);
  digitalWrite (PIN_RIGHT_2_POS_SW, HIGH);

  /* read tx type based on button pressed at startup */
  if (digitalRead(PIN_LEFT_BUTTON) == false)
    tx_type = TX_TYPE_CRRCSIM;
  else
    tx_type = TX_TYPE_AUTOQUAD;

  /* initializing the serial port */
  Serial.begin(115200);

  #ifdef DEBUG_TO_SERIAL
    Serial.print ("SDU UAS TX 03-2018 firmware ");
    Serial.println (FIRMWARE_VERSION);
    if (tx_type == TX_TYPE_AUTOQUAD)
      Serial.println ("TX type: AutoQuad");
    else if (tx_type == TX_TYPE_CRRCSIM)
      Serial.println ("TX type: CRRCsim");
  #endif
  
  /* enter calibration mode */
  setup_cal_minmax();
}
/****************************************************************************/
void loop()
{
  time_ms = millis();
  led_update();

  switch (main_state)
  {
    case MAIN_STATE_CAL_MINMAX:
      loop_cal_minmax();
      break;
    case MAIN_STATE_CAL_NEUTRAL_L:
      loop_cal_neutral_left();
      break;
    case MAIN_STATE_RUN:
      loop_run();
      break;
   }
    
  delay(10);
}
/****************************************************************************/


