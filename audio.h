/*
   ESP8266 + FastLED + IR Remote: https://github.com/jasoncoon/esp8266-fastled-audio
   Copyright (C) 2015-2016 Jason Coon
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <driver/adc.h>
// Portions of this file are adapted from the work of Stefan Petrick:
// https://plus.google.com/u/0/115124694226931502095

// Portions of this file are adapted from RGB Shades Audio Demo Code by Garrett Mace:
// https://github.com/macetech/RGBShadesAudio

// Pin definitions
#define MSGEQ7_AUDIO_PIN 36 //14
//audio pin is gpio36
#define MSGEQ7_STROBE_PIN 26 //15
#define MSGEQ7_RESET_PIN  27 //16

#define AUDIODELAY 0

// Smooth/average settings
#define SPECTRUMSMOOTH 0.08
#define PEAKDECAY 0.01
#define NOISEFLOOR 65

// AGC settings
#define AGCSMOOTH 0.004
#define GAINUPPERLIMIT 15.0
#define GAINLOWERLIMIT 0.1

// Global variables
unsigned int spectrumValue[7];  // holds raw adc values
float spectrumDecay[7] = {0};   // holds time-averaged values
float spectrumPeaks[7] = {0};   // holds peak values
float audioAvg = 500.0;
float gainAGC = 2.0;

uint8_t spectrumByte[7];        // holds 8-bit adjusted adc values

uint8_t spectrumAvg;
float reading=0.0;
unsigned long currentMillis; // store current loop's millis value
unsigned long audioMillis; // store time of last audio update

void initializeAudio() {
  pinMode(MSGEQ7_AUDIO_PIN, INPUT);
  pinMode(MSGEQ7_RESET_PIN, OUTPUT);
  pinMode(MSGEQ7_STROBE_PIN, OUTPUT);

  digitalWrite(MSGEQ7_RESET_PIN, LOW);
  digitalWrite(MSGEQ7_STROBE_PIN, HIGH);
}

void readAudio() {
//  adc1_config_width(ADC_WIDTH_BIT_12);
//adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
// double reading = adc1_get_raw(ADC1_CHANNEL_0);
// 
  static PROGMEM const byte spectrumFactors[7] = {9, 11, 13, 13, 12, 12, 13};

  // reset MSGEQ7 to first frequency bin
  digitalWrite(MSGEQ7_RESET_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(MSGEQ7_RESET_PIN, LOW);

  // store sum of values for AGC
  int analogsum = 0;

  // cycle through each MSGEQ7 bin and read the analog values

 

  
  for (int i = 0; i < 7; i++) {
    reading =  analogRead(MSGEQ7_AUDIO_PIN);
    // set up the MSGEQ7
    digitalWrite(MSGEQ7_STROBE_PIN, LOW);
    delayMicroseconds(50); // to allow the output to settle
    
    // read the analog value
    
//    if(reading < 1 || reading > 4095){
//    reading=0;
//    }
//    else{
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
//  reading= -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
 // Added an improved polynomial, use either, comment out as required


    spectrumValue[i] = reading;
    digitalWrite(MSGEQ7_STROBE_PIN, HIGH);

    // noise floor filter
    if (spectrumValue[i] < NOISEFLOOR) {
      spectrumValue[i] = 0;
    } else {
      spectrumValue[i] -= NOISEFLOOR;
    }

    // apply correction factor per frequency bin
    spectrumValue[i] = (spectrumValue[i] * pgm_read_byte_near(spectrumFactors + i)) / 10;

    // prepare average for AGC
    analogsum += spectrumValue[i];

    // apply current gain value
    spectrumValue[i] *= gainAGC;

    // process time-averaged values
    spectrumDecay[i] = (1.0 - SPECTRUMSMOOTH) * spectrumDecay[i] + SPECTRUMSMOOTH * spectrumValue[i];

    // process peak values
    if (spectrumPeaks[i] < spectrumDecay[i]) spectrumPeaks[i] = spectrumDecay[i];
    spectrumPeaks[i] = spectrumPeaks[i] * (1.0 - PEAKDECAY);

    spectrumByte[i] = spectrumValue[i] / 4;
  }

  // Calculate audio levels for automatic gain
  audioAvg = (1.0 - AGCSMOOTH) * audioAvg + AGCSMOOTH * (analogsum / 7.0);

  spectrumAvg = (analogsum / 7.0) / 4;

  // Calculate gain adjustment factor
  gainAGC = 270.0 / audioAvg;
  if (gainAGC > GAINUPPERLIMIT) gainAGC = GAINUPPERLIMIT;
  if (gainAGC < GAINLOWERLIMIT) gainAGC = GAINLOWERLIMIT;
}



void print_audio() {
  
  for (int band = 0; band < 7; band++) {
    Serial.print(spectrumByte[band]);
    Serial.print("\t");
  }
  Serial.println();
}

void radiate() {

  //HALF_POS = beatsin8(40, 30 + SPEED, 80 + SPEED);
  int MILLISECONDS  = 0;
  //  EVERY_N_MILLISECONDS(50) {
  //    hue++;
  //    if ( hue > 255) hue = 0;
  //  }
uint8_t zero_l, three_l, six_l, zero_r, three_r, six_r;

  zero_l  = spectrumByte[0];
  three_l = spectrumByte[3];
  six_l   = spectrumByte[6];

  zero_r  = spectrumByte[0];
  three_r = spectrumByte[3];
  six_r   = spectrumByte[6];

  leds[CENTER_LED] = CRGB(zero_l, three_l, six_l);
  leds[CENTER_LED + 1] = CRGB(zero_r, three_r, six_r);
  //leds[HALF_POS].fadeToBlackBy(30);


  EVERY_N_MILLISECONDS(11) {
    for (int i = NUM_LEDS - 1; i > CENTER_LED + 1; i--) {
      leds[i].blue = leds[i - 1].blue;
    }
    for (int i = 0; i < CENTER_LED; i++) {
      leds[i].blue = leds[i + 1].blue;
    }
  }
  EVERY_N_MILLISECONDS(27) {
    for (int i = NUM_LEDS - 1; i > CENTER_LED + 1; i--) {
      leds[i].green = leds[i - 1].green;
    }
    for (int i = 0; i < CENTER_LED; i++) {
      leds[i].green = leds[i + 1].green;
    }
  }
  EVERY_N_MILLISECONDS(52) {
    for (int i = NUM_LEDS - 1; i > CENTER_LED + 1; i--) {
      leds[i].red = leds[i - 1].red;
    }
    for (int i = 0; i < CENTER_LED; i++) {
      leds[i].red = leds[i + 1].red;
    }
  }

  EVERY_N_MILLISECONDS(2) {
    //blur1d(leds, NUM_LEDS, 1);
  }
}
