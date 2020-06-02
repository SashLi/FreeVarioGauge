//    VarioSound is a programm to generate the vario sound using NMEA Output of OpenVario.
//    Copyright (C) 2019  Dirk Jung Blaubart@gmx.de
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#define STF_MODE 13
#define STF_AUTO 33
#define XC_WK 14                      //Automatikmodus durch Wölbklappen oder XCSoar; Taster an GND anschließen, 10 kOhm Pullup-Widerstand zw. 3,3V und Pin anschließen
#define RXD1 32
#define TXD1 33
#define RXD2 16
#define TXD2 -1 // -1 means it not used beacause of trouble with Dispolay-ESP32. Set TXD2 to 17 if you like to use
#define FNC_PIN 4       // Can be any digital IO pin

#include <AD9833.h>     // Include the library
AD9833 gen(FNC_PIN);       // Defaults to 25MHz internal reference frequency

TaskHandle_t SoundTask;

const int Varioschalter = 15;        //Taster an GND anschließen, 10 kOhm Pullup-Widerstand zw. 3,3V und Pin anschließen
const int STFSchalter = 5;          //Taster an GND anschließen, 10 kOhm Pullup-Widerstand zw. 3,3V und Pin anschließen
const int STFAuto = 19;             //Wölbklappenanschluss; Taster an GND anschließen, 10 kOhm Pullup-Widerstand zw. 3,3V und Pin anschließen´

float stf = 0;           //Speed to Fly Wert

String mod;           //aktueller Modus

bool error = false;

float sf = 0;
float sfOld = 0;
float var = 0;
float varOld = 0;
float tas;
float freqValue = 0;
float freqValueOld = 350;
float freqValueNeg = 0;
float freqValueInc = 0 ;
float errorFreq = 1000;
long pulseStarts = 0;
long pulsEnds = 0;

float calculatePulse(float liftIn) {
  float calculatedPulseLength = 0;
  if (digitalRead(STF_MODE) == LOW) {
    if (liftIn > 0.5 && liftIn < 7) {
      calculatedPulseLength = (3.5 - (44100 / (48000 / (liftIn * 0.5)))) * 100;
    }
    else if (liftIn > 7) {
      calculatedPulseLength = 30;
    }
    else {
      calculatedPulseLength = (44100 / (48000 * 2));
    }
    return calculatedPulseLength;
  }
  if (digitalRead(STF_MODE) == HIGH) {
    if (liftIn > 0.5 && liftIn < 7) {
      calculatedPulseLength = (4 - (44100 / (48000 / (liftIn * 0.2)))) * 100;
    }
    else if (liftIn > 7) {
      calculatedPulseLength = 270;
    }
    else {
      calculatedPulseLength = (44100 / (48000 * 2));
    }
    return calculatedPulseLength;
  }
}

float calculateNewFreq(float newValue, float oldValue) {
  if (digitalRead(STF_MODE) == LOW && (newValue >= 0) && (newValue < 8) && (newValue != oldValue)) {
    freqValue = (350 + (120 * newValue));
  }
  else if (digitalRead(STF_MODE) == LOW && (newValue < 0) && (newValue > -8) && (newValue != oldValue)) {
    freqValue = (350 / (1 - (0.85 * newValue)));
  }
  else if (digitalRead(STF_MODE) == LOW && (newValue > 8) && (newValue != oldValue)) {
    freqValue = 1310;
  }
  else if (digitalRead(STF_MODE) == HIGH && (newValue >= 0) && (newValue < 8) && (newValue != oldValue)) {
    freqValue = (350 + (30 * newValue));
  }
  else if (digitalRead(STF_MODE) == HIGH && (newValue < 0) && (newValue > -8) && (newValue != oldValue)) {
    freqValue = (350 / (1 - (0.1 * newValue)));
  }
  else if (digitalRead(STF_MODE) == HIGH && (newValue > 8) && (newValue != oldValue)) {
    freqValue = 600;
  }
  freqValueInc = (freqValue - freqValueOld) / 8;
  freqValueOld = freqValue;
  return freqValue;
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  gen.Begin();
  gen.ApplySignal(SINE_WAVE, REG0, freqValueOld);
  gen.EnableOutput(true);   // Turn ON the output - it defaults to OFF
  AD9833 gen(FNC_PIN);       // Defaults to 25MHz internal reference frequency
  pinMode(STF_MODE, OUTPUT);
  pinMode(STF_AUTO, OUTPUT);
  pinMode(XC_WK, INPUT_PULLUP);
  pinMode(Varioschalter, INPUT_PULLUP);
  pinMode(STFSchalter, INPUT_PULLUP);
  pinMode(STFAuto, INPUT_PULLUP);

  xTaskCreate(Sound, "Create Sound", 1000, NULL, 50, &SoundTask);
}

void loop() {

  int varioSchalter_state, stfSchalter_state, stfAuto_state, xc_WK_state;
  varioSchalter_state = digitalRead(Varioschalter);
  stfSchalter_state = digitalRead(STFSchalter);
  stfAuto_state = digitalRead(STFAuto);
  xc_WK_state = digitalRead(XC_WK);

  char Data;
  String DataString;
  if (Serial2.available()) {
    Data = Serial2.read();
    if (Data == '$') {
      while (Data != 10) {
        DataString += Data;
        Data = Serial2.read();
      }
      //Serial.println(DataString);
    }
    if (DataString.startsWith("$PFV")) {
      //Serial2.println(DataString);
      int pos = DataString.indexOf(',');
      DataString.remove(0, pos + 1);
      int pos1 = DataString.indexOf(',');                //findet den Ort des ersten ,
      String variable = DataString.substring(0, pos1);      //erfasst den ersten Datensatz
      int pos2 = DataString.indexOf('*', pos1 + 1 );     //findet den Ort des *
      String wert = DataString.substring(pos1 + 1, pos2);   //erfasst den zweiten Datensatz

      //
      //Analyse des Steigwertes
      //
      if (variable == "VAR") {
        var = wert.toFloat();
      }

      //
      //Analyse des aktuellen Modus
      //
      if (variable == "MOD") {
        mod = wert;
      }

      //
      //Analyse der true Airspeed
      //
      if (variable == "TAS") {
        tas = wert.toFloat();
      }

      //
      //Analyse Speed to Fly
      //
      if (variable == "STF") {
        stf = wert.toFloat();
      }
    }
    DataString = "";
    vTaskDelay(20);
  }

  if (((varioSchalter_state == 1) && (stfSchalter_state == 1) && (xc_WK_state == 0) && (stfAuto_state == 1)) ||
      ((varioSchalter_state == 1) && (stfSchalter_state == 1) && (xc_WK_state == 1) && (mod == "C"))) {
    digitalWrite(STF_MODE, LOW);
    digitalWrite(STF_AUTO, HIGH);
  }
  else if (((varioSchalter_state == 1) && (stfSchalter_state == 1) && (xc_WK_state == 0) && (stfAuto_state == 0)) ||
           ((varioSchalter_state == 1) && (stfSchalter_state == 1) && (xc_WK_state == 1) && (mod == "S"))) {
    digitalWrite(STF_MODE, HIGH);
    digitalWrite(STF_AUTO, HIGH);
  }
  else if ((varioSchalter_state == 1) && (stfSchalter_state == 0)) {
    digitalWrite(STF_MODE, HIGH);
    digitalWrite(STF_AUTO, LOW);
  }
  else if ((varioSchalter_state == 0) && (stfSchalter_state == 1)) {
    digitalWrite(STF_MODE, LOW);
    digitalWrite(STF_AUTO, LOW);
  }
}

void Sound(void *) {
  while (true) {
    sf = (tas - stf) / 10;

    if (digitalRead(STF_MODE) == LOW && var > 0.5) {
      float startTimePulse = millis();
      float pulseTime = 0;
      while (pulseTime < calculatePulse(var)) {
        freqValueNeg = (-1 * freqValueOld) / 8;
        int  i = 0;
        while (i < 8 && freqValueNeg < 0) {
          i = i + 1;
          gen.IncrementFrequency (REG0, freqValueNeg);
          delay(1);
        }
        gen.ApplySignal(SINE_WAVE, REG0, 0);
        freqValueOld = 0;
        pulseTime = millis() - startTimePulse;
      }
      do  {
        calculateNewFreq(var, varOld);
        int  i = 0;
        while (i < 8) {
          i = i + 1;
          gen.IncrementFrequency (REG0, freqValueInc);
          delay(1);
        }
        gen.ApplySignal(SINE_WAVE, REG0, freqValue);
        pulseTime = millis() - startTimePulse;
      } while (pulseTime < (calculatePulse(var) + (calculatePulse(var) / 2)));
    }
    else if (digitalRead(STF_MODE) == LOW && var <= 0.5) {
      calculateNewFreq(var, varOld);
      int  i = 0;
      while (i < 8) {
        i = i + 1;
        gen.IncrementFrequency (REG0, freqValueInc);
        delay(1);
      }
      gen.ApplySignal(SINE_WAVE, REG0, freqValue);
    }
    else if (digitalRead(STF_MODE) == HIGH && sf > 0.5) {
      float startTimePulse = millis();
      float pulseTime = 0;
      while (pulseTime < 20) {
        freqValueNeg = (-1 * freqValueOld) / 8;
        int  i = 0;
        while (i < 8 && freqValueNeg < 0) {
          i = i + 1;
          gen.IncrementFrequency (REG0, freqValueNeg);
          delay(1);
        }
        gen.ApplySignal(SINE_WAVE, REG0, 0);
        freqValueOld = 0;
        pulseTime = millis() - startTimePulse;
      }
      do  {
        calculateNewFreq(var, varOld);
        int  i = 0;
        while (i < 8) {
          i = i + 1;
          gen.IncrementFrequency (REG0, freqValueInc);
          delay(1);
        }
        gen.ApplySignal(SINE_WAVE, REG0, freqValue);
        pulseTime = millis() - startTimePulse;
      } while (pulseTime < (calculatePulse(var) + 20));
    }
    else if (digitalRead(STF_MODE) == HIGH && sf < -0.5) {
      calculateNewFreq(var, varOld);
      int  i = 0;
      while (i < 8) {
        i = i + 1;
        gen.IncrementFrequency (REG0, freqValueInc);
        delay(1);
      }
      gen.ApplySignal(SINE_WAVE, REG0, freqValue);
    }
    else if (digitalRead(STF_MODE) == HIGH && sf > -0.5 && sf < 0.5) {
      gen.ApplySignal(SINE_WAVE, REG0, 0);
    }
  }
  varOld = var;
  sfOld = sf;
  vTaskDelay(10);
}
