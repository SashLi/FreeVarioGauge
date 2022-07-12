//    OpenVarioGauge is a programm to generate the vario display using NMEA Output
//    of OpenVario.
//    Copyright (C) 2019  Dirk Jung Blaubart@gmx.de
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) anyt later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

//************************************************
//****  Screen and SPIFFS Headers and Defines ****
//************************************************
//#include<HardwareSerial.h>
#include<SPI.h>
#include<TFT_eSPI.h>
#include <Preferences.h>
Preferences prefs;

#include "LogoOV.h"
#include "FS.h"
#include "SPIFFS.h"
#include <ESP32Encoder.h>

ESP32Encoder Vario_Enc;

#define BLACK   0x0000
#define RED     0x001F
#define BLUE    0x9806
#define GREEN   0x07E0
#define CYAN    0xFFE0
#define MAGENTA 0xF81F
#define YELLOW  0x07FF
#define WHITE   0xFFFF
#define GREY    0x632c
#define RXD2 16
#define TXD2 17
#define VE_PB 27
#define DEG2RAD 0.0174532925
#define STF_MODE 13
#define STF_AUTO 33
#define OuterRadius 160
#define InnerRadius 130
#define xCenter 160
#define yCenter 160

static TFT_eSPI tft = TFT_eSPI();
TFT_eSprite nameOfField = TFT_eSprite(&tft);
TFT_eSprite infoLarge = TFT_eSprite(&tft);
TFT_eSprite infoSmall = TFT_eSprite(&tft);

TaskHandle_t SerialScanTask, TaskEncoder, TaskValueRefresh, ArcRefreshTask;

SemaphoreHandle_t xTFTSemaphore;

const String SOFTWARE_VERSION = "  V1.1.5 - 2022";

static String mod;
static String mce;
static String nameSetting = "QNH";
static String nameSpeed = "GS";
static String nameHight = "MSL";
static String valueSetting, valueSpeed, valueHight;
static String unitSpeed, unitHight, unitSetting;
static String stf_mode;
static String valueQnhAsString = "1013";
static String valueBugAsString = "0";
static String valueMuteAsString = "ON";
static String valueAttenAsString = "2";
static String valueGrsAsString = "0";
static String valueTasAsString = "0";
static String valueVaaAsString = "0.0";
static String valueHigAsString = "0";
static String valueHagAsString = "0";
static String valueMacAsString = "0.0";
static String menuSpeedColor = "WHITE";
static String menuHightColor = "WHITE";
static String menuValueColor = "WHITE";

extern uint16_t logoOV[];

static float var = 0;
static float valueVaaAsFloat = 0;
static float valueTasAsFloat = 0;
static float valueGrsAsFloat = 0;
static float valueMacAsFloat = 0.5;
static float valueHagAsFloat = 0;
static float valueHigAsFloat = 0;
static float tem = 0;
static float stf = 0.0;

static double stfValue = 0;
static double valueQnhAsFloat = 1013;
static double valueBugAsFloat = 0;

static bool varWasUpdated = true;
static bool vaaWasUpdated = true;
static bool tasWasUpdated = true;
static bool grsWasUpdated = true;
static bool mcWasUpdated =  true;
static bool hagWasUpdated = true;
static bool higWasUpdated = true;
static bool tempWasUpdated = true;
static bool qnhWasUpdated = true;
static bool bugWasUpdated = true;
static bool attWasUpdated = true;
static bool muteWasUpdated = true;
static bool stfModeWasUpdate = true;
static bool serial2Error = false;

bool showBootscreen = true;
bool mci = false;

int stf_mode_state;

int spriteNameWidthSpeed, spriteValueWidthSpeed, spriteunitWidthSpeed;
int spriteNameWidthHight, spriteValueWidthHight, spriteunitWidthHight;
int spriteNameWidthSetting, spriteValueWidthSetting, spriteunitWidthSetting;
int startAngle, segmentDraw, segmentCountOld, segmentCount;
int valueMuteAsInt = 1;
int valueAttenAsInt = 2;

static int requestDrawMenu = 0;
static int requestDrawMenuLevel = 0;
static bool requestMenuPaint = false;
static bool requestMenuFontPaint = false;

static unsigned long lastTimeBoot = 0;
static unsigned long lastTimeReady = 0;
static unsigned long lastTimeModeWasSend = 0;
static unsigned long lastTimeSerial2 = 0;

void setup() {
  // Enable the weak pull down resistors
  ESP32Encoder::useInternalWeakPullResistors = true;

  if ( xTFTSemaphore == NULL )
  { xTFTSemaphore = xSemaphoreCreateMutex();
    if ( ( xTFTSemaphore ) != NULL )
      xSemaphoreGive( ( xTFTSemaphore ) );
  }

  tft.init();
  tft.setRotation(0);

  // set starting count value
  Vario_Enc.setCount(16380);
  Vario_Enc.attachHalfQuad(23, 32);
  pinMode(VE_PB, INPUT_PULLUP);

  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200, SERIAL_8N1);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  SPIFFSstart();

  xTaskCreate(SerialScan, "Serial Scan", 5000, NULL, 50, &SerialScanTask);
  xTaskCreate(EncoderReader, "Encoder Task", 5000, NULL, 80, &TaskEncoder);
  xTaskCreate(ValueRefresh, "Value Refresh", 5000, NULL, 40, &TaskValueRefresh);

  prefs.begin("settings", false);
  valueMuteAsInt = prefs.getUInt("Mute", 1);
  valueAttenAsInt = prefs.getUInt("ATTEN", 2);
  nameSpeed = prefs.getString("nameSpeed", "GS");
  nameHight = prefs.getString("nameHight", "MSL");
  prefs.end();

  valueAttenAsString = String(valueAttenAsInt);

  if (valueMuteAsInt == 0) {
    valueMuteAsString = "OFF";
    muteWasUpdated = true;
  }
  else if (valueMuteAsInt == 1) {
    valueMuteAsString = "ON";
    muteWasUpdated = true;
  }
}

void loop() {

  if (showBootscreen) {
    showBootScreen(SOFTWARE_VERSION, tft);
  } else {
    ArcRefresh();
  }
}

void showBootScreen(String versionString, TFT_eSPI tftIN) {

  String waitingMessage = "Waiting for XCSoar ...";
  String dataString;
  int serial2IsReady = 0;
  tft.loadFont("micross20");

  TFT_eSprite bootSprite = TFT_eSprite(&tftIN);
  bootSprite.loadFont("micross20_boot");
  bootSprite.createSprite(195, 25);
  bootSprite.fillSprite(WHITE);
  bootSprite.setCursor(0, 2);
  bootSprite.setTextColor(GREY, BLACK);
  tftIN.fillScreen(WHITE);
  tftIN.setWindow(40, 55, 40 + 193, 55 + 155);
  tftIN.pushColors(logoOV, 194 * 156);
  bootSprite.println(versionString);
  bootSprite.pushSprite(40, 245);
  bootSprite.deleteSprite();
  lastTimeBoot = millis();
  vTaskDelay(4800);
  bootSprite.createSprite(195, 25);
  bootSprite.fillSprite(WHITE);
  bootSprite.setCursor(0, 2);
  bootSprite.println(waitingMessage);
  bootSprite.pushSprite(40, 245);
  bootSprite.deleteSprite();

  //Waiting until XCSoar delivers correct values
  do {
    if (Serial2.available()) {
      char serialString = Serial2.read();
      if (serialString == '$') {
        while (serialString != 10) {
          dataString += serialString;
          serialString = Serial2.read();
        }
      }
      if (dataString.startsWith("$PFV")) {
        serial2IsReady = 1;
        dataString = "";
      }
    }
  } while (serial2IsReady == 0);
  bootSprite.unloadFont();
  tftIN.fillScreen(BLACK);
  lastTimeReady = millis();
  showBootscreen = false;
}

void EncoderReader(void *p) {
  const int DEBOUNCE_DELAY = 80;
  const long LONGPRESS_TIME = 500;
  const long TIME_SINCE_BOOT = 5000;
  const long NOT_SET = -1;

  const int MENU_SPEED_TYP = 1;
  const int MENU_HIGHT_TYP = 2;
  const int MENU_VALUE_TYP = 3;

  const int MENU_VALUE_QNH = 1;
  const int MENU_VALUE_BUG = 2;
  const int MENU_VALUE_MUTE = 3;

  float encoderPosition = (float) - 999;

  long pushButtonPressTime = NOT_SET;
  float menuActiveSince = 0;                  // Will be updated in menu run

  bool pushButtonPressed = false;
  bool pushButtonIsLongpress = false;
  bool menuWasTriggered = false;
  bool subMenuTriggered = false;
  bool subMenuLevelTwoTriggered = false;

  int selectedMenu = MENU_SPEED_TYP;
  int selectedLevelTwoMenu = MENU_VALUE_QNH;
  while (true) {
    // READ ENCODER & BUTTONS / DIPS

    bool encoderLeft = false;
    bool encoderRight = false;
    bool encoderWasMoved = false;
    long encoderPositionNew = Vario_Enc.getCount();
    long timeSystemRuns = millis() - lastTimeBoot;

    if (encoderPositionNew > encoderPosition) {
      encoderRight = true;
      encoderWasMoved = true;
    }
    else if (encoderPositionNew < encoderPosition) {
      encoderLeft = true;
      encoderWasMoved = true;
    }
    else {
      encoderWasMoved = false;
    }
    encoderPosition = encoderPositionNew;

    if (menuWasTriggered && encoderRight) {
      selectedMenu ++;
    }
    else if  (menuWasTriggered && encoderLeft) {
      selectedMenu --;
    }

    if (selectedMenu > MENU_VALUE_TYP ) {
      selectedMenu = MENU_SPEED_TYP;
    }
    else if (selectedMenu < MENU_SPEED_TYP) {
      selectedMenu = MENU_VALUE_TYP;
    }

    if (digitalRead(VE_PB) == LOW && pushButtonIsLongpress == false) {
      pushButtonPressed = true;
    }
    else {
      pushButtonPressed = false;
    }

    // Check if Long or Shortpress
    // set Time when button was pressed else set to zero or do nothing when already pressed
    if (pushButtonPressed && pushButtonPressTime == NOT_SET) {
      pushButtonPressTime = millis();
      pushButtonPressed = true;
    }
    else if (pushButtonPressed && pushButtonPressTime != NOT_SET) {}
    else {
      pushButtonPressTime = NOT_SET;
      pushButtonPressed = false;
      vTaskDelay(DEBOUNCE_DELAY);
    }
    if (pushButtonPressTime != NOT_SET && (millis() - pushButtonPressTime >= LONGPRESS_TIME) && menuWasTriggered == false) {
      pushButtonIsLongpress = true;
      pushButtonPressed = false;
    } else {
      pushButtonIsLongpress = false;
    }

    if (digitalRead(STF_MODE) == LOW) {
      if (stf_mode != "Vario") {
        stfModeWasUpdate = true;
        if (digitalRead(STF_AUTO) == LOW) {
          Serial2.println("$POV,C,VAR*4F");  //Vario-Mode
        }
      }
      stf_mode = "Vario";
    }
    else if (digitalRead(STF_MODE) == HIGH) {
      if (stf_mode != "STF") {
        stfModeWasUpdate = true;
        if (digitalRead(STF_AUTO) == LOW) {
          Serial2.println("$POV,C,STF*4B");  //STF-Mode
        }
      }
      stf_mode = "STF";
    }

    // *********** CALCULATE ACTIONS ********************
    // **************************************************
    if (pushButtonIsLongpress && !pushButtonPressed && !menuWasTriggered && !subMenuTriggered && !encoderWasMoved && timeSystemRuns > TIME_SINCE_BOOT) {
      menuWasTriggered = true;
      menuActiveSince = millis(); // set time to now
      // Wait for release of pushButton
      while (digitalRead(VE_PB) == LOW) {}
      setDrawMenuLevel(selectedMenu, 1);
      requestFontRepaint();
    }

    else if (!pushButtonIsLongpress && !pushButtonPressed && !menuWasTriggered && !subMenuTriggered && !subMenuLevelTwoTriggered && encoderWasMoved) {
      changeMCvalue(encoderRight);
    }

    else if (!pushButtonIsLongpress && !pushButtonPressed && menuWasTriggered && !subMenuTriggered && encoderWasMoved) {

      if (selectedMenu == MENU_SPEED_TYP) {
        setDrawMenuLevel(selectedMenu, 1);
        requestFontRepaint();
      }
      else if (selectedMenu == MENU_HIGHT_TYP) {
        setDrawMenuLevel(selectedMenu, 1);
        requestFontRepaint();
      }
      else if (selectedMenu == MENU_VALUE_TYP) {
        setDrawMenuLevel(selectedMenu, 1);
        requestFontRepaint();
      }
      menuActiveSince = millis(); // set time to now
    }
    else if (menuWasTriggered && !subMenuTriggered && !pushButtonIsLongpress && pushButtonPressed) {
      // Wait for release of pushButton
      while (digitalRead(VE_PB) == LOW) {}
      if (selectedMenu == MENU_SPEED_TYP) {
        menuWasTriggered = false;
        subMenuTriggered = true;
        DrawMenu(selectedMenu, 2);
      }
      else if (selectedMenu == MENU_HIGHT_TYP) {
        menuWasTriggered = false;
        subMenuTriggered = true;
        DrawMenu(selectedMenu, 2);;

      }
      else if (selectedMenu == MENU_VALUE_TYP) {
        menuWasTriggered = false;
        subMenuTriggered = true;
        settingStartValueType();
        DrawMenu(selectedMenu, 2);
      }
      menuActiveSince = millis(); // set time to now
    }
    else if (!menuWasTriggered && subMenuTriggered && !pushButtonIsLongpress && !pushButtonPressed &&  encoderWasMoved ) {
      if (selectedMenu == MENU_SPEED_TYP) {
        changeSpeedOption();
      }
      else if (selectedMenu == MENU_HIGHT_TYP) {
        changeHighOption();
      }
      else if (selectedMenu == MENU_VALUE_TYP && encoderRight ) {
        changeValueOptionRight();
      }
      else if (selectedMenu == MENU_VALUE_TYP && encoderLeft ) {
        changeValueOptionLeft();
      }
      menuActiveSince = millis(); // set time to now
    }

    else if (!menuWasTriggered && subMenuTriggered && !pushButtonIsLongpress && pushButtonPressed ) {
      // Wait for release of pushButton
      while (digitalRead(VE_PB) == LOW) {}

      if (selectedMenu == MENU_SPEED_TYP || selectedMenu  == MENU_HIGHT_TYP) {
        subMenuTriggered = false;
        selectedMenu = MENU_SPEED_TYP;
        setDrawMenuLevel(selectedMenu, 0);
        requestFontRepaint();
      }
      else if (selectedMenu == MENU_VALUE_TYP) {
        menuActiveSince = millis(); // set time to now
        subMenuTriggered = false;
        subMenuLevelTwoTriggered = true;
        setDrawMenuLevel(selectedMenu, 3);
        requestFontRepaint();
      }
    }

    else if (!menuWasTriggered && !subMenuTriggered && !pushButtonIsLongpress  && !pushButtonPressed && subMenuLevelTwoTriggered && encoderWasMoved) {
      changeLevelTwoMenu(encoderRight);
      changeLevelTwoMenuTurn(encoderWasMoved);
      menuActiveSince = millis(); // set time to now
    }

    else if (!menuWasTriggered && !subMenuTriggered && !pushButtonIsLongpress  && pushButtonPressed && subMenuLevelTwoTriggered && !encoderWasMoved) {
      // Wait for release of pushButton
      while (digitalRead(VE_PB) == LOW) {}
      subMenuLevelTwoTriggered = false;
      settingStandardValueType();
      selectedMenu = MENU_SPEED_TYP;
      setDrawMenuLevel(selectedMenu, 0);
      requestFontRepaint();
    }

    // check run time in menu and exit if time > 10000
    if ((millis() - menuActiveSince) > 10000 && subMenuLevelTwoTriggered) {
      subMenuLevelTwoTriggered = false;
      settingStandardValueType();
      selectedMenu = MENU_SPEED_TYP;
      setDrawMenuLevel(selectedMenu, 0);
      requestFontRepaint();
    }
    else if ((millis() - menuActiveSince) > 10000 && subMenuTriggered) {
      subMenuTriggered = false;
      settingStandardValueType();
      selectedMenu = MENU_SPEED_TYP;
      setDrawMenuLevel(selectedMenu, 0);
      requestFontRepaint();
    }
    else if ((millis() - menuActiveSince) > 10000 && menuWasTriggered) {
      menuWasTriggered = false;
      settingStandardValueType();
      selectedMenu = MENU_SPEED_TYP;
      setDrawMenuLevel(selectedMenu, 0);
      requestFontRepaint();
    }
  }
}

void DrawMenu(int selectedMenuNumber, int level) {

  setDrawMenuLevel(selectedMenuNumber, level);
  requestMenuPaint = true;
  requestMenuFontPaint = true;

}

void setDrawMenuLevel(int selectedMenuNumber, int level) {
  requestDrawMenu = selectedMenuNumber;
  requestDrawMenuLevel = level;
}

void requestFontRepaint() {
  requestMenuFontPaint = true;
}

void changeMCvalue(bool mcUp) {
  if (mci == true) {
    mce = ("$PFV,M,S," + String((float)valueMacAsFloat) + "*");
    int checksum = calculateChecksum(mce);
    Serial2.printf("%s%X\n", mce.c_str(), checksum); //set MCE to MCI
    mci = false;
  }
  if (mcUp) {
    Serial2.println("$PFV,M,U*58");  //McCready Up
  }
  else {
    Serial2.println("$PFV,M,D*49");  //McCready Down
  }
  nameSetting = "MC";
  mcWasUpdated = true;
}

void changeSpeedOption () {
  if (nameSpeed == "TAS") {
    nameSpeed = "GS";
    grsWasUpdated = true;
  }
  else {
    nameSpeed = "TAS";
    tasWasUpdated = true;
  }
  prefs.begin("settings", false);
  prefs.putString("nameSpeed", nameSpeed);
  prefs.end();
}

void changeHighOption () {
  if (nameHight == "AGL") {
    nameHight = "MSL";
    higWasUpdated = true;
  }
  else {
    nameHight = "AGL";
    hagWasUpdated = true;
  }
  prefs.begin("settings", false);
  prefs.putString("nameHight", nameHight);
  prefs.end();
}

void changeValueOptionRight () {
  if ( nameSetting == "Bug") {
    nameSetting = "ATTEN";
    attWasUpdated = true;
  }
  else if ( nameSetting == "QNH") {
    nameSetting = "Bug";
    bugWasUpdated = true;
  }
  else if ( nameSetting == "ATTEN") {
    nameSetting = "Mute";
    muteWasUpdated = true;
  }
  else if ( nameSetting == "Mute") {
    nameSetting = "QNH";
    qnhWasUpdated = true;
  }
}

void changeValueOptionLeft () {
  if ( nameSetting == "Bug") {
    nameSetting = "QNH";
    qnhWasUpdated = true;
  }
  else if ( nameSetting == "QNH") {
    nameSetting = "Mute";
    muteWasUpdated = true;
  }
  else if ( nameSetting == "ATTEN") {
    nameSetting = "Bug";
    bugWasUpdated = true;
  }
  else if ( nameSetting == "Mute") {
    nameSetting = "ATTEN";
    attWasUpdated = true;
  }
}
void changeLevelTwoMenu (bool changeLevelTwoValue) {
  if (nameSetting == "QNH") {
    if (changeLevelTwoValue && valueQnhAsFloat < 1300) {
      valueQnhAsFloat = valueQnhAsFloat + 1;
    }
    else if (!changeLevelTwoValue && valueQnhAsFloat > 850) {
      valueQnhAsFloat = valueQnhAsFloat - 1;
    }
    String qnhStr = ("$PFV,Q,S," + String(valueQnhAsFloat) + "*");
    int checksum = calculateChecksum(qnhStr);
    char buf[20];
    // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
    valueQnhAsString = dtostrf(valueQnhAsFloat, 4, 0, buf);
    qnhWasUpdated = true;
    Serial2.printf("%s%X\n", qnhStr.c_str(), checksum);                //send QNH to XCSoar
  }

  if (nameSetting == "Bug") {
    if (changeLevelTwoValue && valueBugAsFloat < 50) {
      valueBugAsFloat = valueBugAsFloat + 1;
    }
    else if (!changeLevelTwoValue && valueBugAsFloat > 0) {
      valueBugAsFloat = valueBugAsFloat - 1;
    }
    String bugStr = ("$PFV,B,S," + String(valueBugAsFloat) + "*");
    int checksum = calculateChecksum(bugStr);
    char buf[20];
    // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
    valueBugAsString = dtostrf(valueBugAsFloat, 2, 0, buf);
    bugWasUpdated = true;
    Serial2.printf("%s%X\n", bugStr.c_str(), checksum);                //send bug to XCSoar
  }
  if (nameSetting == "ATTEN") {
    if (changeLevelTwoValue && valueAttenAsInt < 3) {
      valueAttenAsInt = valueAttenAsInt + 1;
    }
    else if (!changeLevelTwoValue && valueAttenAsInt > 0) {
      valueAttenAsInt = valueAttenAsInt - 1;
    }
    valueAttenAsString = String(valueAttenAsInt);
    prefs.begin("settings", false);
    prefs.putUInt("ATTEN", valueAttenAsInt);
    prefs.end();
    attWasUpdated = true;
    String attStr = ("$PFV,A,S," + valueAttenAsString + "*");
    int checksum = calculateChecksum(attStr);
    char buf[20];
    // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
    Serial2.printf("%s%X\n", attStr.c_str(), checksum);
  }
}
void changeLevelTwoMenuTurn (bool changeLevelTwoValue) {
  if (nameSetting == "Mute") {
    if (changeLevelTwoMenuTurn && valueMuteAsInt == 1) {
      valueMuteAsInt = 0;
      valueMuteAsString = "OFF";
    }
    else if (changeLevelTwoMenuTurn && valueMuteAsInt == 0) {
      valueMuteAsInt = 1;
      valueMuteAsString = "ON";
    }
    prefs.begin("settings", false);
    prefs.putUInt("Mute", valueMuteAsInt);
    prefs.end();
    muteWasUpdated = true;
    String muteStr = ("$PFV,S,S," + String(valueMuteAsInt) + "*");
    int checksum = calculateChecksum(muteStr);
    char buf[20];
    // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
    Serial2.printf("%s%X\n", muteStr.c_str(), checksum);
  }
}

void settingStartValueType () {
  nameSetting = "QNH";
  qnhWasUpdated = true;
}

void settingStandardValueType () {
  nameSetting = "MC";
  mcWasUpdated = true;
}


//void ArcRefresh(void *p) {
void ArcRefresh() {

  while (showBootscreen) {
    vTaskDelay(1000);
  }

  if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
  {
    float angle = (var * 22) + 180;
    DrawArc(angle, var, stf, valueTasAsFloat);
    xSemaphoreGive(xTFTSemaphore);
  }
  vTaskDelay(10);
}

void ValueRefresh(void *parameter) {

  while (showBootscreen) {
    vTaskDelay(1000);
  }
  while (true) {
    if (nameHight == "MSL") {
      valueHight = valueHigAsString;
    }
    else if (nameHight == "AGL") {
      valueHight = valueHagAsString;
    }
    if (nameSpeed == "GS") {
      valueSpeed = valueGrsAsString;
    }
    else if (nameSpeed == "TAS") {
      valueSpeed = valueTasAsString;
    }

    if (requestMenuPaint) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        if (requestDrawMenuLevel == 0) {
          tft.drawRect(47, 111, 166, 37, BLACK);
          tft.drawRect(46, 110, 168, 39, BLACK);
          tft.drawRect(45, 109, 170, 41, BLACK);

          tft.drawRect(65, 154, 148, 37, BLACK);
          tft.drawRect(64, 153, 150, 39, BLACK);
          tft.drawRect(63, 152, 152, 41, BLACK);

          tft.drawRect(66, 197, 147, 37, BLACK);
          tft.drawRect(65, 196, 149, 39, BLACK);
          tft.drawRect(64, 195, 151, 41, BLACK);
        }
        else if (requestDrawMenuLevel == 2) {
          if (requestDrawMenu == 1) {
            tft.drawRect(47, 111, 166, 37, RED);
            tft.drawRect(46, 110, 168, 39, RED);
            tft.drawRect(45, 109, 170, 41, RED);
          }

          if (requestDrawMenu == 2) {
            tft.drawRect(65, 154, 148, 37, RED);
            tft.drawRect(64, 153, 150, 39, RED);
            tft.drawRect(63, 152, 152, 41, RED);
          }

          if (requestDrawMenu == 3) {
            tft.drawRect(66, 197, 147, 37, RED);
            tft.drawRect(65, 196, 149, 39, RED);
            tft.drawRect(64, 195, 151, 41, RED);
          }
        }
        else if (requestDrawMenuLevel == 3) {
          tft.drawRect(66, 197, 147, 37, BLACK);
          tft.drawRect(65, 196, 149, 39, BLACK);
          tft.drawRect(64, 195, 151, 41, BLACK);

          tft.drawLine(64, 233, 214, 233, RED);
          tft.drawLine(64, 234, 214, 234, RED);
          tft.drawLine(64, 235, 214, 235, RED);
        }
        xSemaphoreGive(xTFTSemaphore);
      }
    }

    //void DrawInfo(TFT_eSprite fontOfName, TFT_eSprite fontOfInfo, String spriteName, String value,
    //String unit, int spriteNameWidth, int spriteValueHight, int spriteValueWidth, int spriteunitWidth, int x, int y)
    if (vaaWasUpdated) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        DrawInfo(nameOfField, infoLarge, WHITE, "large", "Avg.", valueVaaAsString, "", 34, 40, 94, 0, 78, 60);
        vaaWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }
    if (tasWasUpdated || grsWasUpdated || requestMenuFontPaint ) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        if ((requestDrawMenuLevel == 1 ) && (requestDrawMenu == 1) || (requestDrawMenuLevel == 2 ) && (requestDrawMenu == 1)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", nameSpeed, valueSpeed, "km/h", 28, 25, 57, 68, 53, 118);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", nameSpeed, valueSpeed, "km/h", 28, 25, 57, 68, 53, 118);
        }
        tasWasUpdated = false;
        grsWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }

    if (hagWasUpdated || higWasUpdated || requestMenuFontPaint) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        if ((requestDrawMenuLevel == 1 ) && (requestDrawMenu == 2) || (requestDrawMenuLevel == 2 ) && (requestDrawMenu == 2)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", nameHight, valueHight, "m", 31, 25, 74, 29, 72, 161);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", nameHight, valueHight, "m", 31, 25, 74, 29, 72, 161);
        }
        hagWasUpdated = false;
        higWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }

    if (nameSetting == "MC" && (mcWasUpdated || requestFontRepaint)) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        valueSetting = valueMacAsString;
        if ((requestDrawMenuLevel == 1 ) && (requestDrawMenu == 3)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", "MC", valueSetting, "m/s", 24, 25, 56, 53, 73, 204);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", "MC", valueSetting, "m/s", 24, 25, 56, 53, 73, 204);
        }
        mcWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }
    else if (nameSetting == "QNH" && qnhWasUpdated) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        valueSetting = valueQnhAsString;
        if ((requestDrawMenuLevel == 2 ) && (requestDrawMenu == 3) || (requestDrawMenuLevel == 3 ) && (requestDrawMenu == 3)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", "QNH", valueSetting, "", 50, 25, 82, 1, 73, 204);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", "QNH", valueSetting, "", 50, 25, 82, 1, 73, 204);
        }
        qnhWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }
    else if (nameSetting == "Bug" && bugWasUpdated) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        valueSetting = valueBugAsString;
        if ((requestDrawMenuLevel == 2 ) && (requestDrawMenu == 3) || (requestDrawMenuLevel == 3 ) && (requestDrawMenu == 3)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", "Bug", valueSetting, "%", 39, 25, 63, 31, 73, 204);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", "Bug", valueSetting, "%", 39, 25, 63, 31, 73, 204);
        }
        bugWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }
    else if (nameSetting == "ATTEN" && attWasUpdated) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        valueSetting = valueAttenAsString;
        if ((requestDrawMenuLevel == 2 ) && (requestDrawMenu == 3) || (requestDrawMenuLevel == 3 ) && (requestDrawMenu == 3)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", "ATT", valueSetting, "", 39, 25, 63, 31, 73, 204);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", "ATT", valueSetting, "", 39, 25, 63, 31, 73, 204);
        }
        attWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }
    else if (nameSetting == "Mute" && muteWasUpdated) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        valueSetting = valueMuteAsString;
        if ((requestDrawMenuLevel == 2 ) && (requestDrawMenu == 3) || (requestDrawMenuLevel == 3 ) && (requestDrawMenu == 3)) {
          DrawInfo(nameOfField, infoSmall, RED, "small", "Mute", valueSetting, "", 39, 25, 63, 31, 73, 204);
        }
        else {
          DrawInfo(nameOfField, infoSmall, WHITE, "small", "Mute", valueSetting, "", 39, 25, 63, 31, 73, 204);
        }
        muteWasUpdated = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }

    if (stfModeWasUpdate) {
      if ( xSemaphoreTake( xTFTSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        DrawInfo(nameOfField, infoSmall, WHITE, "small", "Mode", stf_mode, "", 41, 25, 70, 0, 95, 248);
        stfModeWasUpdate = false;
        xSemaphoreGive(xTFTSemaphore);
      }
    }

    vTaskDelay(15);
  }
  if (requestMenuPaint) {
    requestMenuPaint = false;
    requestMenuFontPaint = false;
  }
}

void SerialScan (void *p) {
  Serial.println("Serial Scan Task Created");
  char serialString;
  int pos, pos1, pos2;
  while (1) {
    String dataString;
    if (Serial2.available()) {
      serialString = Serial2.read();
      if (serialString == '$') {
        long timeSystemReady = millis() - lastTimeReady;
        while (serialString != 10) {
          dataString += serialString;
          serialString = Serial2.read();
          if (dataString.length() > 300) {
            dataString = "ERROR";
            Serial.println("Break serial Read!");
            break;
          }
        }
        if (timeSystemReady < 2000) {
          Serial2.println("$PFV,M,S,0.5*59");
          Serial2.println("$PFV,Q,S,1013*6D");
        }
        //Serial.println(dataString);
      }
    }

    if (dataString.startsWith("$PFV")) {
      if (serial2Error == true) {
        serial2Error = false;
        Serial.println("Error detected");

        // synchronize QNH after reboot of OV
        String qnhStr = ("$PFV,Q,S," + String(valueQnhAsFloat) + "*");
        int checksumQnh = calculateChecksum(qnhStr);
        String checksumQnhAsString = String (checksumQnh, HEX);
        String syncQnh = (qnhStr + checksumQnhAsString);
        Serial2.println(syncQnh);

        // synchronize Bug after reboot of OV
        String bugStr = ("$PFV,B,S," + String(valueBugAsFloat) + "*");
        int checksumBug = calculateChecksum(bugStr);
        String checksumBugAsString = String (checksumBug, HEX);
        String syncBug = (bugStr + checksumBugAsString);
        Serial2.println(syncBug);

        // synchronize MC after reboot of OV
        String mcStr = ("$PFV,M,S," + valueMacAsString + "*");
        int checksumMc = calculateChecksum(mcStr);
        String checksumMcAsString = String (checksumMc, HEX);
        String synMc = (mcStr + checksumMcAsString);
        Serial2.println(synMc);
      }

      lastTimeSerial2 = millis();
      //Serial2.println(DataString);
      int pos = dataString.indexOf(',');
      dataString.remove(0, pos + 1);
      int pos1 = dataString.indexOf(',');                   //findet den Ort des ersten ,
      String variable = dataString.substring(0, pos1);      //erfasst den ersten Datensatz
      int pos2 = dataString.indexOf('*', pos1 + 1 );        //findet den Ort des *
      String wert = dataString.substring(pos1 + 1, pos2);   //erfasst den zweiten Datensatz
      float wertAsFloat = wert.toFloat();                   // der Wert als float

      //
      //analyse vertical speed
      //


      if (variable == "VAR") {
        if (var !=  wertAsFloat) {
          varWasUpdated = true;
        }
        var = wertAsFloat;
      }

      //
      //analyse average vertical speed
      //
      else if (variable == "VAA") {
        if (valueVaaAsFloat !=  wertAsFloat) {
          vaaWasUpdated = true;
        }
        valueVaaAsFloat = wertAsFloat;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        if (valueVaaAsFloat >= 0) {
          valueVaaAsString = dtostrf(abs(valueVaaAsFloat), 3, 1, buf);
          valueVaaAsString = "+" + valueVaaAsString;
        }
        else {
          valueVaaAsString = dtostrf(abs(valueVaaAsFloat), 3, 1, buf);
          valueVaaAsString = "-" + valueVaaAsString;
        }
      }

      //
      //analyse internal McCready value
      //
      else if (variable == "MCI") {

        if (valueMacAsFloat !=  wertAsFloat) {
          mcWasUpdated = true;
        }
        valueMacAsFloat = wertAsFloat;
        mci = true;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        valueMacAsString = dtostrf(valueMacAsFloat, 3, 1, buf);
      }

      //
      //analyse external McCready value
      //
      else if ((variable == "MCE") && (mci == false)) {
        if (valueMacAsFloat !=  wertAsFloat) {
          mcWasUpdated = true;
        }
        valueMacAsFloat = wertAsFloat;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        valueMacAsString = dtostrf(valueMacAsFloat, 3, 1, buf);
      }

      //
      //analyse current XCSoar mode
      //
      else if (variable == "MOD") {
        mod = wert;
      }

      //
      //analyse speed to fly
      //
      else if (variable == "STF") {
        stfValue = wert.toFloat();
        int FF = (valueAttenAsInt * 10) + 1;
        stf = filter(stfValue, FF);
      }

      //
      //analyse true airspeed
      //
      else if (variable == "TAS") {

        if (valueTasAsFloat != wertAsFloat) {
          tasWasUpdated = true;
        }
        valueTasAsFloat = wertAsFloat;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        valueTasAsString = dtostrf(valueTasAsFloat, 3, 0, buf);
      }

      //
      //analyse groundspeed
      //
      else if (variable == "GRS") {
        if (valueGrsAsFloat != wertAsFloat) {
          grsWasUpdated = true;
        }
        valueGrsAsFloat = wertAsFloat;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        valueGrsAsString = dtostrf(valueGrsAsFloat, 3, 0, buf);
      }

      //
      //analyse hight MSL
      //
      else if (variable == "HIG") {
        if (valueHigAsFloat != wertAsFloat) {
          higWasUpdated = true;
        }
        valueHigAsFloat = wertAsFloat;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        valueHigAsString = dtostrf(valueHigAsFloat, 4, 0, buf);
      }

      //
      //analyse hight about ground level
      //
      else if (variable == "HAG") {
        if (valueHagAsFloat != wertAsFloat) {
          hagWasUpdated = true;
        }
        valueHagAsFloat = wertAsFloat;
        char buf[20];
        // dtostrf(floatvar, stringlength, digits_after_decimal, charbuf);
        valueHagAsString = dtostrf(valueHagAsFloat, 4, 0, buf);
      }

      //
      //analyse temperatur
      //
      else if (variable == "TEM") {
        if (tem != wertAsFloat) {
          tempWasUpdated = true;
        }
        tem = wertAsFloat;
      }

      //
      //analyse QNH
      //
      else if (variable == "QNH") {
        if ((valueQnhAsFloat != wertAsFloat) && (wertAsFloat < 2000)) {
          qnhWasUpdated = true;
        }
        valueQnhAsFloat = wertAsFloat;
        valueQnhAsString = String(valueQnhAsFloat, 0);
      }

      //
      //analyse bug
      //
      else if (variable == "BUG") {
        if (valueBugAsFloat != wertAsFloat) {
          bugWasUpdated = true;
        }
        valueBugAsFloat = wertAsFloat;
        valueBugAsString = wert;
      }

      //
      //analyse Mute
      //
      else if (variable == "MUT") {
        muteWasUpdated = true;
        valueMuteAsInt = wert.toInt();
      }

      //
      //analyse Attenuation
      //
      else if (variable == "ATT") {
        attWasUpdated = true;
        valueAttenAsInt = wert.toInt();
      }
    }

    else if ((millis() - lastTimeSerial2) > 10000) {
      serial2Error = true;
    }
    dataString = "";
    vTaskDelay(20);
  }
}

//**************************
//****  Filter for STF  ****
//**************************
float filter(float filteredSTF, uint16_t filterfactor) {
  static uint16_t count = 0;
  // so that at the beginning the value is close to the measured value
  if (count < filterfactor) {
    filterfactor = count++;
  }
  stf = ((stf * filterfactor) + filteredSTF) / (filterfactor + 1);
  return stf;
}

//********************************
//****  calculate Checksumme  ****
//********************************
int calculateChecksum(String mce) {
  int i, XOR, c;
  for (XOR = 0, i = 0; i < mce.length(); i++) {
    c = (unsigned char)mce.charAt(i);
    if (c == '*') break;
    if ((c != '$') && (c != '!')) XOR ^= c;
  }
  return XOR;
}

//*********************************
//****  Deg to Rad conversion  ****
//*********************************
float deg2rad(float * angle) {
  //  float tempangle=*angle;
  * angle = * angle / 180 * 3.141516;
  //  return angle;
}

//****************************
//****  Calculate an arc  ****
//****************************
// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// yx = y axis radius
// w  = width (thickness) of arc in pixels
// color = 16 bit color value
// Note if rx and ry are the same an arc of a circle is drawn

double sf;                                      //Speedfaktor zur Berechnung des STF-Tons
float B, B_alt;
float MiddleRadius = ((OuterRadius - InnerRadius) / 2) + InnerRadius; //Middle of Sliding Circle radius

int fillArc(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int color)
{
  stf_mode_state = digitalRead(STF_MODE);
  byte seg = 3; // Segments are 3 degrees wide = 120 segments for 360 degrees
  byte inc = 3; // Draw segments every 3 degrees, increase to 6 for segmented ring

  // Calculate first pair of coordinates for segment start
  float sx = cos((start_angle - 90) * DEG2RAD);
  float sy = sin((start_angle - 90) * DEG2RAD);

  uint16_t x0 = sx * (rx - w) + x;
  uint16_t x1 = sx * rx + x;
  uint16_t y0;
  uint16_t y1;

  if (B > 0) {
    y0 = sy * (ry - w) + y;
    y1 = sy * ry + y;
  }
  else {
    y0 = 320 - (sy * (ry - w) + y);
    y1 = 320 - (sy * ry + y);
  }
  // Draw color blocks every inc degrees
  for (int i = start_angle; i < start_angle + seg * seg_count; i += inc) {

    // Calculate pair of coordinates for segment end
    uint16_t y2;
    uint16_t y3;
    float sx2 = cos((i + seg - 90) * DEG2RAD);
    float sy2 = sin((i + seg - 90) * DEG2RAD);
    uint16_t x2 = sx2 * (rx - w) + x;
    uint16_t x3 = sx2 * rx + x;
    if (B > 0) {
      y2 = sy2 * (ry - w) + y;
      y3 = sy2 * ry + y;
    }
    else {
      y2 = -sy2 * (ry - w) + y;
      y3 = -sy2 * ry + y;
    }
    tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);

    // Copy segment end to sgement start for next segment
    x0 = x2;
    y0 = y2;
    x1 = x3;
    y1 = y3;
  }
}
void DrawArc(float inangle, float liftValue, double speedToFly, float trueAirSpeed) {
  stf_mode_state = digitalRead(STF_MODE);
  unsigned int color;


  //*********************************
  //****  Vario Mode Colored Arc ****
  //*********************************
  if (stf_mode_state == 0) {
    if (liftValue <= -6) B = -6;
    if ((liftValue > -6) && (liftValue < 6)) B = liftValue;
    if (liftValue >= 6) B = 6;
  }

  //********************************
  //****  STF Mode colored Arc  ****
  //********************************
  if (stf_mode_state == 1) {
    sf = (trueAirSpeed - speedToFly) / 10;
    if (sf <= -6) B = -6;
    if ((sf > -6) && (sf < 6)) B = sf;
    if (sf >= 6) B = 6;
  }

  //*******************
  //**** Draw Arc  ****
  //*******************
  segmentCount = abs(B) * 7.4;
  deg2rad(&inangle);
  if (B < 0 && B_alt > 0) {
    startAngle = 90;
    segmentDraw = 60;
    color = BLACK;
    segmentCountOld = 0;
  }
  else if (B > 0 && B_alt < 0) {
    startAngle = 90;
    segmentDraw = 60;
    color = BLACK;
    segmentCountOld = 0;
  }
  else if ((B >= 0) && (segmentCountOld > segmentCount)) {
    startAngle = 270 + (3 * segmentCount);
    segmentDraw = segmentCountOld - segmentCount;
    segmentCountOld = segmentCount;
    //segmentCount = segmentCount + 2;
    color = BLACK;
  }
  else if (B >= 0) {
    startAngle = 270 + 3 * segmentCountOld;
    //startAngle = 270;
    segmentDraw = segmentCount - segmentCountOld;
    color = GREEN;
    segmentCountOld = segmentCount;
  }
  else if ((B < 0) && (segmentCountOld > segmentCount)) {
    startAngle = 270 + (3 * segmentCount);
    segmentDraw = segmentCountOld - segmentCount;
    segmentCountOld = segmentCount;
    //segmentCount = segmentCount + 2;
    color = BLACK;
  }
  else if (B < 0) {
    startAngle = 270 + (3 * segmentCountOld);
    //startAngle = 270;
    segmentDraw = segmentCount - segmentCountOld;
    color = RED;
    segmentCountOld = segmentCount;
  }

  fillArc(160, 160, startAngle, segmentDraw, 160, 160, 30, color);
  B_alt = B;

  //****  Draw divisions and numbers  ****
  for (int i = 70; i <= 300; i += 22) {
    float divangle = i;
    deg2rad(&divangle);
    int x0 = OuterRadius * cos(divangle) + xCenter;
    int y0 = OuterRadius * sin(divangle) + yCenter;
    int x1 = (MiddleRadius + 10) * cos(divangle) + xCenter;
    int y1 = (MiddleRadius + 10) * sin(divangle) + yCenter;
    tft.drawLine(x0, y0, x1, y1, WHITE);
  }
  tft.setTextColor(WHITE, GREY);
  tft.setCursor(12, 152);
  tft.println("0");
  tft.setCursor(25, 97);
  tft.println("1");
  tft.setCursor(53, 52);
  tft.println("2");
  tft.setCursor(94, 22);
  tft.println("3");
  tft.setCursor(148, 9);
  tft.println("4");
  tft.setCursor(206, 18);
  tft.println("5");
  tft.setCursor(25, 203);
  tft.println("1");
  tft.setCursor(53, 248);
  tft.println("2");
  tft.setCursor(94, 280);
  tft.println("3");
  tft.setCursor(148, 294);
  tft.println("4");
  tft.setCursor(206, 287);
  tft.println("5");
}

//***********************************
//**** Draw ValueBoxes and Data  ****
//***********************************
void DrawInfo(TFT_eSprite fontOfName, TFT_eSprite fontOfInfo, uint32_t color, String infoType, String spriteName, String value, String unit, int spriteNameWidth, int spriteValueHight, int spriteValueWidth, int spriteunitWidth, int x, int y) {
  fontOfName.loadFont("micross15");
  fontOfName.createSprite(spriteNameWidth, 25);
  fontOfName.setCursor(0, 2);
  fontOfName.setTextColor(color, BLACK);
  fontOfName.setTextSize(2);
  fontOfName.println(spriteName);
  fontOfName.pushSprite(x, y);
  fontOfName.deleteSprite();
  fontOfName.unloadFont();

  if (infoType == "small") {
    fontOfInfo.loadFont("micross30");
  }
  else {
    fontOfInfo.loadFont("micross50");
  }
  fontOfInfo.createSprite(spriteValueWidth, spriteValueHight);
  fontOfInfo.setTextColor(color, BLACK);
  fontOfInfo.setTextSize(3);
  fontOfInfo.setTextDatum(TR_DATUM);
  fontOfInfo.drawString(value, spriteValueWidth, 2);
  fontOfInfo.pushSprite(x + spriteNameWidth, y);
  fontOfInfo.deleteSprite();

  fontOfInfo.createSprite(spriteunitWidth, 25);
  fontOfInfo.setTextColor(color, BLACK);
  fontOfInfo.setTextSize(3);
  fontOfInfo.setTextDatum(TR_DATUM);
  fontOfInfo.drawString(unit, spriteunitWidth, 2);
  fontOfInfo.pushSprite(x + spriteNameWidth + spriteValueWidth, y);
  fontOfInfo.deleteSprite();
  fontOfInfo.unloadFont();
}

// ************************************
// ****  Initialize SPIFFS memory  ****
// ************************************
void SPIFFSstart() {
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
    while (1) yield(); // Stay here twiddling thumbs waiting
  }
  Serial.println("\r\nInitialisation done.");
}
