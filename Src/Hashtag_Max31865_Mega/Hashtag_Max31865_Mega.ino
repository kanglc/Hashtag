/**
 * Project Hashtag
 * By Kang Liat Chuan, Oct 2022
 * 
 * Hardware:
 * 1. Arduino Mega
 *
 */

// Libraries
#include <Adafruit_MAX31865.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include "DFRobot_LedDisplayModule.h"
#include <SD.h>
#include <SPI.h>

// Definitions
// RTC
#define countof(a) (sizeof(a) / sizeof(a[0]))
#define RTC_CLK 16
#define RTC_DAT 15
#define RTC_RST 14
// Max31865
#define MAXCLK    8
#define MAXDO     9
#define MAXDI     10
#define MAXCS     11
#define RREF      430.0
#define RNOMINAL  100.0
// LEDs
#define LED_R_pin 34
#define LED_Y_pin 35
#define LED_G_pin 36
// Switches
#define Switch_Mode  29
#define Switch_ONOFF 28
// Relay
#define Relay     22
#define Relay_ON  0
#define Relay_OFF 1
// Temperatures
#define t_upper -21
#define t_lower -23
// SD
#define SD_MOSI 51
#define SD_SS   53
#define SD_SCK  52
#define SD_MISO 50
#define loginterval 60000

// Constants and Variables
bool Mode = false;
bool Manual_ON_OFF = false;
bool LED_R = false;
bool LED_Y = false;
bool LED_G = false;
float t;
bool cooling = true;
unsigned long current_millis;
unsigned long last_millis;
String buffer;
File myFile;

// Used by RTC
void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

// Constructors
// Max31865
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAXCS, MAXDI, MAXDO, MAXCLK);

// RTC
ThreeWire myWire(RTC_DAT, RTC_CLK, RTC_RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

// DFRobot_LedDisplayModule Constructor
DFRobot_LedDisplayModule LED(&Wire, 0x48);

void setup() {

  // Setup Serial Terminal
  Serial.begin(9600);

  // Initialize variables
  Mode = false;
  Manual_ON_OFF = false;
  LED_R = false;
  LED_Y = false;
  LED_G = false;
  current_millis = millis();
  last_millis = millis();

  // Setup Display
  while(LED.begin(LED.e4Bit) != 0)
  {
    Serial.println("Failed to initialize the chip , please confirm the chip connection!");
    delay(1000);
  }
  LED.setDisplayArea(1,2,3,4);

  // Setup RTC
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  if (!Rtc.IsDateTimeValid()) 
  {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println(F("RTC lost confidence in the DateTime!"));
      Rtc.SetDateTime(compiled);
  }
  if (Rtc.GetIsWriteProtected())
  {
      Serial.println(F("RTC was write protected, enabling writing now"));
      Rtc.SetIsWriteProtected(false);
  }
  if (!Rtc.GetIsRunning())
  {
      Serial.println(F("RTC was not actively running, starting now"));
      Rtc.SetIsRunning(true);
  }
  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) 
  {
      Serial.println(F("RTC is older than compile time!  (Updating DateTime)"));
      Rtc.SetDateTime(compiled);
  }
  else if (now > compiled) 
  {
      Serial.println(F("RTC is newer than compile time. (this is expected)"));
  }
  else if (now == compiled) 
  {
      Serial.println(F("RTC is the same as compile time! (not expected but all is fine)"));
  }

  // Setup Max31865
  //Serial.println("Adafruit MAX31865 PT100 Sensor Test");
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  Serial.println("Max31865 PT100 started");

  // Setup Relay
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, Relay_OFF);

  // Setup Mode and ONOFF pines
  pinMode(Switch_Mode, INPUT);
  pinMode(Switch_ONOFF, INPUT);

  // Setup LED
  pinMode(LED_R_pin, OUTPUT);
  pinMode(LED_Y_pin, OUTPUT);
  pinMode(LED_G_pin, OUTPUT);
  digitalWrite(LED_R_pin, LED_R);
  digitalWrite(LED_Y_pin, LED_Y);
  digitalWrite(LED_G_pin, LED_G);

  // Setup microSD
//  buffer.reserve(128);
//  if (!SD.begin(SD_SS)) {
//    Serial.println(F("microSD initialization failed!"));
//    while (1);
//  }
//  SD.remove("Hashtag.txt");
//  myFile = SD.open("Hashtag.txt", FILE_WRITE);
//  if (myFile) {
//    Serial.print(F("Writing to text file ..."));
//    myFile.println("Hashtag data");
//    Serial.println("Done");
//    myFile.close();
//  } else {
//    Serial.println(F("error opening file"));
//    while (1);
//  }


} // void setup


void loop() {

  /**
  * Read State of Mode and Manual_ON_OFF switches
  */
  Mode = digitalRead(Switch_Mode);
  Manual_ON_OFF = digitalRead(Switch_ONOFF);

  /**
  * Get time from RTC
  */ 
  RtcDateTime now = Rtc.GetDateTime();

  /**
  * Get temperature from Max31865
  */
  t = thermo.temperature(RNOMINAL, RREF);
  if (t < 0) {
    LED_G = true;
  } else {
    LED_G = false;
  }
  digitalWrite(LED_G_pin, LED_G);

  /**
  * Display temperature on display and serial terminal
  */
  LED.print(abs(round(t*100)/100.00));
  //delay(1000);
  Serial.print(now.Day()); Serial.print(now.Month()); Serial.print(now.Year()-2000); Serial.print(" ");
  Serial.print(now.Hour()); Serial.print(now.Minute()); Serial.print(":");
  Serial.print(" Temp = "); Serial.println(t);

  /**
  * Output to Relay
  */ 
  if (Mode) { // Manual Mode

     LED_Y = true;
     if (Manual_ON_OFF) {
        digitalWrite(Relay, Relay_ON);
	      LED_R = true;
     } else {
        digitalWrite(Relay, Relay_OFF);
	      LED_R = false;
     }

  } else { // Auto Mode

     LED_Y = false;
     if (cooling) {
        digitalWrite(Relay, Relay_ON);
        LED_R = true;
        if (t <= t_lower) {
           digitalWrite(Relay, Relay_OFF);
           LED_R = false;
           cooling = false;
        }
     } else {
        digitalWrite(Relay, Relay_OFF);
        LED_R = false;
        if (t >= t_upper) {
           digitalWrite(Relay, Relay_ON);
           LED_R = true;
           cooling = true;
        }
     }
  } // if (Mode)
  digitalWrite(LED_R_pin, LED_R);
  digitalWrite(LED_Y_pin, LED_Y);

  /**
  * Write to microSD
  */
  // check if it's been over loginterval since the last line added
//  current_millis = millis();
//  if ((current_millis - last_millis) >= loginterval) {
//     buffer = now.Day();
//     buffer += "/";
//     buffer += now.Month();
//     buffer += "/";
//     buffer += now.Year()-2000;
//     buffer += " ";
//     buffer += now.Hour();
//     buffer += now.Minute();
//     buffer += " Temperature = ";
//     buffer += t;
//     buffer += " degC";
//     buffer += "\r\n";
//     myFile = SD.open("Hashtag.txt", FILE_WRITE);
//     Serial.print(F("Writing to file: "));
//     Serial.println(buffer.c_str());
//     if (myFile) {
//       myFile.println(buffer.c_str());
//       Serial.println(F("Written to file."));
//       myFile.close();
//     } else {
//       Serial.println(F("Error opening file!"));
//     }
//     last_millis = millis();
//     Serial.print("last_millis = "); Serial.println(last_millis);
//  } // if ((current_millis - last_millis) >= loginterval)


} // void loop
