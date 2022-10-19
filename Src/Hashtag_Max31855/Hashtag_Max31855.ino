/**
 * Project Hashtag
 * By Kang Liat Chuan, Sep 2022.
 * 
 * Hardware:
 * 1. Arduino Uno
 * 2. Max485-to-TTL module
 * 3. RK330-02 sensor (provided by Hashtag)
 * 4. Relay module
 * 5. Solenoid valve (provided by Hashtag)
 * 6. DS1302 RTC module
 * 7. DFR0464 LCD RGB Backlight Module
 * 8. DFR0229 microSD module
 * 9. Power Splitter
 * 10. DHT22 temperature and humidity sensor
 *
 * Software (libraries):
 * 1. Modbus-Master-Slave-for-Arduino-master
 *    Modbus master example 2:
 *    The purpose of this example is to query an array of data
 *    from an external Modbus slave device.
 *    This example is similar to "simple_master", but this example
 *    allows you to use software serial instead of hardware serial 
 * 2. DFRobot_RGBLCD1602
 * 3. Rtc_by_Makuna
 * 4. SoftwareSerial
 * 5. ThreeWire
 * 6. SD
 * 7. SPI
 * 8. DHT
 *
 * RTC (DS1302) Connections:
 * DS1302 CLK/SCLK - 5
 * DS1302 DAT/IO - 7
 * DS1302 RST/CE - 2
 * DS1302 VCC - +5v
 * DS1302 GND - GND
 *
 * Log:
 * 01 Sep 2022 RK330 temperature/humidity/pressure sensor working
 * 30 Sep 2022 DFR0464 LCD RGB Backlight Module working
 * 07 Oct 2022 DS1302 RTC Module working
 * 09 Oct 2022 New platform working. Manual mode pins working
 * 14 Oct 2022 DHT22 temperature and humidity sensor working
 * 17 Oct 2022 Max31855 temperature sensor working
 * 
 */

// Libraries
#include <SD.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "DFRobot_RGBLCD1602.h"
#include <ThreeWire.h>
#include <RtcDS1302.h>

// Definitions

// RTC
#define countof(a) (sizeof(a) / sizeof(a[0]))
#define RTC_CLK 5
#define RTC_DAT 7
#define RTC_RST 2
// Max31855
#define MAXDO   8
#define MAXCS   9
#define MAXCLK  10
// SD
// MOSI - pin 11
// MISO - pin 12
// CLK - pin 13
// CS - pin 6 
#define SD_CS 6
// Relay
#define IN1 3
#define IN2 4
#define Manual_Mode_pin A2
#define Manual_ON_pin A3
#define Relay_ON 0
#define Relay_OFF 1
// Temperature boundaries
#define t20 20
#define t10 10
#define t0 0
#define t_10 -10
#define t_20 -21
#define t_25 -23
// ON times
#define t20_ON 30
#define t10_ON 30
#define t0_ON 20
#define t_10_ON 10
#define t_20_ON 5
#define t_25_ON 5
// OFF times
#define t20_OFF 10
#define t10_OFF 10
#define t0_OFF 10
#define t_10_OFF 10
#define t_20_OFF 10
#define t_25_OFF 15
// LCD
#define LCD_row1 0
#define LCD_row2 1

// Constants and Variables
//unsigned long current_millis;
//unsigned long seconds_millis;
unsigned long lastMillis;
float t, h, p;
bool Mode = false;
bool Manual_ON_OFF = false;
bool cooling = true;
//int16_t t_ON = 0;
//int16_t t_OFF = 0;
uint8_t sec = 0;
bool hysteresis = false;
bool Auto_ON_OFF = false;
String buffer;

// Max31855
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// RTC
ThreeWire myWire(RTC_DAT, RTC_CLK, RTC_RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

// 16 characters and 2 lines
DFRobot_RGBLCD1602 lcd(16, 2); // lcdCols, lcdRows


void setup() {

  // Setup Serial Terminal
  Serial.begin(9600);

  // Setup Relay
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, Relay_OFF);
  digitalWrite(IN2, Relay_OFF);
  pinMode(Manual_Mode_pin, INPUT);
  pinMode(Manual_ON_pin, INPUT);

  // Setup microSD
  buffer.reserve(64);
  if (!SD.begin(SD_CS)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("initialization done."));
  SD.remove("Hashtag.txt");
  File myFile = SD.open("Hashtag.txt", FILE_WRITE);
  if (myFile) {
    Serial.print(F("Writing to text file ..."));
    myFile.println("Hashtag data");
    // close the file:
    myFile.close();
    Serial.println("Done");
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening file"));
    while (1);
  }

  // Setup LCD
  lcd.init();
  lcd.clear();
  lcd.setRGB(0, 0, 70);
  
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

  // Setup Max31855
  Serial.println(F("MAX31855 test"));
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print(F("Initializing Max31855 sensor..."));
  if (!thermocouple.begin()) {
    Serial.println("ERROR");
    while (1) delay(10);
  }
  // OPTIONAL: Can configure fault checks as desired (default is ALL)
  // Multiple checks can be logically OR'd together.
  // thermocouple.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);
  // short to GND fault is ignored
  Serial.println(F("MAX31855 test DONE"));

  // Initialize variables
  //unsigned long current_millis = millis();
  //seconds_millis = millis();
  lastMillis = millis();
  t = 0.0; h = 0.0; p = 0.0;
  cooling = true;
  //t_ON = 0;
  //t_OFF = 0;
  sec = 0;
  hysteresis = false;
  Auto_ON_OFF = false;


} // void setup


void loop() {

  /**
  * Read State of Mode and Manual_ON_OFF switches
  */
  Mode = digitalRead(Manual_Mode_pin);
  Manual_ON_OFF = digitalRead(Manual_ON_pin);

  /**
  * Get temperature from Max31855
  */
  t = thermocouple.readCelsius();
  //delay(1000);
  // no need to delay because the (RTC+LCD) codes below got more than 2 seconds of delay
  //Serial.print("Temperature: "); Serial.println(t);

  /**
  * Get time from RTC
  */ 
  RtcDateTime now = Rtc.GetDateTime();
  //printDateTime(now);
  //Serial.println();

  /**
  * Output to Serial Terminal for debugging
  */ 
  //Serial.print(now.Day()); Serial.print(now.Month()); Serial.print(now.Year()-2000); Serial.print(" ");
  //Serial.print(now.Hour()); Serial.print(now.Minute()); Serial.print(":");
  //Serial.print(" Temp:"); Serial.print(t);
  //Serial.print(" Humid:"); Serial.print(h);
  //Serial.print(" Pressure:"); Serial.println(p);
  //Serial.print(au16data[0]); Serial.print(",");
  //Serial.print(au16data[1]); Serial.print(",");
  //Serial.print(au16data[2]); Serial.print(",");
  //Serial.print(au16data[3]); Serial.print(",");
  //Serial.print(au16data[4]); Serial.print(",");
  //Serial.print(au16data[5]); Serial.println();

  /**
  * Output to LCD
  */ 
  // Output Timestamp to LCD row 1
  // Day
  if (now.Day() < 10) {
     lcd.setCursor(0, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(1, LCD_row1); lcd.print(now.Day());
  } else {
     lcd.setCursor(0, LCD_row1); lcd.print(now.Day());
  }
  lcd.setCursor(2, LCD_row1); lcd.print("/");
  // Month
  if (now.Month() < 10) {
     lcd.setCursor(3, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(4, LCD_row1); lcd.print(now.Month());
  } else {
     lcd.setCursor(3, LCD_row1); lcd.print(now.Month());
  }
  lcd.setCursor(5, LCD_row1); lcd.print("/");
  // Year
  lcd.setCursor(6, LCD_row1); lcd.print(now.Year()-2000);
  // Hour
  if (now.Hour() < 10) {
     lcd.setCursor(9, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(10, LCD_row1); lcd.print(now.Hour());
  } else {
     lcd.setCursor(9, LCD_row1); lcd.print(now.Hour());
  }
  // Minutes
  if (now.Minute() < 10) {
     lcd.setCursor(11, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(12, LCD_row1); lcd.print(now.Minute());
  } else {
     lcd.setCursor(11, LCD_row1); lcd.print(now.Minute());
  }
  lcd.setCursor(14, LCD_row1);
  if (Mode) {
     lcd.print("M");
     lcd.setCursor(15, LCD_row1);
     if (Manual_ON_OFF) {
        lcd.print("1");
     } else {
        lcd.print("0");
     }
  } else {
     lcd.print("A");
     lcd.setCursor(15, LCD_row1); lcd.print(" ");
  }
  // Output LCD row 2
  //lcd.setCursor(0, LCD_row2); lcd.print("T:"); lcd.setCursor(2, LCD_row2); lcd.print(t);
  //lcd.setCursor(8, LCD_row2); lcd.print("P:"); lcd.setCursor(10, LCD_row2); lcd.print(p);
  lcd.setCursor(0, LCD_row2); lcd.print(t); lcd.setCursor(6, LCD_row2); lcd.print(",");
  lcd.setCursor(7, LCD_row2); lcd.print(h); lcd.setCursor(12, LCD_row2); lcd.print(",");
  lcd.setCursor(13, LCD_row2); lcd.print(p);

  /**
  * Output to Relay
  */ 
  if (Mode) { // Manual Mode

     if (Manual_ON_OFF) {
        digitalWrite(IN1, Relay_ON);
     } else {
        digitalWrite(IN1, Relay_OFF);
     }

  } else { // Auto Mode

     if (cooling) {
        digitalWrite(IN1, Relay_ON);
        if (t <= t_25) {
           digitalWrite(IN1, Relay_OFF);
	   cooling = false;
        }
     } else {
        digitalWrite(IN1, Relay_OFF);
        if (t >= t_20) {
           digitalWrite(IN1, Relay_ON);
	   cooling = true;
        }
     }

// -----------------------------------------------
//
//   // Set the ON and OFF times
//   if (t >= t20) {
//      t_ON = t20_ON; t_OFF = t20_OFF;
//      hysteresis = false;
//   } else if (t < t20 && t >= t10) {
//      t_ON = t10_ON; t_OFF = t10_OFF;
//      hysteresis = false;
//   } else if (t < t10 && t >= t0) {
//      t_ON = t0_ON; t_OFF = t0_OFF;
//      hysteresis = false;
//   } else if (t < t0 && t >= t_10) {
//      t_ON = t_10_ON; t_OFF = t_10_OFF;
//      hysteresis = false;
//   } else if (t < t_10 && t >= t_20) {
//      t_ON = t_20_ON; t_OFF = t_20_OFF;
//      hysteresis = false;
//   } else if (t < t_20 && t >= t_25) {
//      t_ON = t_25_ON; t_OFF = t_25_OFF;
//      hysteresis = true;
//   } else {
//      t_ON = 0; t_OFF = 60;
//      hysteresis = true;
//   }
//   t_ON = t_ON * 1000;
//   t_OFF = t_OFF * 1000;
//
//   // Display 1 second timer on serial terminal
//   if ((millis() - seconds_millis) >= 1000) {
//      sec += 1;
//      Serial.print(sec); Serial.println(" sec");
//      seconds_millis = millis();
//   }
//
//   if (hysteresis) {
//
//      if (cooling) {
//         digitalWrite(IN1, Relay_ON);
//         if (t <= t_25) {
//            digitalWrite(IN1, Relay_OFF);
//            cooling = false;
//         }
//      } else {
//         digitalWrite(IN1, Relay_OFF);
//         if (t >= t_20) {
//            digitalWrite(IN1, Relay_ON);
//            cooling = true;
//         }
//      }
//
//   } else {
//
//      if (Auto_ON_OFF) { // ON, turn OFF when t_ON reached
//      
//         // Add time to account for RTC+LCD codes delay above
//         if ((millis() - current_millis) >= t_ON + 4000) {
//            Serial.println(".......... OFF");
//            digitalWrite(IN1, Relay_OFF);
//            sec = 0;
//            Auto_ON_OFF = false;
//            current_millis = millis();
//         }
//      
//      } else { // OFF, turn ON when t_OFF reached
//      
//         // Add time to account for RTC+LCD codes delay above
//         if ((millis() - current_millis) >= t_OFF + 4000) {
//            Serial.println(".......... ON");
//            digitalWrite(IN1, Relay_ON);
//            sec = 0;
//            Auto_ON_OFF = true;
//            current_millis = millis();
//         }
//      
//      }
//
// -----------------------------------------------

  } // if (Mode)

  /**
  * Write to microSD
  */
  // check if it's been over 1000 ms since the last line added
  unsigned long current_millis = millis();
  if ((current_millis - lastMillis) >= 1000) {
     buffer = now.Day();
     buffer += "/";
     buffer += now.Month();
     buffer += "/";
     buffer += now.Year()-2000;
     buffer += " ";
     buffer += now.Hour();
     buffer += now.Minute();
     buffer += " Temperature = ";
     buffer += t;
     buffer += " degC";
     buffer += "\r\n";
     Serial.println(F("Writing to file: "));
     Serial.println(buffer.c_str());
     File myFile = SD.open("Hashtag.txt", FILE_WRITE);
     if (myFile) {
       myFile.println(buffer.c_str());
       myFile.close();
       Serial.println(F("Written to file."));
     } else {
       Serial.println(F("Error opening file!"));
     }
     lastMillis = millis();
  } // if ((current_millis - lastMillis) >= 1000)


} // void loop

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
