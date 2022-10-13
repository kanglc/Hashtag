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
 *
 * Modbus Connections:
 * Max485 RE - Uno pin 8
 * Max485 DE - Uno pin 8
 * Max485 RO - Uno pin 9 (rx)
 * Max485 DI - Uno pin 10 (tx)
 * Max485 Vcc - +5V
 * Max485 Gnd - Ground
 * Max485 A - RK330-02 A (yellow)
 * Max485 B - RK330-02 B (green)
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
 * 
 */

// Definitions
#define countof(a) (sizeof(a) / sizeof(a[0]))

// Libraries
#include <ModbusRtu.h>
#include <SoftwareSerial.h>
#include "DFRobot_RGBLCD1602.h"
#include <ThreeWire.h>
#include <RtcDS1302.h>

// Constants
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
unsigned long u32wait;
float t, h, p;
int16_t t1;
const int DE_RE = 8;
const int rxPin = 9;
const int txPin = 10;
const int LCD_row1 = 0;
const int LCD_row2 = 1;
const int RTC_CLK = 5;
const int RTC_DAT = 7;
const int RTC_RST = 2;
const int IN1 = 3;
const int IN2 = 4;
const int Relay_ON = 0;
const int Relay_OFF = 1;
const int Manual_Mode_pin = A2;
const int Manual_ON_pin = A3;
int Mode = 0;
int ON_OFF = 0;
int cooling = 1;

ThreeWire myWire(RTC_DAT, RTC_CLK, RTC_RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

// 16 characters and 2 lines
DFRobot_RGBLCD1602 lcd(16, 2); // lcdCols, lcdRows

// Create a SoftwareSerial object so that we can use software serial.
SoftwareSerial mySerial(rxPin, txPin);

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, mySerial, DE_RE); // this is master and RS-485 via software serial

/**
 * This structure contains a query to an slave device
 */
modbus_t telegram;

void setup() {

  // Setup Relay
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, Relay_OFF);
  digitalWrite(IN2, Relay_OFF);
  pinMode(Manual_Mode_pin, INPUT);
  pinMode(Manual_ON_pin, INPUT);

  // Setup LCD
  lcd.init();
  lcd.clear();
  lcd.setRGB(0, 0, 70);
  
  // Setup Serial Terminal
  Serial.begin(9600);

  // Setup RTC
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);

  if (!Rtc.IsDateTimeValid()) 
  {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println("RTC lost confidence in the DateTime!");
      Rtc.SetDateTime(compiled);
  }

  if (Rtc.GetIsWriteProtected())
  {
      Serial.println("RTC was write protected, enabling writing now");
      Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning())
  {
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) 
  {
      Serial.println("RTC is older than compile time!  (Updating DateTime)");
      Rtc.SetDateTime(compiled);
  }
  else if (now > compiled) 
  {
      Serial.println("RTC is newer than compile time. (this is expected)");
  }
  else if (now == compiled) 
  {
      Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }


  // Setup Modbus
  mySerial.begin(9600); // start software serial
  master.start(); // start the ModBus object
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  t = 0.0; h = 0.0; p = 0.0; t1 = 0;

} // void setup

void loop() {

  /**
  * State Machine for reading data from RK330-02
  */
  switch(u8state) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  case 1: 
    /**
    * Refer to RK330-02 datasheet for specifications
    */
    telegram.u8id = 1; // slave address
    telegram.u8fct = 3; // function code (this one is registers read)
    telegram.u16RegAdd = 0; // start address in slave
    telegram.u16CoilsNo = 3; // number of elements (coils or registers) to read
    telegram.au16reg = au16data; // pointer to a memory array in the Arduino

    master.query( telegram ); // send query (only once)
    u8state++;
    break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 2000;
      //t = au16data[0]/10.0;
      // Taking care of negative temperatures
      if (au16data[0] > 32767) {
         t = (au16data[0]-65536.0)/10.0;
      } else {
         t = au16data[0]/10.0;
      }
      t1 = (int16_t)au16data[0];
      h = au16data[1]/10.0;
      p = au16data[2]/10.0;

    }
    break; // from case 2
  } // switch(u8state)

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
  //Serial.print(au16data[5]); Serial.print(",");
  //Serial.println(t1);

  /**
  * Read State of Mode and ON_OFF switches
  */
  Mode = digitalRead(Manual_Mode_pin);
  ON_OFF = digitalRead(Manual_ON_pin);

  /**
  * Output to LCD
  */ 
  // Output Date and Time info to LCD row1 - is there a better way to do this???
  if (now.Day() < 10) {
     lcd.setCursor(0, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(1, LCD_row1); lcd.print(now.Day());
  } else {
     lcd.setCursor(0, LCD_row1); lcd.print(now.Day());
  }
  lcd.setCursor(2, LCD_row1); lcd.print("/");
  if (now.Month() < 10) {
     lcd.setCursor(3, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(4, LCD_row1); lcd.print(now.Month());
  } else {
     lcd.setCursor(3, LCD_row1); lcd.print(now.Month());
  }
  lcd.setCursor(5, LCD_row1); lcd.print("/");
  lcd.setCursor(6, LCD_row1); lcd.print(now.Year()-2000);
  if (now.Hour() < 10) {
     lcd.setCursor(9, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(10, LCD_row1); lcd.print(now.Hour());
  } else {
     lcd.setCursor(9, LCD_row1); lcd.print(now.Hour());
  }
  if (now.Minute() < 10) {
     lcd.setCursor(11, LCD_row1); lcd.print("0"); // Insert leading 0
     lcd.setCursor(12, LCD_row1); lcd.print(now.Minute());
  } else {
     lcd.setCursor(11, LCD_row1); lcd.print(now.Minute());
  }
  lcd.setCursor(14, LCD_row1);
  if (Mode == 1) {
     lcd.print("M");
     lcd.setCursor(15, LCD_row1);
     if (ON_OFF == 1) {
        lcd.print("1");
     } else {
        lcd.print("0");
     }
  } else {
     lcd.print("A");
     lcd.setCursor(15, LCD_row1); lcd.print(" ");
  }
  // Output LCD row2
  //lcd.setCursor(0, LCD_row2); lcd.print("T:"); lcd.setCursor(2, LCD_row2); lcd.print(t);
  //lcd.setCursor(8, LCD_row2); lcd.print("P:"); lcd.setCursor(10, LCD_row2); lcd.print(p);
  lcd.setCursor(0, LCD_row2); lcd.print(t); lcd.setCursor(6, LCD_row2); lcd.print(",");
  lcd.setCursor(7, LCD_row2); lcd.print(h); lcd.setCursor(12, LCD_row2); lcd.print(",");
  lcd.setCursor(13, LCD_row2); lcd.print(p);

  /**
  * Output to Relay
  */ 
  if (Mode == 1) {

     // Manual Mode
     if (ON_OFF == 1) {
        digitalWrite(IN1, Relay_ON);
     } else {
        digitalWrite(IN1, Relay_OFF);
     }

  } else {

     // Auto Mode
     if (cooling == 1) {
        digitalWrite(IN1, Relay_ON);
        if (t <= -25.0) {
           digitalWrite(IN1, Relay_OFF);
	   cooling = 0;
        }
     } else {
        digitalWrite(IN1, Relay_OFF);
        if (t >= -20.0) {
           digitalWrite(IN1, Relay_ON);
	   cooling = 1;
        }
     }

  } // if (Mode == HIGH)

} // void loop

// Function used by RTC
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
