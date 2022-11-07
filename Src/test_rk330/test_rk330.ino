/*
 * test rk330 THP sensor via max485 module
 * Modbus Connections:
 * Max485 RE - Uno pin 8 (DE_RE)
 * Max485 DE - Uno pin 8 (DE_RE)
 * Max485 RO - Uno pin 9 (rxPin)
 * Max485 DI - Uno pin 10 (txPin)
 * Max485 Vcc - +5V
 * Max485 Gnd - Ground
 * Max485 A - RK330-02 A (yellow)
 * Max485 B - RK330-02 B (green)
 *
 * Library: Modbus-Master-Slave-for-Arduino-master
 * example: software_serial_simple_master.ino
 *
 */

// Definitions

// Max485
#define DE_RE 8
//#define rxPin 15
//#define txPin 14

// RTC
#define countof(a) (sizeof(a) / sizeof(a[0]))
#define RTC_CLK 10
#define RTC_DAT 11
#define RTC_RST 12


// Libraries
#include <ModbusRtu.h>
#include "DFRobot_LedDisplayModule.h"
#include <ThreeWire.h>
#include <RtcDS1302.h>

// Constants

// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
unsigned long u32wait;
float t, h, p;
String dstr;


// Constructors

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, Serial3, DE_RE); // this is master and RS-485 via software serial

/**
 * This structure contains a query to an slave device
 */
modbus_t telegram;

/**
 * DFRobot_LedDisplayModule Constructor
 * In default, the IIC address of 4 bits digital tube is 0x48 
 */
DFRobot_LedDisplayModule LED(&Wire, 0x48);

// RTC
ThreeWire myWire(RTC_DAT, RTC_CLK, RTC_RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);


void setup() {
  
  // Setup Serial Terminal for debug
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

  // Setup Modbus
  Serial3.begin(9600); // hardware serial
  master.start(); // start the ModBus object
  master.setTimeOut(2000); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  t = 0.0; h = 0.0; p = 0.0;

  // Setup LED
  while(LED.begin(LED.e4Bit) != 0)
  {
    Serial.println("Failed to initialize the chip , please confirm the chip connection!");
    delay(1000);
  }
  LED.setDisplayArea(1,2,3,4);



} // void setup()


void loop() {

  /**
  * Get time from RTC
  */ 
  RtcDateTime now = Rtc.GetDateTime();

  /**
  * State Machine for reading data from RK330-02
  */
  switch (u8state) {
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

    master.query(telegram); // send query (only once)
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
        h = au16data[1]/10.0;
        p = au16data[2]/10.0;
        Serial.print(now.Day()); Serial.print("/");
	Serial.print(now.Month()); Serial.print("/");
	Serial.print(now.Year()-2000); Serial.print(" ");
        Serial.print(now.Hour()); Serial.print(":");
	Serial.print(now.Minute()); Serial.print(" ");
        Serial.print("Temp: "); Serial.print(t);
        Serial.print(" deg, Humid: "); Serial.print(h);
        Serial.print(" %RH, Press: "); Serial.print(p);
        Serial.println(" kPa");
    }
    break;
  } // switch (u8state)

  // Display on LED
  dstr = t;
  if (t >= 0) {
    // positive
    if (abs(t) < 10) {
      // single positive
      LED.print(" ", &dstr[0], &dstr[2], &dstr[3]);
    } else {
      // double positive
      LED.print(" ", &dstr[0], &dstr[1], &dstr[3]);
    }
  } else {
    // negative
    if (abs(t) < 10) {
      // single negative
      LED.print("-", &dstr[1], &dstr[3], &dstr[4]);
    } else {
      // double negative
      LED.print("-", &dstr[1], &dstr[2], &dstr[4]);
    }
  } // if (t >= 0)
  delay(1000);


} // void loop()


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

