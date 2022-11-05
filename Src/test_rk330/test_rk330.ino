/*
 * test rk330 t-h-p sensor via max485 module
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

// DS1302 CONNECTIONS:
// DS1302 CLK/SCLK --> 5
// DS1302 DAT/IO --> 7
// DS1302 RST/CE --> 2
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND
ThreeWire myWire(7,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

//16 characters and 2 lines of show
DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);

// Constants
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
unsigned long u32wait;
float t, h, p;
const int DE_RE = 8;

//Create a SoftwareSerial object so that we can use software serial. Search "software serial" on Arduino.cc to find out more details.
SoftwareSerial mySerial(9, 10);

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
  
  // Setup LCD
  lcd.init();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Temp");
  lcd.setCursor(6,0); lcd.print("Humid");
  lcd.setCursor(12,0); lcd.print("Pres");
  
  //Serial.begin(9600); // start terminal via usb

  // Setup Modbus
  mySerial.begin(9600); // start software serial
  master.start(); // start the ModBus object
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  t = 0.0; h = 0.0; p = 0.0;
  
  // Setup RTC
  Rtc.Begin();
  
  
}

void loop() {

  /**
  * State Machine for reading data from RK330-02
  */
  switch( u8state ) {
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
        t = au16data[0]/10.0;
        h = au16data[1]/10.0;
        p = au16data[2]/10.0;
        //Serial.print("Temperature = "); Serial.print(t);
        //Serial.print(" degC, Humidity = "); Serial.print(h);
        //Serial.print(" %RH, Pressure = "); Serial.print(p); Serial.println(" kPa");
        lcd.setCursor(0,1); lcd.print(t);
        lcd.setCursor(6,1); lcd.print(h);
        lcd.setCursor(12,1); lcd.print(p);
    }
    break;
  }
}

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
