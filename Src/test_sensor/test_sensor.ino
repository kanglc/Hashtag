/**
 * For testing the temperature, humidity, and pressure sensor provided by Hastag
 * By Kang Liat Chuan, 1 Sep 2022
 * 
 * Hardware: Arduino Uno, Max485-to-TTL module, RK330-02 sensor (from Hashtag)
 *           PC running Arduino IDE - USB - Arduino Uno - Max485 - RK330-02
 *           Uno pin 4 - Max485 RE
 *           Uno pin 4 - Max485 DE
 *           Uno pin 10 - Max485 RO
 *           Uno pin 11 - Max485 DI
 *           Max485 Vcc - +5V
 *           Max485 Gnd - Ground
 *           Max485 A - RK330-02 A (yellow)
 *           Max485 B - RK330-02 B (green)
 *           +12v and ground connected to Uno and RK330-02 via a splitter
 *           DFR0464 LCD RGB Backlight Module
 *
 * Software: Modbus-Master-Slave-for-Arduino-master library (by smarmengol/Helium6072)
 *           Modbus master example 2:
 *           The purpose of this example is to query an array of data
 *           from an external Modbus slave device.
 *           This example is similar to "simple_master", but this example
 *           allows you to use software serial instead of hardware serial
 */

#include <ModbusRtu.h>
#include <SoftwareSerial.h>
#include "DFRobot_RGBLCD1602.h"

//16 characters and 2 lines of show
DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);

// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
float t, h, p;

//Create a SoftwareSerial object so that we can use software serial. Search "software serial" on Arduino.cc to find out more details.
SoftwareSerial mySerial(9, 10);

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, mySerial, 2); // this is master and RS-485 via software serial

/**
 * This structure contains a query to an slave device
 */
modbus_t telegram;

unsigned long u32wait;

void setup() {
  lcd.init();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Temp");
  lcd.setCursor(6,0); lcd.print("Humid");
  lcd.setCursor(12,0); lcd.print("Pres");
  //Serial.begin(9600); // start terminal via usb
  mySerial.begin(9600); // start software serial
  master.start(); // start the ModBus object
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  t = 0.0; h = 0.0; p = 0.0;
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
