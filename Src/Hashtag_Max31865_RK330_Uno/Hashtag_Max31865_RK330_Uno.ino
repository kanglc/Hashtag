/**
 * Project Hashtag
 * By Kang Liat Chuan, 11 Nov 2022
 *
 * Distributed Controller:
 * Arduino Uno, Max31865+PT100, Max485+RK330, Max485 Slave,
 * DC-DC converter (24-9V, 9-5V), Relay for solenoid (pressure vent)
 * 
 */

// Libraries
#include <ModbusRtu.h>
#include <SoftwareSerial.h>


// Definitions
// Max485_1
#define DE_RE1 8
#define rxPin1 9
#define txPin1 10
// Max485_2
#define DE_RE2 2
//#define rxPin2 12
//#define txPin2 13


// Constants and Variables
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
unsigned long u32wait;
float t, h, p;
uint16_t au16data1[16] = {
  3, 1415, 9265, 4, 2, 7182, 28182, 8, 0, 0, 0, 0, 0, 0, 1, -1 };
int8_t state = 0;

// Constructors
SoftwareSerial mySerial1(rxPin1, txPin1);
Modbus master(0, mySerial1, DE_RE1);
modbus_t telegram;

//SoftwareSerial mySerial2(rxPin2, txPin2);
Modbus slave(1, Serial, DE_RE2);



void setup() {

  // Setup Serial Terminal
  Serial.begin(9600);

  // Setup Modbus
  mySerial1.begin(9600); // start software serial
  master.start(); // start the ModBus object
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  slave.start();

  // Initialise variables
  t = 0.0; h = 0.0; p = 0.0;



} // void setup


void loop() {


  /**
  * State Machine for reading data from RK330
  */
  switch (u8state) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  case 1: 
    /**
    * Refer to RK330 datasheet for specifications
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
        //Serial.print("Temp: "); Serial.print(t);
        //Serial.print(" deg, Humid: "); Serial.print(h);
        //Serial.print(" %RH, Press: "); Serial.print(p);
        //Serial.println(" kPa");
    }
    break;
  } // switch (u8state)

  slave.poll( au16data, 16 );
  //state = slave.poll( au16data1, 16 );
  //Serial.print("state = "); Serial.println(state);
  //Serial.print("In Cnt = "); Serial.println(slave.getInCnt());
  //Serial.print("Out Cnt = "); Serial.println(slave.getOutCnt());


} // void loop

