/**
 * Project Hashtag
 * By Kang Liat Chuan, 12 Nov 2022
 *
 * Distributed Controller:
 * Arduino Mega (test Modbus comms)
 * 
 */

// Libraries
#include <ModbusRtu.h>


// Definitions
// Max485 (use Mega Serial3)
#define DE_RE 8
//#define rxPin 15
//#define txPin 14


// Constants and Variables
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
unsigned long u32wait;
float t, h, p;


// Constructors
Modbus master(0, Serial3, DE_RE);
modbus_t telegram;



void setup() {

  // Setup Serial Terminal
  Serial.begin(9600);

  // Setup Modbus
  Serial3.begin(9600);
  master.start(); // start the ModBus object
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;

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
        Serial.print("Temp: "); Serial.print(t);
        Serial.print(" deg, Humid: "); Serial.print(h);
        Serial.print(" %RH, Press: "); Serial.print(p);
        Serial.println(" kPa");
    }
    break;
  } // switch (u8state)



} // void loop

