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
// Max485
#define DE_RE 17
#define slave_addr 3

// Constants and Variables
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
uint8_t u8query;
unsigned long u32wait;
float t, h, p;
unsigned long wait1min;


// Constructors
Modbus master(0, Serial1, DE_RE);
modbus_t telegram[2];


void setup() {

  // telegram 0: read registers
  telegram[0].u8id = slave_addr; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 0; // start address in slave
  telegram[0].u16CoilsNo = 3; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // telegram 1: write a single register
  telegram[1].u8id = slave_addr; // slave address
  telegram[1].u8fct = 6; // function code (this one is write a single register)
  telegram[1].u16RegAdd = 3; // start address in slave
  telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+3; // pointer to a memory array in the Arduino

  // Setup Serial Terminal
  Serial.begin(9600);

  // Setup Modbus
  Serial1.begin(9600);
  master.start(); // start the ModBus object
  master.setTimeOut(5000); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  u8query = 0;

  // Initialise variables
  t = 0.0; h = 0.0; p = 0.0;
  au16data[3] = 0;
  wait1min = millis();


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
    master.query(telegram[u8query]); // send query (only once)
    u8state++;
    u8query++;
    if (u8query > 1) u8query = 0;
    break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 1000;
    }
    break;
  } // switch (u8state)

  // Take care of negative temperatures
  if (au16data[0] > 32767) {
     t = (au16data[0]-65536.0)/100.0;
  } else {
     t = au16data[0]/100.0;
  }
  // t = au16data[0]/100.0; // temp from Max31865
  h = au16data[1]/100.0; // humidity from DHT22
  // h = au16data[1]/10.0; // humidity from RK330
  // p = au16data[2]/10.0; // pressure from RK330
  
  // Transmit an increasing number to slave (testing)
  //if ((millis() - wait1min) > 3000) {
  //   au16data[3]++;
  //   wait1min = millis();
  //}

  // Transmit 1/0 to turn on/off servo
  if (Serial.available()) {
     au16data[3] = Serial.parseInt();
  }

  Serial.print("t: "); Serial.print(t);
  Serial.print(" deg, h: "); Serial.print(h);
  Serial.print(" %RH, p: "); Serial.print(p);
  Serial.print(" kPa, au16data[3]: "); Serial.println(au16data[3]);

} // void loop

