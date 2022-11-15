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
#include <Adafruit_MAX31865.h>

// Definitions
// Max485_1
#define DE_RE1 8
#define rxPin1 9
#define txPin1 10
// Max485_2
#define DE_RE2 2
//#define rxPin2 0
//#define txPin2 1
// Max31865
#define MAXCS   7
#define MAXDI   6
#define MAXDO   5
#define MAXCLK  4
#define RREF      430.0
#define RNOMINAL  100.0


// Constants and Variables
// data array for modbus network sharing
uint16_t au16data[16];

uint8_t u8state;
unsigned long u32wait;
float t, h, p;

// Constructors
SoftwareSerial mySerial1(rxPin1, txPin1);
Modbus master(0, mySerial1, DE_RE1);
modbus_t telegram;

//SoftwareSerial mySerial2(rxPin2, txPin2);
Modbus slave(1, Serial, DE_RE2);

// Max31865
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAXCS, MAXDI, MAXDO, MAXCLK);


void setup() {

  // Setup Serial Terminal
  Serial.begin(9600);

  // Setup Modbus
  mySerial1.begin(9600); // start software serial
  master.start(); // start the ModBus object
  master.setTimeOut(5000); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;
  slave.start();

  // Setup Max31865
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  // Initialise variables
  t = 0.0; h = 0.0; p = 0.0;

  pinMode(LED_BUILTIN, OUTPUT);

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
    }
    break;
  } // switch (u8state)

  // Take care of negative temperatures
  // if (au16data[0] > 32767) {
  //    t = (au16data[0]-65536.0)/10.0;
  // } else {
  //    t = au16data[0]/10.0;
  // }
  // h = au16data[1]/10.0;
  // p = au16data[2]/10.0;

  /**
  * Get temperature from Max31865
  */
  au16data[0] = thermo.temperature(RNOMINAL, RREF);

  /*
  *        ***  NOTE  ***
  *
  * CANNOT USE ANY SERIAL.PRINT BECAUSE THE SERIAL
  * PORT IS USED FOR MODBUS SLAVE. OTHERWISE WILL
  * SEND ZEROS TO THE MASTER (MEGA).
  */
  // Serial.print("t: "); Serial.print(t);
  // Serial.print("au16data[0]: "); Serial.println(au16data[0]);
  // Serial.print(" deg, h: "); Serial.print(h);
  // Serial.print(" %RH, p: "); Serial.print(p);
  // Serial.print(" kPa");

  // Sent to and get data from master (mega)
  slave.poll(au16data, 16);
  if ((au16data[3] % 2) == 1) {
     digitalWrite(LED_BUILTIN, HIGH);
  } else {
     digitalWrite(LED_BUILTIN, LOW);
  }

} // void loop

