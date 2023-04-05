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
#include <Servo.h>

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
// Servo
#define servo_port A0
//#define servo_close 5
//#define servo_open 85
#define servo_close 180
#define servo_open 120
#define servo_open_duration 5000

// Constants and Variables
// data array for modbus network sharing
uint16_t au16data[16];

uint8_t u8state;
unsigned long u32wait;
float t, h, p;
uint16_t servo_val;
uint16_t servo_val_last;
unsigned long servo_millis;
uint16_t servo_opened;
int8_t state = 0;


// Constructors
SoftwareSerial mySerial1(rxPin1, txPin1);
Modbus master(0, mySerial1, DE_RE1);
modbus_t telegram;
Servo myservo;

//SoftwareSerial mySerial2(rxPin2, txPin2);
Modbus slave(1, Serial, DE_RE2);

// Max31865
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAXCS, MAXDI, MAXDO, MAXCLK);


void setup() {

  // Setup Serial Terminal
  Serial.begin(9600);
  slave.start();

  // Serup Servo
  myservo.attach(servo_port);
  servo_val = 0;
  servo_val_last = 0;
  servo_opened = 0;
  myservo.write(servo_close);
  delay(15);

  // Setup Modbus
  mySerial1.begin(9600); // start software serial
  master.start(); // start the ModBus object
  master.setTimeOut(5000); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
  u8state = 0;

  // Setup Max31865
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  // Initialise variables
  t = 0.0; h = 0.0; p = 0.0;

  servo_millis = millis();

  // Testing
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
      u32wait = millis() + 1000;
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
  * Get temperature from Max31865 (only temp, humid and pressure from RK330)
  */
  au16data[0] = thermo.temperature(RNOMINAL, RREF) * 100.0;
  // Testing
  //au16data[0] = 6804;
  //au16data[1] = 140;
  //au16data[2] = 881;

  /*
  *        ***  NOTE  ***
  *
  * CANNOT USE ANY SERIAL.PRINT BECAUSE THE SERIAL
  * PORT IS USED FOR MODBUS SLAVE. OTHERWISE WILL
  * SEND ZEROS TO THE MASTER (MEGA).
  */
// Serial.print("au16data[0]: "); Serial.print(au16data[0]);
// Serial.print(", state: "); Serial.print(state);
// Serial.print(", slave.getInCnt(): "); Serial.print(slave.getInCnt());
// Serial.print(", slave.getOutCnt(): "); Serial.print(slave.getOutCnt());
// Serial.print(", slave.getErrCnt(): "); Serial.print(slave.getErrCnt());
// Serial.print(", au16data[3]: "); Serial.println(au16data[3]);


  // Turn on the built-in LED depending on odd or even (testing)
// if ((au16data[3] % 2) == 1) {
//    digitalWrite(LED_BUILTIN, HIGH);
// } else {
//    digitalWrite(LED_BUILTIN, LOW);
// }

  // Move servo only if needed
  servo_val_last = servo_val;
  servo_val = au16data[3];

  // Auto close mode
  // open on rising edge, auto close after 5 seconds

  if ((servo_val == 1) && (servo_val_last == 0) && (servo_opened == 0)) {
     digitalWrite(LED_BUILTIN, HIGH); // test
     // open servo slowly
     for (int i = servo_close; i >= servo_open; i -= 4) {
        myservo.write(i);
        delay(10);
     }
     //servo_val_last = 1;
     servo_opened = 1;
     servo_millis = millis();
  }
  if (((millis() - servo_millis)>servo_open_duration) && (servo_opened == 1)) {
     digitalWrite(LED_BUILTIN, LOW); // test
     // close servo slowly
     for (int i = servo_open; i <= servo_close; i += 4) {
        myservo.write(i);
        delay(10);
     }
     //servo_val_last = 0;
     servo_opened = 0;
  }

  // Sent to and get data from master (mega)
  //state = slave.poll(au16data, 16);
  slave.poll(au16data, 16);

} // void loop
