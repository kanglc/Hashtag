/**
 * test_Uno.ino
 * for master (Mega) to test connectivity before plugging to slave
 * 
 */

#include <ModbusRtu.h>

// Max485
#define DE_RE2 2
//#define rxPin2 0
//#define txPin2 1

/* Constants and Variables */

// data array for modbus network sharing
uint16_t au16data[16];

float t, h, p;
uint16_t servo_val;
uint16_t servo_val_last;

// Max485
Modbus slave(4, Serial, DE_RE2);


void setup() {

  // Setup Modbus. Note: using Serial for Modbus slave cannot use Serial.print
  Serial.begin(9600);
  slave.start();

  // Initialise variables
  t = -20.0; h = 50.0; p = 0.0;
  servo_val = 0;
  servo_val_last = 0;

  // Testing
  pinMode(LED_BUILTIN, OUTPUT);

} // void setup


void loop() {


  // Simulate get temperature from Max31865
  // Multiply by 100 and transmit to master (mega) as 16bit integer
  // Mega will divide by 100 and get 2 decimal points float
  au16data[0] = t * 100.0;
  au16data[1] = h * 100.0;

  /*      ***  NOTE  ***
  *
  * CANNOT USE ANY SERIAL.PRINT BECAUSE THE SERIAL
  * PORT IS USED FOR MODBUS SLAVE. OTHERWISE WILL
  * SEND ZEROS TO THE MASTER (MEGA).
  */
// Serial.print("au16data[0]: "); Serial.print(au16data[0]);
// Serial.print(", au16data[3]: "); Serial.println(au16data[3]);


  servo_val = au16data[3];

  // Simulate move servo only if needed
  if ((servo_val == 0) && (servo_val_last == 1)) {
     // closing
     digitalWrite(LED_BUILTIN, LOW);
     servo_val_last = 0;
  } else if ((servo_val == 1) && (servo_val_last == 0)) {
     // opening
     digitalWrite(LED_BUILTIN, HIGH);
     servo_val_last = 1;
  }

  // Sent to and get data from master mega
  slave.poll(au16data, 16);

} // void loop
