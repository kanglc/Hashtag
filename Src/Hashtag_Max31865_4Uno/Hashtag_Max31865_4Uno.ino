/**
 * Project Hashtag
 * By Kang Liat Chuan, 8 Dec 2022
 *
 * Distributed Controller:
 * Arduino Uno, Max31865+PT100, Max485 Slave,
 * DC-DC converter (24-9V, 9-5V), Servo for pressure vent
 * 
 */

/* Libraries */

#include <ModbusRtu.h>
#include "DHT.h"
#include <DHT_U.h>
#include <Adafruit_MAX31865.h>
#include <Servo.h>

/* Definitions */

// DHT22
#define DHTPIN 8     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Max485
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
#define servo_close 180
#define servo_open 130
#define servo_pin 12

/* Constants and Variables */

// data array for modbus network sharing
uint16_t au16data[16];

uint8_t u8state;
unsigned long u32wait;
float t, h, p;
uint16_t servo_val;
uint16_t servo_val_last;
int8_t state = 0;
unsigned long current_millis;


/* Constructors */

// DHT22
DHT dht(DHTPIN, DHTTYPE);

// Servo
Servo myservo;

// Max485
Modbus slave(4, Serial, DE_RE2);

// Max31865
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAXCS, MAXDI, MAXDO, MAXCLK);


void setup() {

  // Setup Modbus. Note: using Serial for Modbus slave cannot use Serial.print
  Serial.begin(19200);
  slave.start();

  // Serup Servo
  pinMode(servo_pin, INPUT);
  myservo.attach(servo_port);
  servo_val = 0;
  servo_val_last = 0;
  myservo.write(servo_close);
  delay(10);

  // Setup Max31865
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  // Setup DHT22
  dht.begin();

  // Initialise variables
  t = 0.0; h = 0.0; p = 0.0;
  current_millis = millis();

  // Testing
  //pinMode(LED_BUILTIN, OUTPUT);


} // void setup


void loop() {

  // Get temperature and humidity from DHT22
  if ((millis() - current_millis) > 2000) {
     if (isnan(t) || isnan(h)) {
        // Serial.println(F("Failed to read from DHT sensor!"));
        return;
     } else {
        t = dht.readTemperature();
        h = dht.readHumidity();
        current_millis = millis();
     }
  }


  // Get temperature from Max31865
  // Multiply by 100 and transmit to master (mega) as 16bit integer
  // Mega will divide by 100 and get 2 decimal points float
  au16data[0] = thermo.temperature(RNOMINAL, RREF) * 100.0;
  au16data[1] = h * 100.0;

  /*      ***  NOTE  ***
  *
  * CANNOT USE ANY SERIAL.PRINT BECAUSE THE SERIAL
  * PORT IS USED FOR MODBUS SLAVE. OTHERWISE WILL
  * SEND ZEROS TO THE MASTER (MEGA).
  */
// Serial.print("au16data[0]: "); Serial.print(au16data[0]);
// Serial.print(" t: "); Serial.print(t);
// Serial.print(" h: "); Serial.println(h);
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
  //servo_val = au16data[3];
  servo_val = digitalRead(servo_pin);
  if ((servo_val == 0) && (servo_val_last == 1)) {
     // closing
     //digitalWrite(LED_BUILTIN, LOW); // testing
     for (int i = servo_open; i <= servo_close; i += 4) {
        myservo.write(i);
        delay(10);
     }
     servo_val_last = 0;
  } else if ((servo_val == 1) && (servo_val_last == 0)) {
     // opening
     //digitalWrite(LED_BUILTIN, HIGH); // testing
     for (int i = servo_close; i >= servo_open; i -= 4) {
        myservo.write(i);
        delay(10);
     }
     servo_val_last = 1;
  }

  // Sent to and get data from master mega
  //state = slave.poll(au16data, 16);
  slave.poll(au16data, 16);

} // void loop
