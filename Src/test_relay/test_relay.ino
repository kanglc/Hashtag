/**
 * For testing the 2 relay module
 * By Kang Liat Chuan, 6 Sep 2022
 * 
 * Hardware: Arduino Uno, 2 relay module
 *           PC running Arduino IDE - USB - Arduino Uno - Max485 - RK330-02
 *           Uno pin 3 - Relay module IN1
 *           Uno pin 4 - Relay module IN2
 *           Uno pin 5V - Relay module VCC
 *           Uno pin GND - Relay module GND
 *
 *           A separate 12v-5v module is used for the JD-VCC pin of the relay
 *
 * Software: Arduino IDE
 */
const int IN1 = 3;
const int IN2 = 4;

void setup() {
  Serial.begin(9600); // start terminal via usb
  Serial.println("Test 2ch relay module - 0 to activate, others to deactivate.");
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 1);
}

void loop() {

  if (Serial.available() > 0) {

    //int val = Serial.read();
    int val = Serial.parseInt();

    if (Serial.read() == '\n') {
      
      if (val == 0) {
        Serial.println("Activating IN2");
        digitalWrite(IN2, 0);
      } else {
        Serial.println("Deactivating IN2");
        digitalWrite(IN2, 1);
      }
      
    }
  }
}
