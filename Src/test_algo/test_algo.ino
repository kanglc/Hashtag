// Libraries

// Definitions
// Relay
#define IN1 3
#define IN2 4
#define Manual_Mode_pin A2
#define Manual_ON_pin A3
#define Relay_ON 0
#define Relay_OFF 1
// Algo
#define t20 20
#define t10 10
#define t0 0
#define t_10 -10
#define t_20 -20
#define t_25 -25
// On times
#define t20_ON 20
#define t10_ON 15
#define t0_ON 10
#define t_10_ON 10
#define t_20_ON 10
#define t_25_ON 5
// Off times
#define t20_OFF 5
#define t10_OFF 10
#define t0_OFF 10
#define t_10_OFF 15
#define t_20_OFF 20
#define t_25_OFF 20

// Constants
unsigned long current_millis;
unsigned long seconds_millis;

// Variables
float t, h, p;
uint8_t Mode = 0;
uint8_t Manual_ON_OFF = 0;
uint8_t cooling = 1;
uint8_t Auto_ON_OFF = 0;
int16_t t_ON = 0;
int16_t t_OFF = 0;
uint8_t sec = 0;


void setup() {

  // Setup Relay
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, Relay_OFF);
  digitalWrite(IN2, Relay_OFF);
  pinMode(Manual_Mode_pin, INPUT);
  pinMode(Manual_ON_pin, INPUT);

  // Setup Serial Terminal
  Serial.begin(9600);

  // Initialize temperature, humidaity and pressure
  t = 0.0; h = 0.0; p = 0.0;
  Serial.print("Temperature: "); Serial.println(t);
  cooling = 1;
  Auto_ON_OFF = 0;
  t_ON = 0;
  t_OFF = 0;
  sec = 0;

  current_millis = millis();
  seconds_millis = millis();

} // void setup()

void loop() {

  /**
  * Simulate getting temperature using Serial Terminal
  * note: set the serial terminal to 'no line ending'
  */ 
  if (Serial.available() > 0) {
     t = Serial.parseFloat();
     Serial.print("Temperature: "); Serial.println(t);
  }


  /**
  * Read State of Mode and Manual_ON_OFF switches
  */
  //Mode = digitalRead(Manual_Mode_pin);
  Mode = 0;
  Manual_ON_OFF = digitalRead(Manual_ON_pin);

  /**
  * Output to Relay
  */ 
  if (Mode == 1) { // Manual Mode

     if (Manual_ON_OFF == 1) {
        digitalWrite(IN1, Relay_ON);
     } else {
        digitalWrite(IN1, Relay_OFF);
     }

  } else { // Auto Mode

//   if (cooling == 1) {
//      digitalWrite(IN1, Relay_ON);
//      if (t <= 10.0) {
//         digitalWrite(IN1, Relay_OFF);
//               cooling = 0;
//      }
//   } else {
//      digitalWrite(IN1, Relay_OFF);
//      if (t >= 15.0) {
//         digitalWrite(IN1, Relay_ON);
//               cooling = 1;
//      }
//   }

     if (t >= t20) {
	t_ON = t20_ON; t_OFF = t20_OFF;
     } else if (t < t20 && t >= t10) {
        t_ON = t10_ON; t_OFF = t10_OFF;
     } else if (t < t10 && t >= t0) {
        t_ON = t0_ON; t_OFF = t0_OFF;
     } else if (t < t0 && t >= t_10) {
        t_ON = t_10_ON; t_OFF = t_10_OFF;
     } else if (t < t_10 && t >= t_20) {
        t_ON = t_20_ON; t_OFF = t_20_OFF;
     } else if (t < t_20 && t >= t_25) {
        t_ON = t_25_ON; t_OFF = t_25_OFF;
     } else {
        t_ON = 5; t_OFF = 20;
     }
     t_ON = t_ON * 1000;
     t_OFF = t_OFF * 1000;

     if ((millis() - seconds_millis) >= 1000) {
        sec += 1;
        Serial.print(sec); Serial.println(" sec");
	seconds_millis = millis();
     }

     if (Auto_ON_OFF == 0) { // OFF
        if ((millis() - current_millis) >= t_OFF) {
	   Serial.println(".......... ON");
	   Auto_ON_OFF = 1;
	   current_millis = millis();
	   sec = 0;
        }
     } else { // ON
        if ((millis() - current_millis) >= t_ON) {
	   Serial.println(".......... OFF");
	   Auto_ON_OFF = 0;
	   current_millis = millis();
	   sec = 0;
        }
     }



  } // if (Mode == 1)

} // void loop()
