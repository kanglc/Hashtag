/*!
 * @file ledPrint.ino
 * @brief Display experiment of the digital tube
 * @n Experiment phenomenon: The digital tube displays "HALO"，and in one second, displays "H.A.L.O.", then show value of the variable val after one second
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Actor](wenzheng.wang@dfrobot.com),[TangJie](jie.tang@dfrobot.com)
 * @version  V1.0.1
 * @data  2022-03-21
 * @url https://github.com/DFRobot/DFRobot_LedDisplayModule
 */
#include "DFRobot_LedDisplayModule.h"
/**
 * DFRobot_LedDisplayModule Constructor
 * Parameter &wire  Wire
 * In default, the IIC address of 8 bits digital tube is 0xE0
 * The IIC address of 8 bits digital tube can be changed by combining A1 and A0
 * 1  1  1  0  | 0  A1 A0 0
 * 1  1  1  0  | 0  0  0  0    0xE0
 * 1  1  1  0  | 0  0  1  0    0xE2
 * 1  1  1  0  | 0  1  0  0    0xE4
 * 0  0  1  0  | 0  1  1  0    0xE6
 */ 
//DFRobot_LedDisplayModule LED(&Wire, 0xE0);

/**
 * DFRobot_LedDisplayModule Constructor
 * In default, the IIC address of 4 bits digital tube is 0x48 
 */
DFRobot_LedDisplayModule LED(&Wire, 0x48);

float val;
String dstr;

void setup() 
{
  Serial.begin(9600);
  /*
   * Wait for the chip to be initialized completely, and then exit.
   * Select several bits for initialization, e8Bit for 8 bits and e4Bit for 4 bits.
   */
  while(LED.begin(LED.e4Bit) != 0)
  {
    Serial.println("Failed to initialize the chip , please confirm the chip connection!");
    delay(1000);
  }
  
  /**
   * Set the display area to 1, 2, 3, 4
   * It can show 4 bits, the region of each parameter is 1~4 
   * Please resend the display value if the display area is changed
   */
  LED.setDisplayArea(1,2,3,4);
  Serial.println("Setup done. Key in a floating point number from -30.00 to 30.00");

}

void loop() 
{

  // Wait for serial terminal input
  while (Serial.available() == 0) {};
  val = Serial.parseFloat();
  Serial.print("val = "); Serial.print(val);
  dstr = val;
  Serial.print(" dstr = "); Serial.print(dstr); Serial.println();
  if (val >= 0) {

    // positive
    if (abs(val)<10) {
      // single positive
      LED.print(" ", &dstr[0], &dstr[2], &dstr[3]);
    } else {
      // double positive
      LED.print(" ", &dstr[0], &dstr[1], &dstr[3]);
    }

  } else {

    // negative
    if (abs(val)<10) {
      // single negative
      LED.print("-", &dstr[1], &dstr[3], &dstr[4]);
    } else {
      // double negative
      LED.print("-", &dstr[1], &dstr[2], &dstr[4]);
    }

  } // if (val >= 0)
  delay(1000);

}
