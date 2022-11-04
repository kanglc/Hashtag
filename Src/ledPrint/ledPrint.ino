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
char dpt;

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
  Serial.println("Setup done");

}

void loop() 
{
  /**
   * Display "HALO"
   * At present, it only supports showing the numbers 0 to 9, capital letters A, B, C, D, E, F, H, L, O, P, U and dash-,
   * and you can also bring decimal points, such as "0." "9." "A." "-."
   */
  
  //Serial.println("HALO");
  LED.print("F","U.","C","B");
  delay(1000);

  /*
  Serial.println("H.A.L.O.");
  LED.print("H.","A.","L.","O."); 
  delay(1000);
  */

  while (Serial.available() == 0) {};
  val = Serial.parseFloat();
  Serial.print("val = "); Serial.print(val);
  dstr = "";
  dstr = val;
  Serial.print(" dstr = "); Serial.print(dstr);
  if (val>=0) {
    // positive
    if (abs(val)<10) {
     //switch (dstr[0]) {
     //case '0': dpt = "0."; break;
     //case '1': dpt = "1."; break;
     //case '2': dpt = "2."; break;
     //case '3': dpt = "3."; break;
     //case '4': dpt = "4."; break;
     //case '5': dpt = "5."; break;
     //case '6': dpt = "6."; break;
     //case '7': dpt = "7."; break;
     //case '8': dpt = "8."; break;
     //case '9': dpt = "9."; break;
     //}
      //LED.print(" ", " ", &dpt, &dstr[2]);
      dpt = dstr[0];
      dpt += ".";
      Serial.print(" dpt = "); Serial.println(dpt);
      LED.print(" ", " ", dpt, &dstr[2]);
    } else {
      switch (dstr[1]) {
      case '0': dpt = "0."; break;
      case '1': dpt = "1."; break;
      case '2': dpt = "2."; break;
      case '3': dpt = "3."; break;
      case '4': dpt = "4."; break;
      case '5': dpt = "5."; break;
      case '6': dpt = "6."; break;
      case '7': dpt = "7."; break;
      case '8': dpt = "8."; break;
      case '9': dpt = "9."; break;
      }
      //LED.print(" ", &dstr[0], &dpt, &dstr[3]);
      LED.print(" ", &dstr[0], "O.", &dstr[3]);
    }
  } else {
    // negative
    if (abs(val)<10) {
      switch (dstr[1]) {
      case '0': dpt = "0."; break;
      case '1': dpt = "1."; break;
      case '2': dpt = "2."; break;
      case '3': dpt = "3."; break;
      case '4': dpt = "4."; break;
      case '5': dpt = "5."; break;
      case '6': dpt = "6."; break;
      case '7': dpt = "7."; break;
      case '8': dpt = "8."; break;
      case '9': dpt = "9."; break;
      }
      //LED.print("-", " ", &dpt, &dstr[3]);
      LED.print("-", " ", "O.", &dstr[3]);
    } else {
      switch (dstr[2]) {
      case '0': dpt = "0."; break;
      case '1': dpt = "1."; break;
      case '2': dpt = "2."; break;
      case '3': dpt = "3."; break;
      case '4': dpt = "4."; break;
      case '5': dpt = "5."; break;
      case '6': dpt = "6."; break;
      case '7': dpt = "7."; break;
      case '8': dpt = "8."; break;
      case '9': dpt = "9."; break;
      }
      //LED.print("-", &dstr[1], &dpt, &dstr[4]);
      LED.print("-", &dstr[1], "O.", &dstr[4]);
    }
  }
  Serial.println();

  delay(1000);

}
