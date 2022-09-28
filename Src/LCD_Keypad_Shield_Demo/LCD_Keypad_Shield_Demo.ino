/*
  Example featuring the LCD Keypad Shield V1.1.
  Author: Tim Sinaeve
  (c) 2014
  
  Based on code by Mark Bramwell, July 2010
*/

#include <LiquidCrystal.h>

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

const byte BUTTON_RIGHT  = 0;
const byte BUTTON_UP     = 1;
const byte BUTTON_DOWN   = 2;
const byte BUTTON_LEFT   = 3;
const byte BUTTON_SELECT = 4;
const byte BUTTON_NONE   = 5;

// define some values used by the panel and buttons
int lcd_key    = 0;
int adc_key_in = 0;

// read the buttons
int readLCDButtons()
{
  adc_key_in = analogRead(0);      
  
  if (adc_key_in > 1000) 
    return BUTTON_NONE; 
  if (adc_key_in < 50)   
    return BUTTON_RIGHT;
  if (adc_key_in < 250)  
    return BUTTON_UP;
  if (adc_key_in < 350)  
    return BUTTON_DOWN;
  if (adc_key_in < 450)  
    return BUTTON_LEFT;
  if (adc_key_in < 900)  
    return BUTTON_SELECT;
  else
    return BUTTON_NONE;  
}

void setup()
{
  lcd.begin(16, 2);              
  lcd.setCursor(0, 0);
  lcd.print("Push the buttons"); 
}

void loop()
{
  lcd.setCursor(9, 1);         
  lcd.print(millis() / 1000);  
  lcd.setCursor(0, 1);           
  lcd_key = readLCDButtons();  
 
  switch (lcd_key)               
  {
    case BUTTON_RIGHT:
    {
       lcd.print("RIGHT ");
       break;
    }
    case BUTTON_LEFT:
    {
       lcd.print("LEFT   ");
       break;
    }
    case BUTTON_UP:
    {
       lcd.print("UP    ");
       break;
    }
    case BUTTON_DOWN:
    {
       lcd.print("DOWN  ");
       break;
    }
    case BUTTON_SELECT:
    {
       lcd.print("SELECT");
       break;
    }
    case BUTTON_NONE:
    {
       lcd.print("NONE  ");
       break;
    }
  }
}