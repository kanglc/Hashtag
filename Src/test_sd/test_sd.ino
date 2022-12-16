/*
  SD card read/write

  This example shows how to read and write data to and from an SD card file
  The circuit:
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created   Nov 2010
  by David A. Mellis
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/

#define CS 4

#include <SPI.h>
#include <SD.h>

File myFile;

unsigned long now = 0;
unsigned long lastMillis = 0;
String buffer;
String title;
String fname;
float t;


void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only

  Serial.print("Initializing SD card...");

  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  fname = __DATE__ ".csv";
  // File name can only be 8 characters long
  fname.replace(" ", "");
  fname.replace("Jan", "01");
  fname.replace("Feb", "02");
  fname.replace("Mar", "03");
  fname.replace("Apr", "04");
  fname.replace("May", "05");
  fname.replace("Jun", "06");
  fname.replace("Jul", "07");
  fname.replace("Aug", "08");
  fname.replace("Sep", "09");
  fname.replace("Oct", "10");
  fname.replace("Nov", "11");
  fname.replace("Dec", "12");
  fname.replace("2022", "22");
  fname.replace("2023", "23");
  fname.replace("2024", "24");
  fname.replace("2025", "25");
  fname.replace("2026", "26");
  fname.replace("2027", "27");
  fname.replace("2028", "28");
  fname.replace("2029", "29");
  fname.replace("2030", "30");
  Serial.println("fname: " + fname);
  myFile = SD.open(fname, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.print("Data logged on:,");
    myFile.println(__DATE__);
    myFile.println("Date,Time,Temp (C)");
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("Error opening file!");
    // while (1);
  }

  t = 0.0;

} // void setup()


void loop() {

  t = random(0.00, 100.00);

  // write to file once every 1000ms
  now = millis();
  if ((now - lastMillis) >= 1000) {
    buffer = __DATE__ "," __TIME__;
    buffer += ",";
    buffer += t;
    buffer += "\r\n";
    Serial.print("Writing to File: ");
    Serial.println(buffer.c_str());

    myFile = SD.open(fname, FILE_WRITE);
    if (myFile) {
      myFile.println(buffer.c_str());
      myFile.close();
      Serial.println("done.");
    } else {
      Serial.println("Error opening file!");
    }

    // Update time
    lastMillis = now;

  } // if ((now - lastMillis) >= 5000)

} // void loop(
