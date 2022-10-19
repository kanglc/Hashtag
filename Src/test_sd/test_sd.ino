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

  SD.remove("test.txt");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    while (1);
  }

  t = 0.0;

} // void setup()


void loop() {

  t = random(0, 20);

  // check if it's been over 5000 ms since the last line added
  now = millis();
  if ((now - lastMillis) >= 5000) {
    buffer = "Time now: ";
    buffer += now;
    buffer += " Random data = ";
    buffer += t;
    buffer += "\r\n";
    Serial.println("Data: ");
    Serial.println(buffer.c_str());

    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) {
      myFile.println(buffer.c_str());
      myFile.close();
      Serial.println("Written to file");
    } else {
      Serial.println("Error opening file!");
    }

    // Update time
    lastMillis = now;

  } // if ((now - lastMillis) >= 5000)

} // void loop(

