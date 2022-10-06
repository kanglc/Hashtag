#include <Wire.h>
#include <ds3231.h>

struct ts t;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  DS3231_init(DS3231_CONTROL_INTCN);

  t.hour=18;
  t.min=27;
  t.sec=0;
  t.mday=06;
  t.mon=10;
  t.year=2022;
 
  DS3231_set(t);

}

void loop() {
  // put your main code here, to run repeatedly:
  DS3231_get(&t);
  Serial.print("Date : ");
  Serial.print(t.mday);
  Serial.print("/");
  Serial.print(t.mon);
  Serial.print("/");
  Serial.print(t.year);
  Serial.print("\t Time : ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(".");
  Serial.println(t.sec);

  delay(500);
}
