#include <Arduino.h>
#include <AD5940.h>

byte reset = 9;
byte chipSelect = 10; // Digital pin used to target AD5940

AD5940 AD5940_EVAL(chipSelect); // Instantiate AD5940 

void setup()
{
  Serial.begin(9600);

  pinMode(reset, OUTPUT);
  // pinMode(chipSelect, OUTPUT);

  digitalWrite(reset, HIGH);
  // digitalWrite(chipSelect, HIGH);

  delay(10);
  digitalWrite(reset, LOW);
  delay(10);
  digitalWrite(reset, HIGH);
  Serial.println("Reset complete");

  AD5940_EVAL.begin();
  AD5940_EVAL.init();
}

void loop() 
{
  uint16_t regValue = AD5940_EVAL.readOutput();
  Serial.println(regValue, HEX);
  delay(2000);
}