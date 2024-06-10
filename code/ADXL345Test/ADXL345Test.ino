#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(6, OUTPUT);
  pinMode(10, INPUT);
  accel.begin();        // join i2c bus (address optional for master)
  accel.setRange(ADXL345_RANGE_2_G); 
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_THRESH_ACT, 3);
  accel.writeRegister(ADXL345_REG_ACT_INACT_CTL, 240);
  accel.writeRegister(ADXL345_REG_INT_MAP, 16);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 16);
}

void loop() 
{

  int pin10Value = digitalRead(10);
  byte isource = accel.readRegister(ADXL345_REG_INT_SOURCE);
  digitalWrite(6, digitalRead(10));

                    
}
