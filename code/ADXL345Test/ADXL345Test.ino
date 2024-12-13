#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


void setup() 
{
  Serial.begin(9600);
  delay(3000);

  Wire.setSDA(20);
  Wire.setSCL(21);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(10, INPUT_PULLDOWN);
  pinMode(11, INPUT_PULLDOWN);
  accel.begin();        // join i2c bus (address optional for master)
  delay(1000);
  accel.setRange(ADXL345_RANGE_16_G); 
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_THRESH_ACT, 3);
  accel.writeRegister(ADXL345_REG_ACT_INACT_CTL, 240);
  accel.writeRegister(ADXL345_REG_INT_MAP, 16);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 16);
}

void loop() 
{
  delay(1000);  

  Serial.println(accel.readRegister(ADXL345_REG_DATAX0));
  Serial.println(accel.readRegister(ADXL345_REG_DATAY0));
  Serial.println(accel.readRegister(ADXL345_REG_DATAZ0));
  Serial.println(accel.readRegister(ADXL345_REG_DATAX1));
  Serial.println(accel.readRegister(ADXL345_REG_DATAY1));
  Serial.println(accel.readRegister(ADXL345_REG_DATAZ1));
  Serial.println("");

                    
}
