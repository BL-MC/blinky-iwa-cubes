boolean chattyCathy = false;

#define BLINKY_DIAG         0
#define COMM_LED_PIN       LED_BUILTIN
#define RST_BUTTON_PIN     15
#include <BlinkyPicoW.h>

#define NO_MOTION_INTERVAL    120000
#define NO_MOTION_WARN         60000
#define NO_MOTION_ALARM_INT    10000
#define BUTTON_DEBOUNCE_DELAY 50
#define BUTTON_DOWN_TIME      3000
#define CHARGE_DEBOUNCE_DELAY 100
#define BATTERY_OK_LEVEL      450
#define CHARGING_LEVEL        682
#define AUTH_TIMEOUT          60000

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
 

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const int vsysPin = A2;
const int buttPin = 18;
const int buzzPin = 16;
const int rledPin = 17;
const int ain1Pin = 11;
const int ain2Pin = 10;
const int isdaPin = 20;
const int isclPin = 21;

struct CubeSetting
{
  uint16_t iauth;
  uint16_t iwarn;
  uint16_t publishInterval;
};
CubeSetting cubeSetting;

struct CubeReading
{
  uint16_t ibutt;
  uint16_t imove;
  uint16_t ichrg;
  uint16_t vsys;
};
CubeReading cubeReading;

int buttonState = 0;  
int lastButtonState = 0;  
int lastChargeState = 1;  
int buttonDownCount = 0;
int rLED = 0;

unsigned long lastButtonDebounceTime = 0;  
unsigned long buttonDownStartTime = 0;  
unsigned long lastChargeDebounceTime = 0;  
unsigned long lastMotionErrorTime = 0;  
unsigned long lastMotionWarnTime = 0;  
unsigned long lastPublishTime = 0;     
unsigned long lastAuthTime = 0; 
unsigned long lastMotionAlarmTime = 0; 
int16_t old_imsid = 0;

void setupBlinky()
{
  if (BLINKY_DIAG > 0) Serial.begin(9600);

  BlinkyPicoW.setSsid("RedRoofInn");
  BlinkyPicoW.setWifiPassword("SwedishChef");
  BlinkyPicoW.setMqttServer("hub.bl-mc.com");
  BlinkyPicoW.setMqttUsername("blinky-lite-box-01");
  BlinkyPicoW.setMqttPassword("areallybadpassword");
  BlinkyPicoW.setBox("blinky-lite-box-01");
  BlinkyPicoW.setTrayType("picoW");
  BlinkyPicoW.setTrayName("iwa-01");
  BlinkyPicoW.setCubeType("blinky-hub");
  BlinkyPicoW.setMqttKeepAlive(15);
  BlinkyPicoW.setMqttSocketTimeout(4);
  BlinkyPicoW.setMqttPort(1883);
  BlinkyPicoW.setMqttLedFlashMs(100);
  BlinkyPicoW.setHdwrWatchdogMs(8000);
  BlinkyPicoW.setRouterDelay(4000);

  BlinkyPicoW.begin(BLINKY_DIAG, COMM_LED_PIN, RST_BUTTON_PIN, true, sizeof(cubeSetting), sizeof(cubeReading));
}

void setupCube() 
{
  if (chattyCathy)
  {
    Serial.begin(9600);
    Serial.println("Starting sketch");
    delay(30000);
  }
  Wire.setSDA(isdaPin);
  Wire.setSCL(isclPin);

  pinMode(vsysPin, INPUT);
  pinMode(buttPin, INPUT);
  pinMode(buzzPin, OUTPUT);
  pinMode(rledPin, OUTPUT);
  delay(1000);
  cubeReading.vsys = (int16_t) analogRead(vsysPin);
  if (chattyCathy)
  {
    Serial.print("Battery voltage: ");
    Serial.println(cubeReading.vsys);
  }

  accel.begin();       
  accel.setRange(ADXL345_RANGE_2_G); 
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_THRESH_ACT, 4);
  accel.writeRegister(ADXL345_REG_ACT_INACT_CTL, 240);
  accel.writeRegister(ADXL345_REG_INT_MAP, 16);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 16);
    
  for (int ii = 0; ii < 3; ++ii) soundBeep(50);
  lastPublishTime = millis();
  lastMotionErrorTime = lastPublishTime;
  lastMotionWarnTime = lastPublishTime;
  lastAuthTime = lastPublishTime;
  lastMotionAlarmTime = lastPublishTime;
  buttonDownStartTime = lastPublishTime;
  buttonDownCount = 0;
  rLED = 1;
  digitalWrite(rledPin, rLED);
  cubeSetting.publishInterval = 5000;

  cubeReading.ichrg = 0; 
  cubeReading.ibutt = 0; 
  cubeReading.imove = 0; 
}
void loopCube() 
{
  unsigned long now =  millis();
  cubeReading.vsys = (uint16_t) analogRead(vsysPin);
  checkButton(now);
  checkMotion(now);
  checkCharging(now);
  if (cubeSetting.iwarn > 0) soundSOS();
  if ((now - lastPublishTime) > cubeSetting.publishInterval)
  {
    boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, false);
    lastPublishTime = now;
  }
  uint16_t prev_iauth = cubeSetting.iauth;
  if (BlinkyPicoW.retrieveCubeSetting((uint8_t*) &cubeSetting) )
  {
    boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
    lastPublishTime = now;
    if (cubeSetting.iauth > 0)
    {
        if ((cubeReading.ichrg > 0) && (prev_iauth == 0))
        {
          for (int ii = 0; ii < 3; ++ii) soundBeep(50);
          lastAuthTime = now;
        }
    }
    else
    {
      if (prev_iauth == 1)
      {
        cubeSetting.iauth = 1;
     }
    }
    if (cubeSetting.iwarn > 0)
    {
      if ((cubeReading.ichrg == 0) && (cubeSetting.iauth > 0))
      {
        soundSOS();
      }
      else
      {
        cubeSetting.iwarn = 0;
      }
    }
  }
}

void checkButton(unsigned long now)
{
  int reading = digitalRead(buttPin);
  if (reading != lastButtonState) 
  {
    lastButtonDebounceTime = now;
  }
  if ((now - lastButtonDebounceTime) > BUTTON_DEBOUNCE_DELAY )
  {
    if (reading != buttonState) 
    {
      buttonState = reading;
      if (buttonState > 0)
      {
        buttonDownStartTime = now;
        buttonDownCount = 0;
      }
      if (chattyCathy)
      {
        Serial.print("Button state: ");
        Serial.println(buttonState);
      }
    }
  }
  lastButtonState = reading;  
  if ((buttonState > 0) && ((now - buttonDownStartTime) > BUTTON_DOWN_TIME) )
  {
    if (chattyCathy)
    {
      Serial.print("Button down count: ");
      Serial.println(buttonDownCount);
    }
    if (buttonDownCount == 0)
    {
      if (cubeReading.ichrg ==  0)
      {
        if (cubeReading.ibutt == 0)
        {
          cubeReading.ibutt = 1;
          boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
          lastPublishTime = now;
          rLED = 1;
          digitalWrite(rledPin, rLED);
          digitalWrite(buzzPin, HIGH);
        }
        else
        {
          cubeReading.ibutt = 0;
          boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
          lastPublishTime = now;
          if (cubeReading.imove == 0)
          {
            if (cubeSetting.iauth > 0)
            {
              rLED = 0;
              digitalWrite(rledPin, rLED); 
              digitalWrite(buzzPin, LOW); 
            }
            
          }
        }
      }
      else
      {
        if (cubeReading.vsys >= BATTERY_OK_LEVEL) 
        {
          soundBeep(500);
          rLED = 1;
          digitalWrite(rledPin, rLED);
        }
      }
    }
    buttonDownCount = buttonDownCount + 1;
    buttonDownStartTime = now;
  }
}
void checkMotion(unsigned long now)
{
  
  uint8_t readData = accel.readRegister(ADXL345_REG_INT_SOURCE);
  if (chattyCathy)
  {
//    if ((readData & 16) > 0) Serial.println("Motion detected");
  }
  if ((readData & 16) > 0)
  {
    lastMotionErrorTime = now;
    lastMotionWarnTime = now;
    if ((cubeReading.ichrg == 0) && (cubeReading.ibutt == 0) && (cubeSetting.iauth > 0))
    {
      rLED = 0;
      digitalWrite(rledPin, rLED);
      digitalWrite(buzzPin, LOW); 
      if (cubeReading.imove == 1) 
      {
        cubeReading.imove = 0;
        boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
        lastPublishTime = now;
      }
    }
    cubeReading.imove = 0;
  }
  if ((cubeReading.ichrg == 0) && (cubeReading.ibutt == 0) && (cubeSetting.iauth > 0))
  {
    if ((now - lastMotionWarnTime) > NO_MOTION_WARN)
    {
      lastMotionWarnTime = now;
      if (cubeReading.imove == 0)
      {
        soundBeep(500);
        accel.readRegister(ADXL345_REG_INT_SOURCE);
      }
    }
    if ((now - lastMotionErrorTime) > NO_MOTION_INTERVAL)
    {
      lastMotionWarnTime = now;
      lastMotionErrorTime = now;
      if (cubeReading.imove == 0)
      {
        cubeReading.imove = 1;
        boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
        lastPublishTime = now;
        for (int ii = 0; ii < 3; ++ii) soundBeep(500);
        rLED = 1;
        digitalWrite(rledPin, rLED);
        accel.readRegister(ADXL345_REG_INT_SOURCE);
      }
    }
    if ((now - lastMotionAlarmTime) > NO_MOTION_ALARM_INT)
    {
      lastMotionAlarmTime = now;
      if (cubeReading.imove > 0)
      {
        soundBeep(100);
        rLED = 1;
        digitalWrite(rledPin, rLED);
        accel.readRegister(ADXL345_REG_INT_SOURCE);
      }
    }
  }
}

void checkCharging(unsigned long now)
{

  int chargeReading = 0;
  if (cubeReading.vsys > CHARGING_LEVEL) chargeReading = 1;
  if (chargeReading != lastChargeState) 
  {
    lastChargeDebounceTime = now;
  }
  if ((now - lastChargeDebounceTime) > CHARGE_DEBOUNCE_DELAY) 
  {
    if (chargeReading != cubeReading.ichrg) 
    {
      cubeReading.ichrg = chargeReading;
      cubeReading.ibutt = 0;
      cubeReading.imove = 0;
      if (cubeReading.ichrg > 0)
      {
        cubeSetting.iauth = 0;
        cubeSetting.iwarn = 0;
        digitalWrite(buzzPin, LOW);
        rLED = 1;
        digitalWrite(rledPin, rLED);
      }
      else
      {
        lastMotionWarnTime = now;
        lastMotionErrorTime = now;
        if (cubeSetting.iauth == 0)
        {
          rLED = 1;
          digitalWrite(rledPin, rLED);
          digitalWrite(buzzPin, HIGH);
        }
        else
        {
          rLED = 0;
          digitalWrite(rledPin, rLED);
          digitalWrite(buzzPin, LOW);
        }
      }
      boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
      lastPublishTime = now;
    }
  }
  lastChargeState = chargeReading; 
  if ((cubeSetting.iauth > 0) && (cubeReading.ichrg > 0) )
  {
    if ((now - lastAuthTime) > AUTH_TIMEOUT) 
    {
      lastAuthTime = now;
      cubeSetting.iauth = 0;
      rLED = 1;
      digitalWrite(rledPin, rLED);
      boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &cubeSetting, (uint8_t*) &cubeReading, true);
      lastPublishTime = now;
    }
  }
  if ((cubeSetting.iauth == 0) && (cubeReading.ichrg == 0) )
  {
      rLED = 1;
      digitalWrite(rledPin, HIGH);
      digitalWrite(buzzPin, HIGH);
  }
}

void soundSOS()
{
  int idelay[3] = {100,200,100};
  for (int is = 0; is < 3; ++is)
  {
    for (int ii = 0; ii < 3; ++ii)
    {
      soundBeep(idelay[is]);
    }
  }  
}
void soundBeep(int idelay)
{
  digitalWrite(buzzPin, LOW);
  rLED = 0;
  digitalWrite(rledPin, rLED);
  delay(idelay);
  digitalWrite(buzzPin, HIGH);
  rLED = 1;
  digitalWrite(rledPin, rLED);
  delay(idelay);
  delay(idelay);
  digitalWrite(buzzPin, LOW);
  rLED = 0;
  digitalWrite(rledPin, rLED);
  delay(idelay);
}
