boolean chattyCathy = false;
#define DEVICE_ADDRESS        10
#define NO_MOTION_INTERVAL    120000
#define NO_MOTION_WARN         60000
#define NO_MOTION_ALARM_INT    10000
#define RF_FREQ               433.200
#define PUBLISH_INTERVAL      30000
#define PUBLISH_OFFSET        10000
#define BUTTON_DEBOUNCE_DELAY 50
#define CHARGE_DEBOUNCE_DELAY 100
#define BATTERY_OK_LEVEL      642
#define AUTH_TIMEOUT          60000

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include "RH_RF95.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int sigPower = 20;
int modemConfigIndex = 0;
float rfFreq = RF_FREQ;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const int commLEDPin = 12; //LED_BUILTIN;
const int vbatPin = A7;
const int vusbPin = A1;
const int buttPin = 5;
const int mesgPin = 6;

union RadioPacketBadge
{
  struct
  {
    int16_t itype;
    int16_t iaddr;
    int16_t istatus;
    int16_t ivbat;
    int16_t ivusb;
    int16_t iwatchdog;
  };
  uint8_t buffer[12];
};
RadioPacketBadge radioPacketBadge;
uint8_t sizeOfRadioPacketBadge = 12;

union RadioPacketStation
{
  struct
  {
    int16_t itype;
    int16_t iaddr;
    int16_t istatus;
    int16_t imsid;
    int16_t extra[2];
  };
  uint8_t buffer[12];
};
RadioPacketStation radioPacketStation;
uint8_t sizeOfRadioPacketStation = 12;

struct BadgeStatus
{
  int ibutt = 0;
  int imove = 0;
  int ichrg = 1;
  int iauth = 0;
};
BadgeStatus badgeStatus;

struct StationStatus
{
  int iwarn = 0;
  int iauth = 0;
};
StationStatus stationStatus;

int buttonState = 0;  
int lastButtonState = 0;  
int lastChargeState = 1;  
int iwarn = 0;

unsigned long lastButtonDebounceTime = 0;  
unsigned long lastChargeDebounceTime = 0;  
unsigned long lastMotionErrorTime = 0;  
unsigned long lastMotionWarnTime = 0;  
unsigned long publishInterval = PUBLISH_INTERVAL;     
unsigned long lastPublishTime = 0;     
unsigned long lastAuthTime = 0; 
unsigned long lastMotionAlarmTime = 0; 
int16_t old_imsid = 0;

void setup() 
{
  if (chattyCathy)
  {
    Serial.begin(9600);
    delay(3000);
    Serial.println("Starting sketch");
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(A0, INPUT);
  pinMode(vbatPin, INPUT);
  pinMode(vusbPin, INPUT);
  pinMode(buttPin, INPUT);
  pinMode(mesgPin, OUTPUT);
  pinMode(commLEDPin, OUTPUT);
  digitalWrite(commLEDPin, LOW);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(rfFreq);
  rf95.setModemConfig(modeConfig[modemConfigIndex]); 
  rf95.setTxPower(sigPower, false);
  radioPacketBadge.iaddr = DEVICE_ADDRESS;

  accel.begin();       
  accel.setRange(ADXL345_RANGE_2_G); 
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_THRESH_ACT, 3);
  accel.writeRegister(ADXL345_REG_ACT_INACT_CTL, 240);
  accel.writeRegister(ADXL345_REG_INT_MAP, 16);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 16);
    
  for (int ii = 0; ii < 3; ++ii) soundBeep(50);
  radioPacketBadge.iwatchdog = 0;
  randomSeed(analogRead(A0));
  publishInterval = publishInterval + (unsigned long) random(PUBLISH_OFFSET);
  if (chattyCathy) 
  {
    Serial.print("publishInterval: ");
    Serial.println(publishInterval);
  }
  lastPublishTime = millis();
  lastMotionErrorTime = lastPublishTime;
  lastMotionWarnTime = lastPublishTime;
  lastAuthTime = lastPublishTime;
  lastMotionAlarmTime = lastPublishTime;
  digitalWrite(commLEDPin, HIGH);
   
}
void loop() 
{
  unsigned long now =  millis();
  radioPacketBadge.ivbat = (int16_t) analogRead(vbatPin);
  radioPacketBadge.ivusb = (int16_t) analogRead(vusbPin);
  checkButton(now);
  checkMotion(now);
  checkCharging(now);
  if (iwarn > 0) soundSOS();
  if ((now - lastPublishTime) > publishInterval) transmitMsg(now);
  checkForMessage(now);
}

void checkButton(unsigned long now)
{
  int reading = digitalRead(buttPin);
  if (reading != lastButtonState) 
  {
    lastButtonDebounceTime = now;
  }
  if ((now - lastButtonDebounceTime) > BUTTON_DEBOUNCE_DELAY) 
  {
    if (reading != buttonState) 
    {
      buttonState = reading;
      if (buttonState == 1)
      {
        if (badgeStatus.ichrg == 0)
        {
          if (badgeStatus.ibutt == 0)
          {
            badgeStatus.ibutt = 1;
            transmitMsg(now);
            digitalWrite(commLEDPin, HIGH);
            digitalWrite(mesgPin, HIGH);
          }
          else
          {
            badgeStatus.ibutt = 0;
            transmitMsg(now);
            if (badgeStatus.imove == 0)
            {
              if (badgeStatus.iauth > 0)
              {
                digitalWrite(commLEDPin, LOW); 
                digitalWrite(mesgPin, LOW); 
              }
              
            }
          }
        }
        else
        {
          if (radioPacketBadge.ivbat >= BATTERY_OK_LEVEL) soundBeep(500);
        }
      }
    }
  }
  lastButtonState = reading;  
}
void checkMotion(unsigned long now)
{
  
  uint8_t readData = accel.readRegister(ADXL345_REG_INT_SOURCE);
  if ((readData & 16) > 0)
  {
    lastMotionErrorTime = now;
    lastMotionWarnTime = now;
    if ((badgeStatus.ichrg == 0) && (badgeStatus.ibutt == 0) && (badgeStatus.iauth > 0))
    {
      digitalWrite(commLEDPin, LOW);
      digitalWrite(mesgPin, LOW); 
      if (badgeStatus.imove == 1) 
      {
        badgeStatus.imove = 0;
        transmitMsg(now);
      }
    }
    badgeStatus.imove = 0;
  }
  if ((badgeStatus.ichrg == 0) && (badgeStatus.ibutt == 0) && (badgeStatus.iauth > 0))
  {
    if ((now - lastMotionWarnTime) > NO_MOTION_WARN)
    {
      lastMotionWarnTime = now;
      if (badgeStatus.imove == 0)
      {
        soundBeep(500);
        accel.readRegister(ADXL345_REG_INT_SOURCE);
      }
    }
    if ((now - lastMotionErrorTime) > NO_MOTION_INTERVAL)
    {
      lastMotionWarnTime = now;
      lastMotionErrorTime = now;
      if (badgeStatus.imove == 0)
      {
        badgeStatus.imove = 1;
        transmitMsg(now);
        for (int ii = 0; ii < 3; ++ii) soundBeep(500);
        digitalWrite(commLEDPin, HIGH);
        accel.readRegister(ADXL345_REG_INT_SOURCE);
      }
    }
    if ((now - lastMotionAlarmTime) > NO_MOTION_ALARM_INT)
    {
      lastMotionAlarmTime = now;
      if (badgeStatus.imove > 0)
      {
        soundBeep(100);
        digitalWrite(commLEDPin, HIGH);
        accel.readRegister(ADXL345_REG_INT_SOURCE);
      }
    }
  }
}

void checkCharging(unsigned long now)
{

  int chargeReading = 0;
  if (radioPacketBadge.ivusb > radioPacketBadge.ivbat) chargeReading = 1;
  if (chargeReading != lastChargeState) 
  {
    lastChargeDebounceTime = now;
  }
  if ((now - lastChargeDebounceTime) > CHARGE_DEBOUNCE_DELAY) 
  {
    if (chargeReading != badgeStatus.ichrg) 
    {
      badgeStatus.ichrg = chargeReading;
      badgeStatus.ibutt = 0;
      badgeStatus.imove = 0;
      if (badgeStatus.ichrg > 0)
      {
        badgeStatus.iauth = 0;
        iwarn = 0;
        digitalWrite(mesgPin, LOW);
        digitalWrite(commLEDPin, HIGH);
      }
      else
      {
        lastMotionWarnTime = now;
        lastMotionErrorTime = now;
        if (badgeStatus.iauth == 0)
        {
          digitalWrite(commLEDPin, HIGH);
          digitalWrite(mesgPin, HIGH);
        }
        else
        {
          digitalWrite(commLEDPin, LOW);
          digitalWrite(mesgPin, LOW);
        }
      }
      transmitMsg(now);
    }
  }
  lastChargeState = chargeReading; 
  if ((badgeStatus.iauth > 0) && (badgeStatus.ichrg > 0) )
  {
    if ((now - lastAuthTime) > AUTH_TIMEOUT) 
    {
      lastAuthTime = now;
      badgeStatus.iauth = 0;
      digitalWrite(commLEDPin, HIGH);
      transmitMsg(now);
    }
  }
}

void transmitMsg(unsigned long now)
{
  radioPacketBadge.itype = 0; // tell that it is a badge
  radioPacketBadge.iwatchdog = radioPacketBadge.iwatchdog + 1;
  if (radioPacketBadge.iwatchdog > 32765) radioPacketBadge.iwatchdog = 0;

  radioPacketBadge.istatus = 0;

  if (badgeStatus.ibutt > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 1;
  if (badgeStatus.imove > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 2;
  if (badgeStatus.ichrg > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 4;
  if (badgeStatus.iauth > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 8;

  if (chattyCathy)
  {
      Serial.print("wdog: ");
      Serial.println(radioPacketBadge.iwatchdog);
  }
  
  rf95.send(radioPacketBadge.buffer, sizeOfRadioPacketBadge);
  delay(10);
  rf95.waitPacketSent();
  lastPublishTime = now;
}

void checkForMessage(unsigned long now)
{
  while (rf95.available())
  {
    if (rf95.recv(radioPacketStation.buffer, &sizeOfRadioPacketStation))
    {
      if (radioPacketStation.itype == 1) //check to see that it is a station
      {
        if (old_imsid != radioPacketStation.imsid)
        {
          stationStatus.iwarn = ((radioPacketStation.istatus >> 0) & 0x01);
          stationStatus.iauth = ((radioPacketStation.istatus >> 1) & 0x01);
          if (stationStatus.iauth > 0)
          {
            if (radioPacketStation.iaddr == DEVICE_ADDRESS) 
            {
              if ((badgeStatus.ichrg > 0) && (badgeStatus.iauth == 0))
              {
                badgeStatus.iauth = stationStatus.iauth;
                transmitMsg(now);
                for (int ii = 0; ii < 3; ++ii) soundBeep(50);
                lastAuthTime = now;
              }
            }
          }
          if ((badgeStatus.ichrg == 0) && (badgeStatus.iauth > 0))
          {
            if ((radioPacketStation.iaddr == DEVICE_ADDRESS) || (radioPacketStation.iaddr == 0) )
            {
              iwarn = stationStatus.iwarn;
              if (iwarn > 0) soundSOS();
            }
          }
          old_imsid = radioPacketStation.imsid;
        }
      }
    }
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
  digitalWrite(mesgPin, LOW);
  digitalWrite(commLEDPin, LOW);
  delay(idelay);
  digitalWrite(mesgPin, HIGH);
  digitalWrite(commLEDPin, HIGH);
  delay(idelay);
  delay(idelay);
  digitalWrite(mesgPin, LOW);
  digitalWrite(commLEDPin, LOW);
  delay(idelay);
}
