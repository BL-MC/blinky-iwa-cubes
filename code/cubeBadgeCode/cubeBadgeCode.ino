#define DEVICE_ADDRESS        10
#define NO_MOTION_INTERVAL    120000
#define NO_MOTION_WARN         60000
#define RF_FREQ               433.200
#define PUBLISH_INTERVAL      300000
#define PUBLISH_OFFSET        10000
#define BUTTON_DEBOUNCE_DELAY 50
#define CHARGE_DEBOUNCE_DELAY 100
#define BATTERY_OK_LEVEL      642

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
const int movePin = 10;

struct RadioPacketBadge
{
  byte iaddr;
  byte istatus;
  int16_t ivbat;
  int16_t ivusb;
  int16_t iwatchdog;
};

RadioPacketBadge radioPacketBadge;
uint8_t sizeOfRadioPacketBadge = 8;

struct RadioPacketStation
{
  byte iaddr;
  byte istatus;
};


RadioPacketStation radioPacketStation;
uint8_t sizeOfRadioPacketStation = 2;

struct BadgeStatus
{
  int ibutt = 0;
  int imove = 0;
  int ichrg = 0;
  int iauth = 0;
};
BadgeStatus badgeStatus;

struct StationStatus
{
  int iwarn = 0;
  int iauth = 0;
};
StationStatus stationStatus;

int lastButtonState = 0;  
int lastChargeState = 0;  

unsigned long lastButtonDebounceTime = 0;  
unsigned long lastChargeDebounceTime = 0;  
unsigned long lastMotionErrorTime = 0;  
unsigned long lastMotionWarnTime = 0;  
unsigned long publishInterval = PUBLISH_INTERVAL;     
unsigned long lastPublishTime = 0;     

void setup() 
{
  pinMode(A0, INPUT);
  pinMode(vbatPin, INPUT);
  pinMode(vusbPin, INPUT);
  pinMode(buttPin, INPUT);
  pinMode(mesgPin, OUTPUT);
  pinMode(movePin, INPUT);
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
    
  for (int ii = 0; ii < 3; ++ii) soundBeep(500, commLEDPin);
  radioPacketBadge.iwatchdog = 0;
  randomSeed(analogRead(A0));
  publishInterval = publishInterval + (unsigned long) random(PUBLISH_OFFSET);
  lastPublishTime = millis();
  lastMotionErrorTime = lastPublishTime;
  lastMotionWarnTime = lastPublishTime;
   
}
void loop() 
{
  unsigned long now =  millis();
  radioPacketBadge.ivbat = (int16_t) analogRead(vbatPin);
  radioPacketBadge.ivusb = (int16_t) analogRead(vusbPin);
  checkButton(now);
  checkMotion(now);
  checkCharging(now);
  if ((now - lastPublishTime) > publishInterval) transmitMsg(now);
  checkForMessage();
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
    if (reading != badgeStatus.ibutt) 
    {
      badgeStatus.ibutt = reading;
      if (badgeStatus.ibutt == 1) 
      {
        if (badgeStatus.ichrg == 0)
        {
          transmitMsg(now);
          for (int ii = 0; ii < 3; ++ii) soundBeep(500, mesgPin);
        }
        else
        {
          if (radioPacketBadge.ivbat >= BATTERY_OK_LEVEL) soundBeep(500, mesgPin);
        }
      }
    }
  }
  lastButtonState = reading;  
}
void checkMotion(unsigned long now)
{
  int movePinValue = digitalRead(movePin);
  
  accel.readRegister(ADXL345_REG_INT_SOURCE);
  if (movePinValue > 0)
  {
    lastMotionErrorTime = now;
    lastMotionWarnTime = now;
    badgeStatus.imove = 0;
  }
  if (badgeStatus.ichrg == 0)
  {
    if ((now - lastMotionWarnTime) > NO_MOTION_WARN)
    {
      lastMotionWarnTime = now;
      soundBeep(500, mesgPin);
      accel.readRegister(ADXL345_REG_INT_SOURCE);
    }
    if ((now - lastMotionErrorTime) > NO_MOTION_INTERVAL)
    {
      lastMotionWarnTime = now;
      lastMotionErrorTime = now;
      badgeStatus.imove = 1;
      transmitMsg(now);
      for (int ii = 0; ii < 3; ++ii) soundBeep(500, mesgPin);
      badgeStatus.imove = 0;
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
      if (badgeStatus.ichrg > 0) badgeStatus.iauth = 0;
      transmitMsg(now);
      digitalWrite(commLEDPin, badgeStatus.ichrg);
    }
  }
  lastChargeState = chargeReading;  
}

void transmitMsg(unsigned long now)
{
  radioPacketBadge.iwatchdog = radioPacketBadge.iwatchdog + 1;
  if (radioPacketBadge.iwatchdog > 32765) radioPacketBadge.iwatchdog = 0;

  radioPacketBadge.istatus = 0;
  if (badgeStatus.ibutt > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 1;
  if (badgeStatus.imove > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 2;
  if (badgeStatus.ichrg > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 4;
  if (badgeStatus.iauth > 0) radioPacketBadge.istatus = radioPacketBadge.istatus + 8;
  
  rf95.send((uint8_t *)&radioPacketBadge, sizeOfRadioPacketBadge);
  delay(10);
  rf95.waitPacketSent();
  if (badgeStatus.ichrg == 0) soundBeep(50, commLEDPin);
  lastPublishTime = now;
}

void checkForMessage()
{
  while (rf95.available())
  {
    if (rf95.recv((uint8_t *)&radioPacketStation, &sizeOfRadioPacketStation))
    {
      stationStatus.iwarn = ((radioPacketStation.istatus >> 0) & 0x01);;
      stationStatus.iauth = ((radioPacketStation.istatus >> 1) & 0x01);;
      if (radioPacketStation.iaddr == DEVICE_ADDRESS) 
      {
        badgeStatus.iauth = stationStatus.iauth;
        if (badgeStatus.iauth > 0) soundBeep(1000, mesgPin);
      }
      if ((stationStatus.iwarn > 0)  && (badgeStatus.ichrg == 0) ) soundSOS(mesgPin);
    }
  }
}
void soundSOS(int ipin)
{
  int idelay[3] = {250,500,250};
  for (int is = 0; is < 3; ++is)
  {
    for (int ii = 0; ii < 3; ++ii)
    {
      soundBeep(idelay[is],ipin);
    }
  }  
}
void soundBeep(int idelay, int ipin)
{
  delay(idelay);
  digitalWrite(ipin, HIGH);
  delay(idelay);
  delay(idelay);
  digitalWrite(ipin, LOW);
  delay(idelay);
}
