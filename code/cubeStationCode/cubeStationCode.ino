#define RF_FREQ  433.200
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

const int commLEDPin = 13;

union StationReport
{
  struct
  {
    int16_t iaddr;
    int16_t ivbat;
    int16_t ivusb;
    int16_t iwatchdog;
    int16_t ibutt;
    int16_t imove;
    int16_t ichrg;
    int16_t iauth;
    int16_t irssi;
  };
  byte buffer[18];
};
StationReport stationReport;
uint8_t sizeOfStationReport = 18;

union StationCommand
{
  struct
  {
    int16_t iaddr;
    int16_t iwarn;
    int16_t iauth;
  };
  byte buffer[6];
};
StationCommand stationCommand;
uint8_t sizeOfStationCommand = 6;

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

void setup() 
{
  Serial.begin(57600);
  Serial1.begin(57600);
  delay(3000);
  
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

  pinMode(commLEDPin, OUTPUT);
  digitalWrite(commLEDPin, LOW);
  
}
void loop() 
{

  while (rf95.available())
  {
    if (rf95.recv((uint8_t *)&radioPacketBadge, &sizeOfRadioPacketBadge))
    {
      digitalWrite(commLEDPin, HIGH);
      stationReport.iaddr = (int16_t) radioPacketBadge.iaddr;
      stationReport.ivbat = (int16_t) radioPacketBadge.ivbat;
      stationReport.ivusb = (int16_t) radioPacketBadge.ivusb;
      stationReport.iwatchdog = (int16_t) radioPacketBadge.iwatchdog;
      stationReport.ibutt = (int16_t) ((radioPacketBadge.istatus >> 0) & 0x01);
      stationReport.imove = (int16_t) ((radioPacketBadge.istatus >> 1) & 0x01);
      stationReport.ichrg = (int16_t) ((radioPacketBadge.istatus >> 2) & 0x01);
      stationReport.iauth = (int16_t) ((radioPacketBadge.istatus >> 3) & 0x01);
      stationReport.irssi = (int16_t) rf95.lastRssi();
      Serial1.write(stationReport.buffer, sizeOfStationReport);
        
      Serial.print("addr: ");
      Serial.print(stationReport.iaddr);
      Serial.print(", vbat: ");
      Serial.print(stationReport.ivbat);
      Serial.print(", vusb: ");
      Serial.print(stationReport.ivusb);
      Serial.print(", butt: ");
      Serial.print(stationReport.ibutt);
      Serial.print(", move: ");
      Serial.print(stationReport.imove);
      Serial.print(", chrg: ");
      Serial.print(stationReport.ichrg);
      Serial.print(", auth: ");
      Serial.print(stationReport.iauth);
      Serial.print(", wdog: ");
      Serial.print(stationReport.iwatchdog);
      Serial.print(", rssi: ");
      Serial.println(stationReport.irssi);
        
      delay(200);
      digitalWrite(commLEDPin, LOW);

    }
  }
  while (Serial1.available() > 0) 
  {
    digitalWrite(commLEDPin, HIGH);
    Serial1.readBytes(stationCommand.buffer, sizeOfStationCommand);
    radioPacketStation.iaddr = stationCommand.iaddr;
    radioPacketStation.istatus = 0;
    if (stationCommand.iwarn > 0) radioPacketStation.istatus = radioPacketStation.istatus + 1;
    if (stationCommand.iauth > 0) radioPacketStation.istatus = radioPacketStation.istatus + 2;
    rf95.send((uint8_t *)&radioPacketStation, sizeOfRadioPacketStation);
    delay(10);
    rf95.waitPacketSent();
    digitalWrite(commLEDPin, LOW);  
  }

}
