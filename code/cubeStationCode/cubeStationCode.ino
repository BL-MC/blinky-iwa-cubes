boolean chattyCathy = false;
#define RF_FREQ  433.200
#include <SPI.h>
#include "RH_RF95.h"
#include "CRC16.h"
#include "CRC.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);
CRC16 crc;

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
  uint8_t buffer[18];
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
    int16_t imsid;
  };
  uint8_t buffer[8];
};
StationCommand stationCommand;
uint8_t sizeOfStationCommand = 8;

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
    uint16_t crc;
  };
  uint8_t buffer[14];
};
RadioPacketBadge radioPacketBadge;
uint8_t sizeOfRadioPacketBadge = 14;

union RadioPacketStation
{
  struct
  {
    int16_t itype;
    int16_t iaddr;
    int16_t istatus;
    int16_t imsid;
    int16_t extra[2];
    uint16_t crc;
  };
  uint8_t buffer[14];
};
RadioPacketStation radioPacketStation;
uint8_t sizeOfRadioPacketStation = 14;

void setup() 
{
  if (chattyCathy)
  {
    Serial.begin(9600);
    delay(3000);
    Serial.println("Starting sketch");
  }
  Serial1.begin(9600);
  
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
    if (rf95.recv(radioPacketBadge.buffer, &sizeOfRadioPacketBadge))
    {
      crc.restart();
      for (int ii = 0; ii < (sizeOfRadioPacketBadge - 2); ii++)
      {
        crc.add(radioPacketBadge.buffer[ii]);
      }
      uint16_t crcCheck = crc.calc();
      if (chattyCathy)
      {
        Serial.print("crRecv  : ");
        Serial.println(radioPacketBadge.crc, HEX);
        Serial.print("crcCheck: ");
        Serial.println(crcCheck, HEX);
      }
      
      if ((radioPacketBadge.itype == 0) && (radioPacketBadge.crc == crcCheck)) //check to see that it is a badge
      {
        digitalWrite(commLEDPin, HIGH);
        stationReport.iaddr = radioPacketBadge.iaddr;
        stationReport.ivbat = radioPacketBadge.ivbat;
        stationReport.ivusb = radioPacketBadge.ivusb;
        stationReport.iwatchdog = radioPacketBadge.iwatchdog;
        stationReport.ibutt =  ((radioPacketBadge.istatus >> 0) & 0x01);
        stationReport.imove =  ((radioPacketBadge.istatus >> 1) & 0x01);
        stationReport.ichrg =  ((radioPacketBadge.istatus >> 2) & 0x01);
        stationReport.iauth =  ((radioPacketBadge.istatus >> 3) & 0x01);
        stationReport.irssi =  rf95.lastRssi();
        Serial1.write(stationReport.buffer, sizeOfStationReport);
          
        if (chattyCathy)
        {
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
          Serial.print(radioPacketBadge.iwatchdog);
          Serial.print(", rssi: ");
          Serial.println(stationReport.irssi);
        }
          
//        delay(200);
        digitalWrite(commLEDPin, LOW);
      }
    }
  }
  while (Serial1.available() > 0) 
  {
    digitalWrite(commLEDPin, HIGH);
    Serial1.readBytes(stationCommand.buffer, sizeOfStationCommand);
    radioPacketStation.itype = 1; // tell that it is a badge
    radioPacketStation.iaddr = stationCommand.iaddr;
    radioPacketStation.imsid = stationCommand.imsid;
    radioPacketStation.istatus = 0;
    if (stationCommand.iwarn > 0) radioPacketStation.istatus = radioPacketStation.istatus + 1;
    if (stationCommand.iauth > 0) radioPacketStation.istatus = radioPacketStation.istatus + 2;

    crc.restart();
    for (int ii = 0; ii < (sizeOfRadioPacketStation - 2); ii++)
    {
      crc.add(radioPacketStation.buffer[ii]);
    }
    radioPacketStation.crc = crc.calc();
    
    rf95.setCADTimeout(20000);
    if (chattyCathy)
    {
        Serial.print("  Sending msg....");
    }
    rf95.send(radioPacketStation.buffer, sizeOfRadioPacketStation);
    rf95.waitPacketSent(4000);
    if (chattyCathy)
    {
        Serial.println("finshed");
    }
    digitalWrite(commLEDPin, LOW);  
  }

}
