boolean printDiagnostics = true;

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
uint8_t sizeOfStationCommand = 6;

union CubeData
{
  struct
  {
    int16_t state;
    int16_t watchdog;
    int16_t sendStationCommand;
    int16_t newData;
    StationReport stationReport;
    StationCommand stationCommand;
  };
  byte buffer[32];
};
CubeData cubeData;


#include "BlinkyPicoWCube.h"


int commLEDPin = 16;
int commLEDBright = 255; 
int resetButtonPin = 15;

unsigned long lastPublishTime;
unsigned long publishInterval = 2000;

void setupServerComm()
{
  // Optional setup to overide defaults
  if (printDiagnostics) Serial.begin(115200);
  BlinkyPicoWCube.setChattyCathy(printDiagnostics);
  BlinkyPicoWCube.setWifiTimeoutMs(20000);
  BlinkyPicoWCube.setWifiRetryMs(20000);
  BlinkyPicoWCube.setMqttRetryMs(3000);
  BlinkyPicoWCube.setResetTimeoutMs(10000);
  BlinkyPicoWCube.setHdwrWatchdogMs(8000);
  BlinkyPicoWCube.setBlMqttKeepAlive(8);
  BlinkyPicoWCube.setBlMqttSocketTimeout(4);
  BlinkyPicoWCube.setMqttLedFlashMs(10);
  BlinkyPicoWCube.setWirelesBlinkMs(100);
  BlinkyPicoWCube.setMaxNoMqttErrors(5);
  BlinkyPicoWCube.setMaxNoConnectionAttempts(5);
  
  // Must be included
  BlinkyPicoWCube.init(commLEDPin, commLEDBright, resetButtonPin);
}

void setupCube()
{
  cubeData.state = 1;
  cubeData.watchdog = 0;
  cubeData.sendStationCommand = 0;
  cubeData.newData = 0;
  lastPublishTime = millis();
  Serial1.begin(57600);
}

void cubeLoop()
{
  unsigned long nowTime = millis();
  while (Serial1.available() > 0) 
  {
    Serial1.readBytes(cubeData.stationReport.buffer, sizeOfStationReport);
    cubeData.watchdog = cubeData.watchdog + 1;
    cubeData.newData = 1;
    if (cubeData.watchdog > 32765) cubeData.watchdog = 0;
    BlinkyPicoWCube.publishToServer();
    BlinkyPicoWCube.loop();
    if (printDiagnostics)
    {
      Serial.print("addr: ");
      Serial.print(cubeData.stationReport.iaddr);
      Serial.print(", vbat: ");
      Serial.print(cubeData.stationReport.ivbat);
      Serial.print(", vusb: ");
      Serial.print(cubeData.stationReport.ivusb);
      Serial.print(", butt: ");
      Serial.print(cubeData.stationReport.ibutt);
      Serial.print(", move: ");
      Serial.print(cubeData.stationReport.imove);
      Serial.print(", chrg: ");
      Serial.print(cubeData.stationReport.ichrg);
      Serial.print(", auth: ");
      Serial.print(cubeData.stationReport.iauth);
      Serial.print(", wdog: ");
      Serial.print(cubeData.stationReport.iwatchdog);
      Serial.print(", rssi: ");
      Serial.println(cubeData.stationReport.irssi);
    }
    cubeData.newData = 0;
    cubeData.stationReport.iaddr = 0;
    cubeData.stationReport.ivbat = 0;
    cubeData.stationReport.ivusb = 0;
    cubeData.stationReport.ibutt = 0;
    cubeData.stationReport.imove = 0;
    cubeData.stationReport.ichrg = 0;
    cubeData.stationReport.iauth = 0;
    cubeData.stationReport.iaddr = 0;
    cubeData.stationReport.iwatchdog = 0;
    cubeData.stationReport.irssi = 0;
    lastPublishTime = nowTime;
  }
  
  if ((nowTime - lastPublishTime) > publishInterval)
  {
    lastPublishTime = nowTime;
    cubeData.watchdog = cubeData.watchdog + 1;
    if (cubeData.watchdog > 32760) cubeData.watchdog= 0 ;
    BlinkyPicoWCube.publishToServer();
  } 
}

void handleNewSettingFromServer(uint8_t address)
{
  switch(address)
  {
    case 0:
      break;
    case 1:
      break;
    case 2:
      if (cubeData.sendStationCommand > 0)
      {
        Serial1.write(cubeData.stationCommand.buffer, sizeOfStationCommand);
        cubeData.sendStationCommand = 0;
      }
      break;
    case 3:
      break;
    case 4:
      break;
    case 5:
      break;
    default:
      break;
  }
}
