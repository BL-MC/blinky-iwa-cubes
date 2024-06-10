#include <WiFi.h>
#include <PubSubClient.h>
#include "LittleFS.h"

#define BL_MQTT_KEEP_ALIVE    8
#define BL_MQTT_SOCKETTIMEOUT 3
#define HDWR_WATCHDOG_MS      8000
#define WIFI_TIMEOUT          20000
#define WIFI_RETRY            20000
#define MQTT_RETRY            3000
#define RESET_TIMEOUT         10000
#define MQTT_LED_FLASH_MS     10
#define WIRELESS_BLINK_MS     100
#define MAX_NO_MQTT_ERRORS    5
#define MAX_NO_CONNECTION_ATTEMPTS 5

union SubscribeData
{
  struct
  {
      uint8_t command;
      uint8_t address;
      int16_t value;
  };
  byte buffer[4];
};

SubscribeData subscribeData;
static void   BlinkyPicoWCubeWifiApButtonHandler();
static void   BlinkyPicoWCubeCallback(char* topic, byte* payload, unsigned int length);
void          handleNewSettingFromServer(uint8_t address);
void          loop();
void          loop1();
void          setup();
void          setup1();
void          setupServerComm();
void          setupCube();
void          cubeLoop();

class BlinkyPicoWCube
{
  private:
  
    String        g_ssid;
    String        g_wifiPassword;
    String        g_mqttServer;
    String        g_mqttUsername;
    String        g_mqttPassword;
    String        g_mqttPublishTopic;
    String        g_mqttSubscribeTopic;
    String        g_box;
    String        g_trayType;
    String        g_trayName;
    String        g_cubeType;

    WiFiClient    g_wifiClient;
    PubSubClient  g_mqttClient;
    WiFiServer*   g_wifiServer;

    boolean       g_init = true;
    int           g_wifiStatus = 0;
    unsigned long g_wifiTimeout = WIFI_TIMEOUT;
    unsigned long g_wifiRetry = WIFI_RETRY;
    unsigned long g_wifiLastTry = 0;
    unsigned long g_mqttRetry = MQTT_RETRY;
    unsigned long g_mqttLastTry = 0;
    unsigned long g_lastWirelessBlink = 0;
    unsigned long g_lastMsgTime = 0;
    boolean       g_publishNow = false;
    int           g_commLEDPin = LED_BUILTIN;
    int           g_commLEDBright = 255;
    boolean       g_commLEDState = false;
    boolean       g_wifiAccessPointMode = false;
    boolean       g_webPageServed = false;
    boolean       g_webPageRead = false;
    boolean       g_chattyCathy = false;
    SubscribeData g_subscribeData;
    unsigned int  g_cubeDataSize;
    int           g_resetButtonPin = -1;
    boolean       g_resetButtonDownState = false;
    unsigned long g_resetTimeout = RESET_TIMEOUT;
    uint32_t      g_hdwrWatchdogMs  = HDWR_WATCHDOG_MS;
    uint16_t      g_blMqttKeepAlive = BL_MQTT_KEEP_ALIVE;
    uint16_t      g_blMqttSocketTimeout = BL_MQTT_SOCKETTIMEOUT;
    int           g_mqttLedFlashMs = MQTT_LED_FLASH_MS;
    int           g_wirelessBlinkMs = WIRELESS_BLINK_MS;
    int           g_maxNoMqttErrors = MAX_NO_MQTT_ERRORS;
    int           g_noMqttErrors = 0; 
    int           g_maxConnectionAttempts = MAX_NO_CONNECTION_ATTEMPTS;
    int           g_noConnectionAttempts = 0; 
    volatile unsigned long g_resetButtonDownTime = 0;    

    void          setupWifiAp();
    boolean       reconnect(); 
    void          setup_wifi();
    void          readWebPage();
    void          serveWebPage();
    String        replaceHtmlEscapeChar(String inString);
    void          setCommLEDPin(boolean ledState);

  public:
    BlinkyPicoWCube();
    void          loop();
    void          init(int commLEDPin,  int commLEDBright, int resetButtonPin);
    void          setChattyCathy(boolean chattyCathy);
    void          mqttCubeCallback(char* topic, byte* payload, unsigned int length);
    void          wifiApButtonHandler();
    void          setWifiTimeoutMs(unsigned long wifiTimeout){g_wifiTimeout = wifiTimeout;};
    void          setWifiRetryMs(unsigned long wifiRetry){g_wifiRetry = wifiRetry;};
    void          setMqttRetryMs(unsigned long mqttRetry){g_mqttRetry = mqttRetry;};
    void          setResetTimeoutMs(unsigned long resetTimeout){g_resetTimeout = resetTimeout;};
    void          setHdwrWatchdogMs(uint32_t hdwrWatchdogMs){g_hdwrWatchdogMs = hdwrWatchdogMs;};
    void          setBlMqttKeepAlive(uint16_t blMqttKeepAlive){g_blMqttKeepAlive = blMqttKeepAlive;};
    void          setBlMqttSocketTimeout(uint16_t blMqttSocketTimeout){g_blMqttSocketTimeout = blMqttSocketTimeout;};
    void          setMqttLedFlashMs(int mqttLedFlashMs){g_mqttLedFlashMs = mqttLedFlashMs;};
    void          setWirelesBlinkMs(int wirelessBlinkMs){g_wirelessBlinkMs = wirelessBlinkMs;};
    void          setMaxNoMqttErrors(int maxNoMqttErrors){g_maxNoMqttErrors = maxNoMqttErrors;};
    void          setMaxNoConnectionAttempts(int maxConnectionAttempts){g_maxConnectionAttempts = maxConnectionAttempts;};
    void          publishToServer();

};
BlinkyPicoWCube::BlinkyPicoWCube()
{
  g_cubeDataSize = sizeof(cubeData.buffer);
}
void BlinkyPicoWCube::setCommLEDPin(boolean ledState)
{
  g_commLEDState = ledState;
  if (g_commLEDPin == LED_BUILTIN)
  {
    digitalWrite(g_commLEDPin, g_commLEDState);
  }
  else
  {
    if (g_commLEDState)
    {
      analogWrite(g_commLEDPin, g_commLEDBright);
    }
    else
    {
      analogWrite(g_commLEDPin, 0);
    }
  }
}

void BlinkyPicoWCube::loop()
{
  rp2040.wdt_reset();;
  
  if (g_wifiAccessPointMode)
  {
    serveWebPage();
    readWebPage();
    delay(100);
  }
  else
  {
    if (g_resetButtonPin > 0 )
    {
      if (digitalRead(g_resetButtonPin))
      {
        if ((millis() - g_resetButtonDownTime) < g_resetTimeout)
        {
          setCommLEDPin(false);
          return;
        }
        else
        {
          setCommLEDPin(true);
          return;
        }
      }
    }
    g_wifiStatus = WiFi.status();
    unsigned long now = millis();
    if (g_wifiStatus == WL_CONNECTED)
    {
      if (!g_mqttClient.connected()) 
      {
        if (!reconnect()) return;
      }
      g_mqttClient.loop();
    
      if (g_publishNow)
      {
          g_lastWirelessBlink = 0;
          g_commLEDState = true;
          setCommLEDPin(g_commLEDState);
          g_publishNow = false;
          g_mqttClient.publish(g_mqttPublishTopic.c_str(), cubeData.buffer, g_cubeDataSize);
//          if (g_chattyCathy) Serial.print("Publishing to MQTT ");
//          if (g_chattyCathy) Serial.print(g_mqttPublishTopic);
//          if (g_chattyCathy) Serial.print(" ");
//          if (g_chattyCathy) Serial.println(g_cubeData.watchdog);
          g_lastMsgTime = now;
      }
      if (g_commLEDState)
      {
        if (now - g_lastMsgTime > g_mqttLedFlashMs) 
        {
          setCommLEDPin(false);
        }
      }
    }
    else
    {
      setup_wifi();
      if ((now - g_lastWirelessBlink) > g_wirelessBlinkMs)
      {
        setCommLEDPin(!g_commLEDState);
        g_lastWirelessBlink = now;
      }
    }
  }

}
void BlinkyPicoWCube::init(int commLEDPin, int commLEDBright, int resetButtonPin)
{
  g_init = true;
  g_wifiClient = WiFiClient();
  g_mqttClient = PubSubClient(g_wifiClient);
  g_wifiServer = new WiFiServer(80);
  g_commLEDBright =  commLEDBright;

  if (g_chattyCathy) Serial.println("Reading creds.txt file");
  LittleFS.begin();
  File file = LittleFS.open("/creds.txt", "r");
  if (file) 
  {
      String lines[8];
      String data = file.readString();
      int startPos = 0;
      int stopPos = 0;
      for (int ii = 0; ii < 9; ++ii)
      {
        startPos = data.indexOf("{") + 1;
        stopPos = data.indexOf("}");
        lines[ii] = data.substring(startPos,stopPos);
        if (g_chattyCathy) Serial.println(lines[ii]);
        data = data.substring(stopPos + 1);
      }
      g_ssid               = lines[0];
      g_wifiPassword       = lines[1];
      g_mqttServer         = lines[2];
      g_mqttUsername       = lines[3];
      g_mqttPassword       = lines[4];
      g_box                = lines[5];
      g_trayType           = lines[6];
      g_trayName           = lines[7];
      g_cubeType           = lines[8];
      g_mqttSubscribeTopic = g_box + "/" + g_cubeType + "/" + g_trayType + "/" + g_trayName + "/setting";
      g_mqttPublishTopic   = g_box + "/" + g_cubeType + "/" + g_trayType + "/" + g_trayName + "/reading";
      file.close();
      g_mqttClient.setKeepAlive(g_blMqttKeepAlive);
      g_mqttClient.setSocketTimeout(g_blMqttSocketTimeout);
  }
  else
  {
    if (g_chattyCathy) Serial.println("file open failed");
  }

  if (g_chattyCathy) Serial.println("Starting Communications");
  g_mqttClient.setServer(g_mqttServer.c_str(), 1883);
  g_mqttClient.setCallback(BlinkyPicoWCubeCallback);
  g_commLEDPin = commLEDPin;
  pinMode(g_commLEDPin, OUTPUT);
  setCommLEDPin(false);
  g_resetButtonPin = resetButtonPin;
  if (g_resetButtonPin > 0 )
  { 
    attachInterrupt(digitalPinToInterrupt(g_resetButtonPin), BlinkyPicoWCubeWifiApButtonHandler, CHANGE);
    pinMode(g_resetButtonPin, INPUT);
  }
  rp2040.wdt_begin(g_hdwrWatchdogMs);
  g_wifiLastTry = 0;
  setup_wifi();
  g_init = false;
}
void BlinkyPicoWCube::setup_wifi() 
{
  if (((millis() - g_wifiLastTry) < g_wifiRetry) && !g_init) return;
  setCommLEDPin(true);
  
  if (g_chattyCathy) Serial.println();
  if (g_chattyCathy) Serial.print("Connecting to ");
  if (g_chattyCathy) Serial.println(g_ssid);
  
  if (g_chattyCathy) Serial.print("SSID: ");
  if (g_chattyCathy) Serial.println(g_ssid);
  if (g_chattyCathy) Serial.print("Wifi Password: ");
  if (g_chattyCathy) Serial.println(g_wifiPassword);

  WiFi.mode(WIFI_STA);
  WiFi.begin(g_ssid.c_str(), g_wifiPassword.c_str());
  
  g_wifiLastTry = millis();
  g_wifiStatus = WiFi.status();
  while ((g_wifiStatus != WL_CONNECTED) && ((millis() - g_wifiLastTry) < g_wifiTimeout))
  {
    g_wifiStatus = WiFi.status();
    delay(500);
    if (g_chattyCathy) Serial.print(".");
    setCommLEDPin(!g_commLEDState);

    rp2040.wdt_reset();
  }
  g_wifiLastTry = millis();

  if (g_wifiStatus == WL_CONNECTED)
  {
    if (g_chattyCathy) Serial.println("");
    if (g_chattyCathy) Serial.println("WiFi connected");
    if (g_chattyCathy) Serial.println("IP address: ");
    if (g_chattyCathy) Serial.println(WiFi.localIP());
    if (g_chattyCathy)
    {
      byte mac[6];
      WiFi.macAddress(mac);
      Serial.print("MAC: ");
      Serial.print(mac[0],HEX);
      Serial.print(":");
      Serial.print(mac[1],HEX);
      Serial.print(":");
      Serial.print(mac[2],HEX);
      Serial.print(":");
      Serial.print(mac[3],HEX);
      Serial.print(":");
      Serial.print(mac[4],HEX);
      Serial.print(":");
      Serial.println(mac[5],HEX);
    }
    g_noConnectionAttempts = 0; 
  }
  else
  {
    g_noConnectionAttempts = g_noConnectionAttempts + 1;
    if (g_chattyCathy) Serial.println("");
    if (g_chattyCathy) Serial.print("WiFi not connected after ");
    if (g_chattyCathy) Serial.print(g_noConnectionAttempts);
    if (g_chattyCathy) Serial.println(" attempts");
    if (g_noConnectionAttempts > g_maxConnectionAttempts)
    {
      if (g_chattyCathy)  Serial.println("Too many wifi attempts. Rebooting...");
      delay(20000);
    }
  }
}


boolean BlinkyPicoWCube::reconnect() 
{
  unsigned long now = millis();
  boolean connected = g_mqttClient.connected();
  if (connected) return true;
  if ((now - g_mqttLastTry) < g_mqttRetry) return false;

  if (g_chattyCathy) Serial.print("Attempting MQTT connection using ID...");
// Create a random client ID
  String mqttClientId = g_mqttUsername + "-" + g_trayType + "-" + g_trayName;
//  mqttClientId += String(random(0xffff), HEX);
  if (g_chattyCathy) Serial.print(mqttClientId);
  rp2040.wdt_reset();
  connected = g_mqttClient.connect(mqttClientId.c_str(),g_mqttUsername.c_str(), g_mqttPassword.c_str());
  rp2040.wdt_reset();
  g_mqttLastTry = now;
  if (connected) 
  {
    if (g_chattyCathy) Serial.println("...connected");
    g_mqttClient.subscribe(g_mqttSubscribeTopic.c_str());
    g_noMqttErrors = 0;
    return true;
  } 
  int mqttState = g_mqttClient.state();
  if (g_chattyCathy) Serial.print(" failed, rc=");
  if (g_chattyCathy) Serial.print(mqttState);
  if (g_chattyCathy) Serial.print(": ");

  switch (mqttState) 
  {
    case -4:
      if (g_chattyCathy) Serial.println("MQTT_CONNECTION_TIMEOUT");
      break;
    case -3:
      if (g_chattyCathy) Serial.println("MQTT_CONNECTION_LOST");
      break;
    case -2:
      g_noMqttErrors = g_noMqttErrors + 1;
      if (g_chattyCathy) Serial.print("Number of MQTT connection attempts: ");
      if (g_chattyCathy) Serial.print(g_noMqttErrors);
      if (g_noMqttErrors > g_maxNoMqttErrors)
      {
        if (g_chattyCathy) Serial.println("");
        if (g_chattyCathy) Serial.println("Too may attempts. Rebooting...");
        delay(20000);
      }
      break;
    case -1:
      if (g_chattyCathy) Serial.println("MQTT_DISCONNECTED");
      break;
    case 0:
      if (g_chattyCathy) Serial.println("MQTT_CONNECTED");
      break;
    case 1:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_BAD_PROTOCOL");
      break;
    case 2:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_BAD_CLIENT_ID");
      break;
    case 3:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_UNAVAILABLE");
      break;
    case 4:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_BAD_CREDENTIALS");
      break;
    case 5:
      if (g_chattyCathy) Serial.println("MQTT_CONNECT_UNAUTHORIZED");
      break;
    default:
      break;
  }
  return false;
}
void BlinkyPicoWCube::readWebPage()
{
  if (!g_webPageServed) return;
  if (g_webPageRead) return;
  WiFiClient client = g_wifiServer->available();

  if (client) 
  {
    String header = "";
    while (client.connected() && !g_webPageRead) 
    {
      rp2040.wdt_reset();;
      if (client.available()) 
      {
        char c = client.read();
        header.concat(c);
      }
      else
      {
        break;
      }
    }
    if (header.indexOf("POST") >= 0)
    {
      if (g_chattyCathy) Serial.println(header);
      String data = header.substring(header.indexOf("ssid="),header.length());
      g_ssid               = replaceHtmlEscapeChar(data.substring(5,data.indexOf("&pass=")));
      g_wifiPassword       = replaceHtmlEscapeChar(data.substring(data.indexOf("&pass=")  + 6,data.indexOf("&serv=")));
      g_mqttServer         = replaceHtmlEscapeChar(data.substring(data.indexOf("&serv=")  + 6,data.indexOf("&unam=")));
      g_mqttUsername       = replaceHtmlEscapeChar(data.substring(data.indexOf("&unam=")  + 6,data.indexOf("&mpas=")));
      g_mqttPassword       = replaceHtmlEscapeChar(data.substring(data.indexOf("&mpas=")  + 6,data.indexOf("&bbox=")));
      g_box                = replaceHtmlEscapeChar(data.substring(data.indexOf("&bbox=")  + 6,data.indexOf("&tryt=")));
      g_trayType           = replaceHtmlEscapeChar(data.substring(data.indexOf("&tryt=")  + 6,data.indexOf("&tryn=")));
      g_trayName           = replaceHtmlEscapeChar(data.substring(data.indexOf("&tryn=")  + 6,data.indexOf("&cube=")));
      g_cubeType           = replaceHtmlEscapeChar(data.substring(data.indexOf("&cube=")  + 6,data.length()));
      g_mqttSubscribeTopic = g_box + "/" + g_cubeType + "/" + g_trayType + "/" + g_trayName + "/setting";
      g_mqttPublishTopic   = g_box + "/" + g_cubeType + "/" + g_trayType + "/" + g_trayName + "/reading";
      if (g_chattyCathy) Serial.println(header);
      if (g_chattyCathy) Serial.println(data);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_ssid);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_wifiPassword);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_mqttServer);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_mqttUsername);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_mqttPassword);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_mqttPublishTopic);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_trayType);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_trayName);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_cubeType);
      if (g_chattyCathy) Serial.print("X");
      if (g_chattyCathy) Serial.print(g_mqttSubscribeTopic);
      if (g_chattyCathy) Serial.println("X");
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println("Connection: close");  // the connection will be closed after completion of the response
      client.println();
      client.println("<!DOCTYPE HTML>");
      client.println("<html><head><title>Blinky-Lite Credentials</title><style>");
      client.println("body{background-color: #083357 !important;font-family: Arial;}");
      client.println(".labeltext{color: white;font-size:250%;}");
      client.println(".formtext{color: black;font-size:250%;}");
      client.println(".cell{padding-bottom:25px;}");
      client.println("</style></head><body>");
      client.println("<h1 style=\"color:white;font-size:300%;text-align: center;\">Blinky-Lite Credentials</h1><hr><div>");
      client.println("<div>");
      client.println("<h1 style=\"color:yellow;font-size:300%;text-align: center;\">Accepted</h1><div>");
      client.println("</div>");
      client.println("</body></html>");
      g_webPageRead = true;
    }
    delay(100);
    client.stop();
    if (g_webPageRead)
    {
      g_wifiAccessPointMode = false;
      WiFi.disconnect();
      if (g_chattyCathy) Serial.println("Writing creds.txt file");
      File file = LittleFS.open("/creds.txt", "w");
      if (file) 
      {
        String line = "";
        line = "{" + g_ssid + "}";
        file.println(line);
        line = "{" + g_wifiPassword + "}";
        file.println(line);
        line = "{" + g_mqttServer + "}";
        file.println(line);
        line = "{" + g_mqttUsername + "}";
        file.println(line);
        line = "{" + g_mqttPassword + "}";
        file.println(line);
        line = "{" + g_box + "}";
        file.println(line);
        line = "{" + g_trayType + "}";
        file.println(line);
        line = "{" + g_trayName + "}";
        file.println(line);
        line = "{" + g_cubeType + "}";
        file.println(line);
        file.println("");
        file.println("");
        file.println("");
        file.close();
      }
      else
      {
        if (g_chattyCathy) Serial.println("file open failed");
      }
      if (g_chattyCathy) Serial.println("Rebooting");
      delay(10000); //watchdog will kick in
    }
  }
}
void BlinkyPicoWCube::serveWebPage()
{
  if (g_webPageServed) return;
  WiFiClient client = g_wifiServer->available();

  if (client) 
  {
    if (g_chattyCathy) Serial.println("new client");
    String header = "";
    while (client.connected() && !g_webPageServed) 
    {
      rp2040.wdt_reset();;
      if (client.available()) 
      {
        char c = client.read();
        header.concat(c);
      }
      else
      {
        break;
      }
    }
    if (g_chattyCathy) Serial.println(header);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");  // the connection will be closed after completion of the response
    client.println();
    client.println("<!DOCTYPE HTML>");
    client.println("<html><head><title>Blinky-Lite Credentials</title><style>");
    client.println("body{background-color: #083357 !important;font-family: Arial;}");
    client.println(".labeltext{color: white;font-size:250%;}");
    client.println(".formtext{color: black;font-size:250%;}");
    client.println(".cell{padding-bottom:25px;}");
    client.println("</style></head><body>");
    client.println("<h1 style=\"color:white;font-size:300%;text-align: center;\">Blinky-Lite Credentials</h1><hr><div>");
    client.println("<form action=\"/disconnected\" method=\"POST\">");
    client.println("<table align=\"center\">");
    
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"ssid\" class=\"labeltext\">SSID</label></td>");
    String tag = "<td class=\"cell\"><input name=\"ssid\" id=\"ssid\" type=\"text\" value=\"" + g_ssid + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
    
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"pass\" class=\"labeltext\">Wifi Password</label></td>");
    tag = "<td class=\"cell\"><input name=\"pass\" id=\"pass\" type=\"password\" value=\"" + g_wifiPassword + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
    
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"serv\" class=\"labeltext\">MQTT Server</label></td>");
    tag = "<td class=\"cell\"><input name=\"serv\" id=\"serv\" type=\"text\" value=\"" + g_mqttServer + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
    
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"unam\" class=\"labeltext\">MQTT Username</label></td>");
    tag = "<td class=\"cell\"><input name=\"unam\" id=\"unam\" type=\"text\" value=\"" + g_mqttUsername + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
    
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"mpas\" class=\"labeltext\">MQTT Password</label></td>");
    tag = "<td class=\"cell\"><input name=\"mpas\" id=\"mpas\" type=\"password\" value=\"" + g_mqttPassword + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
   
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"bbox\" class=\"labeltext\">Box</label></td>");
    tag = "<td class=\"cell\"><input name=\"bbox\" id=\"bbox\" type=\"text\" value=\"" + g_box + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");

    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"tryt\" class=\"labeltext\">Tray Type</label></td>");
    tag = "<td class=\"cell\"><input name=\"tryt\" id=\"tryt\" type=\"text\" value=\"" + g_trayType + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
      
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"tryn\" class=\"labeltext\">Tray Name</label></td>");
    tag = "<td class=\"cell\"><input name=\"tryn\" id=\"tryn\" type=\"text\" value=\"" + g_trayName + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
      
    client.println("<tr>");
    client.println("<td class=\"cell\"><label for=\"cube\" class=\"labeltext\">Cube Type</label></td>");
    tag = "<td class=\"cell\"><input name=\"cube\" id=\"cube\" type=\"text\" value=\"" + g_cubeType + "\" class=\"formtext\"/></td>";
    client.println(tag.c_str());
    client.println("</tr>");
    
    client.println("<tr>");
    client.println("<td></td>");
    client.println("<td><input type=\"submit\" class=\"formtext\"/></td>");
    client.println("</tr>");
    client.println("</table>");
    client.println("</form>");
    client.println("</div>");
    client.println("</body></html>");
    g_webPageRead = false;
    g_webPageServed = true;
    delay(100);
    client.stop();
    if (g_chattyCathy) Serial.println("client disconnected");
  }
}
String BlinkyPicoWCube::replaceHtmlEscapeChar(String inString)
{
  String outString = "";
  char hexBuff[3];
  char *hexptr;
  for (int ii = 0; ii < inString.length(); ++ii)
  {
    char testChar = inString.charAt(ii);
    if (testChar == '%')
    {
      inString.substring(ii + 1,ii + 3).toCharArray(hexBuff, 3);
      char specialChar = (char) strtoul(hexBuff, &hexptr, 16);
      outString += specialChar;
      ii = ii + 2;
    }
    else
    {
      outString += testChar;
    }
  }
  return outString;
}
void BlinkyPicoWCube::setupWifiAp()
{
  if (g_wifiAccessPointMode) return;
  setCommLEDPin(true);
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(g_trayType + "-" + g_trayName);
  if (g_chattyCathy) Serial.print("[+] AP Created with IP Gateway ");
  if (g_chattyCathy) Serial.println(WiFi.softAPIP());
  g_wifiServer = new WiFiServer(80);
  g_wifiAccessPointMode = true;
  g_wifiServer->begin();
  if (g_chattyCathy) Serial.print("Connected to wifi. My address:");
  IPAddress myAddress = WiFi.localIP();
  if (g_chattyCathy) Serial.println(myAddress);
  g_webPageServed = false;
  g_webPageRead = false;
}
void BlinkyPicoWCube::setChattyCathy(boolean chattyCathy)
{
  g_chattyCathy = chattyCathy;

  return;
}
void BlinkyPicoWCube::mqttCubeCallback(char* topic, byte* payload, unsigned int length)
{
  if (g_chattyCathy) Serial.print("Message arrived [");
  if (g_chattyCathy) Serial.print(topic);
  if (g_chattyCathy) Serial.print("] {command: ");
  for (int i = 0; i < length; i++) 
  {
    g_subscribeData.buffer[i] = payload[i];
  }
  if (g_chattyCathy) Serial.print(g_subscribeData.command);
  if (g_chattyCathy) Serial.print(", address: ");
  if (g_chattyCathy) Serial.print(g_subscribeData.address);
  if (g_chattyCathy) Serial.print(", value: ");
  if (g_chattyCathy) Serial.print(g_subscribeData.value);
  if (g_chattyCathy) Serial.println("}");
  if (g_subscribeData.command == 1)
  {
    cubeData.buffer[g_subscribeData.address * 2] = payload[2];
    cubeData.buffer[g_subscribeData.address * 2 + 1] = payload[3];
    handleNewSettingFromServer(g_subscribeData.address);
  }
  return;
}
void BlinkyPicoWCube::wifiApButtonHandler()
{
  if (digitalRead(g_resetButtonPin)) 
  {
    if (g_resetButtonDownState) return;
    if ((millis() - g_resetButtonDownTime) > 50)
    {
      if (g_chattyCathy) Serial.println("Button Down");
      g_resetButtonDownTime = millis();
      g_resetButtonDownState = true;
    }
  }
  else
  {
    if (!g_resetButtonDownState) return;
    if (g_chattyCathy) Serial.println("Button up");
    g_resetButtonDownState = false;
    if ((millis() - g_resetButtonDownTime) > g_resetTimeout)
    {
      setupWifiAp();
    }
  }
}
void BlinkyPicoWCube::publishToServer()
{
  g_publishNow = true;
}

BlinkyPicoWCube BlinkyPicoWCube;

void BlinkyPicoWCubeCallback(char* topic, byte* payload, unsigned int length) 
{
  BlinkyPicoWCube.mqttCubeCallback(topic, payload,  length);
}

void BlinkyPicoWCubeWifiApButtonHandler()
{
  delay(50);
  BlinkyPicoWCube.wifiApButtonHandler();
}

void loop() 
{
  BlinkyPicoWCube.loop();
  cubeLoop();
}
void setup() 
{
  setupServerComm();
  setupCube();
}
