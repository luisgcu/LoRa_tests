#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SX127XLT.h>
#include <ProgramLT_Definitions.h>
#include "Settings.h"
//#include <LedSync.h>
//#include <driver/adc.h>
#include <ArduinoJson.h>
#include "credentials.h"


#pragma pack(1)
struct Gpsdata
{
  uint8_t trackerid;
  float latitude;
  float longitude;
};
Gpsdata MyGpsdata;

// char data2buf[70];
// char sendBuf[450];
// char bufsd[100] = "";
// char str_lat[15];
// char str_lon[15];
// char str_dist[7];
//char GATEWAY_NAME[16]=  "gtw_esp32_rfm9";
int lastrssi;
int mobid;
static int taskCore = 1;

uint32_t RXpacketCount; //count of received packets
uint8_t RXPacketL;      //length of received packet
int8_t PacketRSSI;      //RSSI of received packet
int8_t PacketSNR;       //signal to noise ratio of received packet
uint8_t PacketType;     //for packet addressing, identifies packet type
uint8_t Destination;    //for packet addressing, identifies the destination (receiving) node
uint8_t Source;         //for packet addressing, identifies the source (transmiting) node
uint8_t TXStatus;       //A status byte
uint32_t TXupTimemS;    //up time of TX in mS
uint32_t errors;        //count of packet errors
SX127XLT LT;

uint8_t *GpsPayload;
long previousMillis = 0;
long interval_batery = 800000; //was 900000
double raw_read;
float volts;

WiFiClient espClient;
PubSubClient client(espClient);
// Led myLed(2);
// NeoPixel neo;
// neoPixelType mNeoPixelType = NEO_GRB + NEO_KHZ800;
float calc_dist(float flat1, float flon1, float flat2, float flon2);
void battery_read();
void local_data();
void check_incomingdata();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void packet_is_Error();
void packet_is_OK();
void printpacketDetails();
void setup_wifi();
void reconnect();

// void coreTask(void *pvParameters)
// {

//   String taskMessage = "Task running on core ";
//   taskMessage = taskMessage + xPortGetCoreID();

//   while (true)
//   {
//     //Serial.println(taskMessage);
//     delay(10);  
//     //check_incomingdata();  
//     LedSync.update();
//   }
// }

void setup()
{
  
  Serial.begin(115200);
  Serial.print("n/n/");
  Serial.println(F("ESP32 LoRa Gateway 10/4/2020 915.600"));
  delay(400);
  SPI.begin();
  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  //if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, SW, LORA_DEVICE))
  {
    
    Serial.println(F("LoRa Device found"));
    delay(200);
    // neo.setColor("#26f041"); // color
    // neo.blink(20 /* on */, 40 /* off */, 2, /* blinks */ 300 /* pause beetween seq*/, 1, NULL);
  }
  else
  {
    Serial.println(F("Device error"));
    while (1)
    {
    // neo.setColor("#f04f26"); // color
    // neo.blink(200 /* on */, 400 /* off */, 3, /* blinks */ 5000 /* pause beetween seq*/, 1, NULL);
    }
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //order is preamble, header type, packet length, CRC, IQ
  LT.setPacketParams(PreAmblelength, LORA_PACKET_FIXED_LENGTH, PacketLength, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.setHighSensitivity();
  // Serial.print(" op settings:  ");
  // LT.printOperatingSettings();
  // Serial.println("\n");
  // Serial.println();
  // Serial.print(" Header Mode:  ");
  // LT.getHeaderMode();
  // Serial.println("\n");

  Serial.println(F("Transmitter ready"));
  Serial.println();
  //Serial.println(F("ESP32 LoRa Gateway 10/4/2020 915.600"));
  GpsPayload = (uint8_t *)(&MyGpsdata);
  //setup_wifi();
  client.setServer(mqtt_server, mqttPort);
 // reconnect();

  // LedSync.setNeoPixelPin(2);
  // Color *red = new Color("#AE7DF1");
  // neo.setColor("#008AD7"); // color
  // neo.setBrightness(50);   // brillo 0 - 255
  // LedSync.setNeoPixelType(mNeoPixelType);
  // LedSync.add(&neo);
  // neo.blink(200 /* on */, 300 /* off */, 2, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
 


}

void loop()
{
  //LedSync.update();
  //uint32_t time_now = millis();
   setup_wifi();
  if (!client.connected())
  {
    Serial.println("..disconnected from MQTT ");
    reconnect();
  }
  client.loop();

  check_incomingdata();
  //delay(10);
  //uint32_t cycletime = time_now - millis();
  //Serial.print("cycletime = ");
  //Serial.println(cycletime);
  //yield();
}

void check_incomingdata()
{
  RXPacketL = LT.receive(GpsPayload, sizeof(MyGpsdata), 10000, WAIT_RX);
  //RXPacketL = LT.receive(GpsPayload, sizeof(MyGpsdata), 100, NO_WAIT);
  PacketRSSI = LT.readPacketRSSI();
  PacketSNR = LT.readPacketSNR();
  if (RXPacketL == 0)
  {
    //packet_is_Error();
  }
  else
  {
    packet_is_OK();
    // neo.setColor("#32a852"); // color
    // neo.blink(20 /* on */, 40 /* off */, 2, /* blinks */ 300 /* pause beetween seq*/, 1, NULL);
  }
}

/*************************************************************************
   //Function to calculate the distance between two waypoints
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat = radians(flat2 - flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(flat1);
  dist_calc2 *= cos(flat2);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

  dist_calc *= 6371000.0; //Converting to meters
  //Serial.println(dist_calc);
  return dist_calc;
}

void packet_is_OK()
{
  RXpacketCount++;
  Serial.print(RXpacketCount);
  Serial.print(F("  "));
  //printlocation1();
  printpacketDetails();
  float distanc = calc_dist(Mylatitude, Mylongitude, MyGpsdata.latitude, MyGpsdata.longitude);
  String output;
  const size_t capacity = JSON_OBJECT_SIZE(8);
  DynamicJsonDocument doc(capacity);  
  doc["lat"] = MyGpsdata.latitude;
  doc["lon"] = MyGpsdata.longitude;
  doc["lastRssi"] = PacketRSSI;
  doc["lastSnr"] = PacketSNR;
  doc["from"] = MyGpsdata.trackerid;
  doc["distance"] = distanc;
  doc["gtw"]= GATEWAY_NAME;
  serializeJson(doc, Serial);
  Serial.println(" ");
  serializeJson(doc, output);
#ifdef mqtton
  if (client.publish(BASE_TOPIC, output.c_str()))
  {
    // neo.setColor("#1300B3"); // color blue
    // neo.setBrightness(70);   // brillo 0 - 255
    // neo.blink(200 /* on */, 100 /* off */, 3, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
    Serial.println("Sending Mqtt..");
  }
  else
  {
    Serial.println("FALLA MQTT");
    // neo.setColor("#870002"); // color  violeta
    // neo.setBrightness(70);   // brillo 0 - 255
    // neo.blink(300 /* on */, 200 /* off */, 6, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
  }
#endif
}

void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();

  if (IRQStatus & IRQ_RX_TIMEOUT)
  {
    Serial.print(F("RXTimeout"));
    Serial.println();
  }
  else
  {
    errors++;
    Serial.print(F("PacketError"));
    printpacketDetails();
    Serial.print(F(" IRQreg,"));
    Serial.print(IRQStatus, HEX);
    Serial.println();
  }
}

void printpacketDetails()
{
  Serial.print(F("  RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm, SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB "));
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect(GATEWAY_NAME, mqttUser, mqttPassword,lastWillTopic,1,false,lastWillMessage,cleanSession))
    {
      Serial.println("connected");
        //char sendbuf [80];      
      //snprintf(sendbuf,sizeof(sendbuf),"{\"name\":%c}",GATEWAY_NAME);
      //client.publish("/v1.0/lora/espgtwy/host_data", "{\"name\":\"LoraSx1262gtw\"}");
     client.publish(BASE_TOPIC, "{\"name\":\"esp32_rfm96_2\"}");  
      
    }
    else
    {

      Serial.print("failed with state ");
      Serial.print(client.state());
      //pixel.setPixelColor(purple);
      //pixel.blink(100, 50, 2, 1000, 1, NULL);
      delay(2000);
    }
  }
}

void setup_wifi()
{

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(10);
    WiFi.mode(WIFI_STA);
    //WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    Serial.print("Attempting to connect to  network, SSID:  ");
    delay(3000);
    Serial.print(ssid);
    Serial.print("Now WiFi is connected, ");
    Serial.print(" with IP address: ");
    Serial.print(WiFi.localIP());
    //wifiStatus = WiFi.status() == WL_CONNECTED;
     Serial.println("");
  }
 
}
