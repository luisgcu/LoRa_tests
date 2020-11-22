/*
This a LoRa gatwusing RADIOHEAD library 
*/

#include <Arduino.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LedSync.h>
#include <driver/adc.h>
#include <ArduinoJson.h>
#include "credentials.h"
#include "config.h"


#ifdef esp32_lora
RH_RF95 driver(5, 15); //ESP32---> (RFM95_CS, RFM95_INT)    nss,DIo0   (TTGO)
#endif
#ifdef moteino
RH_RF95 driver; //moteino
#endif
#ifdef moteino_mega
RH_RF95 driver(4, 2); //moteino mega  NSS(SS)=4, DIO0=D2
#endif
#ifdef esp8266_lora
RH_RF95 driver(2, 15); //ESP8266---> (RFM95_CS, RFM95_INT)
#endif

static const RH_RF95::ModemConfig radiosetting = {BW_SETTING << 4 | CR_SETTING << 1 | ImplicitHeaderMode_SETTING, SF_SETTING << 4 | CRC_SETTING << 2, LowDataRateOptimize_SETTING << 3 | ACGAUTO_SETTING << 2};
RHReliableDatagram manager(driver, GATEWAY_ADDRESS);   //I am the gateway

//data struct that hold the data from the cliente via Rfm96
#pragma pack(1)
struct Gpsdata
{
  //uint8_t trackerid;
  float latitude;
  float longitude;
};
Gpsdata MyGpsdata;



uint8_t *GpsPayload;


WiFiClient espClient;
PubSubClient client(espClient);
NeoPixel neo;
neoPixelType mNeoPixelType = NEO_GRB + NEO_KHZ800;
float calc_dist(float flat1, float flon1, float flat2, float flon2);
void battery_read();
void local_data();
void check_incomingdata();
void setup_wifi();
void reconnect();

void setup()
{
  Serial.begin(115200);

  if (!manager.init())
  {
    delay(100);
    Serial.println("init failed");
  }
  Serial.println("");
  Serial.println(F("ESP32 LoRa Gateway RadioHead Lib, SF=10, Bw=62.5khz, 10/11/2020 433.000"));
  Serial.println("");
  driver.setModemRegisters(&radiosetting);
  driver.setTxPower(20, false); 
  driver.setFrequency(433.0000);
  GpsPayload = (uint8_t *)(&MyGpsdata);
  client.setServer(mqtt_server, mqttPort);  
  LedSync.setNeoPixelPin(2);
  Color *red = new Color("#AE7DF1");
  neo.setColor("#008AD7"); // color
  neo.setBrightness(50);   // brillo 0 - 255
  LedSync.setNeoPixelType(mNeoPixelType);
  LedSync.add(&neo);
  neo.blink(300 /* on */, 200 /* off */, 5, /* blinks */ 1000 /* pause beetween seq*/, 1, NULL);
}

void loop()
{
  LedSync.update();
  setup_wifi();
  if (!client.connected())
  {
    Serial.println("..disconnected from MQTT ");
    reconnect();
  }
  check_incomingdata();
  client.loop();
}

void check_incomingdata()
{
  if (manager.available())
  {
    uint8_t len = sizeof(MyGpsdata);
    uint8_t from;
    if (manager.recvfromAck(GpsPayload, &len, &from))
    {
      //Serial.println("got something");
      lastrssi = driver.lastRssi();
      int8_t lastsnr = driver.lastSNR();
      int offset = driver.frequencyError();
      float distanc = calc_dist(Mylatitude, Mylongitude, MyGpsdata.latitude, MyGpsdata.longitude);

      String output;
      const size_t capacity = JSON_OBJECT_SIZE(9);
      DynamicJsonDocument doc(capacity);
      doc["lat"] = MyGpsdata.latitude;
      doc["lon"] = MyGpsdata.longitude;
      doc["lastRssi"] = lastrssi;
      doc["lastSnr"] = lastsnr;
      doc["from"] = from;
      doc["distance"] = distanc;
      doc["offset"] = offset;
      doc["gtw"] = GATEWAY_NAME;
      serializeJson(doc, Serial);
      Serial.println(" ");
      serializeJson(doc, output);

#ifdef mqtt_on
      if (client.publish(BASE_TOPIC, output.c_str()))
      {
        neo.setColor("#1300B3"); // color blue
        neo.setBrightness(70);   // brillo 0 - 255
        neo.blink(40 /* on */, 80 /* off */, 2, /* blinks */ 300 /* pause beetween seq*/, 1, NULL);
      }
      else
      {
        Serial.println("FALLA MQTT");
        neo.setColor("#870002"); // color  violeta
        neo.setBrightness(70);   // brillo 0 - 255
        neo.blink(40 /* on */, 80 /* off */, 3, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
      }
#endif
    }
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

void reconnect()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");

    if (client.connect(GATEWAY_NAME, mqttUser, mqttPassword, lastWillTopic, 1, false, lastWillMessage, cleanSession))
    {
      Serial.println("connected");

      client.publish(BASE_TOPIC, "{\"name\":\"esp32_rfm96_433mhz\"}");
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
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    Serial.print("Attempting to connect to  network, SSID:  ");
    delay(3000);
    Serial.print(ssid);
    Serial.print(", Now WiFi is connected, ");
    Serial.print(" with IP address: ");
    Serial.print(WiFi.localIP());
    //wifiStatus = WiFi.status() == WL_CONNECTED;
    Serial.println("");
  }
}
