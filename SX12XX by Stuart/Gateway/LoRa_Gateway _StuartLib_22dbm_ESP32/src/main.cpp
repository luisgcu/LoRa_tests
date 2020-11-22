#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SX126XLT.h>
#include <ProgramLT_Definitions.h>
#include "Settings.h"
#include <LedSync.h>
#include <driver/adc.h>
#include <ArduinoJson.h>
#include "credentials.h"


//data struct that hold the data from the cliente via Rfm96
#pragma pack(1)
struct Gpsdata
{
  uint8_t trackerid;
  float latitude;
  float longitude;
};
Gpsdata MyGpsdata;

// char sendBufSD[200];
// char data2buf[70];
// char sendBuf[450];
// char bufsd[100] = "";
// char str_lat[15];
// char str_lon[15];
// char str_dist[7];
int lastrssi;
int mobid;

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
SX126XLT LT;

uint8_t *GpsPayload;


long previousMillis = 0;
long interval_batery = 800000; //was 900000
double raw_read;
float volts;

WiFiClient espClient;
PubSubClient client(espClient);
// Led myLed(2);
NeoPixel neo;
neoPixelType mNeoPixelType = NEO_GRB + NEO_KHZ800;
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

void setup()
{
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH); //setup pin as output for indicator LED
  led_Flash(2, 125);
  SPI.begin();
  //if (LT.begin(NSS, NRESET, RFBUSY, DIO1, SW, LORA_DEVICE))
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, SW, LORA_DEVICE))
  {
    led_Flash(2, 125);
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("Device error"));
    while (1)
    {
      led_Flash(50, 50); //long fast speed flash indicates device error
    }
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //order is preamble, header type, packet length, CRC, IQ
  LT.setPacketParams(PreAmblelength, LORA_PACKET_FIXED_LENGTH, PacketLength, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.setHighSensitivity();
  Serial.print(" op settings:  ");
  LT.printOperatingSettings();
  Serial.println("\n");
  Serial.println();
  Serial.print(" Header Mode:  ");
  LT.getHeaderMode();
  Serial.println("\n");

  Serial.println(F("Transmitter ready"));
  Serial.println();
  Serial.println(F("ESP32 LoRa Gateway 03/23/30 915.009"));
  GpsPayload = (uint8_t *)(&MyGpsdata);
  //setup_wifi();
  client.setServer(mqtt_server, mqttPort);
  //reconnect();

  //analogReadResolution(11);  // voltage a medir 2vols , resistores de 100k y 91 k
  //analogSetAttenuation(ADC_6db);   //ADC_ATTEN_2_5db
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db); //set reference voltage to internal
  LedSync.setNeoPixelPin(2);
  // Color *red = new Color("#AE7DF1");
  neo.setColor("#008AD7"); // color
  neo.setBrightness(50);   // brillo 0 - 255
  LedSync.setNeoPixelType(mNeoPixelType);
  LedSync.add(&neo);
  neo.blink(300 /* on */, 200 /* off */, 5, /* blinks */ 1000 /* pause beetween seq*/, 1, NULL);
}

void loop()
{
  uint32_t time_now = millis();
  setup_wifi();
  if (!client.connected())
  {
    Serial.println("..disconnected from MQTT ");
    reconnect();
  }
  client.loop();
  // LedSync.update();
  //local_data();
  check_incomingdata();
  //delay(50);
  // uint32_t cycletime= time_now - millis();
  // Serial.print("cycletime = "); Serial.println(cycletime);
  //yield();
}

// void local_data()
// {

//   unsigned long currentMillis = millis();
//   if (currentMillis - previousMillis > interval_batery)
//   {
//     int bat_percent;
//     battery_read();

//     bat_percent = map(raw_read, 1500, 2179, 0, 100);
//     //Serial.print("Battery level %: "); Serial.print(bat_percent); Serial.println("%");
//     Serial.println();
//     //Serial.print("Battery Voltage = "); Serial.print(volts-0.06, 2); Serial.println(" V");
//     snprintf(data2buf, sizeof(data2buf), "{\"Gateway_Battery_volt\":%.2f,\"Battery %\":%d}", volts - 0.06, bat_percent); //susbtract 0.06 because wifi
//     if (client.publish("/v1.0/lora/espgtwy/host_data", data2buf))
//     {
//       // neo.setColor("#1300B3"); // color blue
//       // neo.setBrightness(70);   // brillo 0 - 255
//       // neo.blink(200 /* on */, 100 /* off */, 3, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
//     }
//     else
//     {
//       Serial.println("FALLA MQTT");
//       // neo.setColor("#870002"); // color  violeta
//       // neo.setBrightness(70);   // brillo 0 - 255
//       // neo.blink(300 /* on */, 200 /* off */, 6, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
//     }
//     previousMillis = currentMillis;
//   }
// }

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

// void battery_read() //for some reason when wifi is ON it affect the alalog reading and add 0.06 volts
// {
//   //read battery voltage per %
//   long sum = 0; // sum of samples taken
//   for (int i = 0; i < 250; i++)
//   {
//     sum += analogRead(35);
//     delayMicroseconds(1000);
//   }
//   // calculate the voltage
//   raw_read = sum / (double)250;
//   if (raw_read > 1 || raw_read < 4096)
//   {
//     // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
//     volts = 2.267 * (-0.000000000000016 * pow(raw_read, 4) + 0.000000000118171 * pow(raw_read, 3) - 0.000000301211691 * pow(raw_read, 2) + 0.001109019271794 * raw_read + 0.034143524634089);
//     volts = roundf(volts * 100) / 100;
//     //Serial.print("voltage: ");
//     //Serial.println(volts, 3);
//   }
//   else
//   {
//     volts = 0;
//   }
// }

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, LOW);
    delay(delaymS);
    digitalWrite(LED1, HIGH);
    delay(delaymS);
  }
}

void packet_is_OK()
{
  RXpacketCount++;
  Serial.print(RXpacketCount);
  Serial.print(F("  "));
  //printlocation1();
  printpacketDetails();

  float distanc = calc_dist(Mylatitude, Mylongitude, MyGpsdata.latitude, MyGpsdata.longitude);
  // Serials print en programas anteriores.
  // mobid = MyGpsdata.trackerid;
  // dtostrf(distanc, 6, 1, str_dist);
  // dtostrf(MyGpsdata.latitude, 9, 6, str_lat);
  // dtostrf(MyGpsdata.longitude, 9, 6, str_lon);
  // snprintf(sendBuf, sizeof(sendBuf), "{\"lat\":%s, \"lon\":%s, \"lastRssi\":%d,\"lastSnr\":%d, \"from\":%d,\"distance\":%s}", str_lat, str_lon, PacketRSSI, PacketSNR, mobid, str_dist);
  // //Serial.println(sendBuf);
  String output;
  const size_t capacity = JSON_OBJECT_SIZE(9);
  DynamicJsonDocument doc(capacity);
  doc["lat"] = MyGpsdata.latitude;
  doc["lon"] = MyGpsdata.longitude;
  doc["lastRssi"] = PacketRSSI;
  doc["lastSnr"] = PacketSNR;
  doc["from"] = MyGpsdata.trackerid;
  doc["distance"] = distanc;
  doc["gtw"] = GATEWAY_NAME;
  serializeJson(doc, Serial);
  Serial.println(" ");
  serializeJson(doc, output);
  if (client.publish(BASE_TOPIC, output.c_str()))
  //if (client.publish("/v1.0/lora/espgtwy/host_data", "{\"name\":\"LoraSx1262gtw\"}"))
  {
    // neo.setColor("#1300B3"); // color blue
    // neo.setBrightness(70);   // brillo 0 - 255
    // neo.blink(200 /* on */, 100 /* off */, 3, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
    Serial.println("Sending Mqtt..");
    led_Flash(3, 80);
  }
  else
  {
    Serial.println("FALLA MQTT");
    // neo.setColor("#870002"); // color  violeta
    // neo.setBrightness(70);   // brillo 0 - 255
    // neo.blink(300 /* on */, 200 /* off */, 6, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
  }
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
      //client.publish("/v1.0/lora/espgtwy/host_data", "{\"name\":\"LoraSx1262gtw\"}");
      client.publish(BASE_TOPIC, "{\"name\":\"esp32_sx1262_1\"}"); 
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
    Serial.print("Attempting to connect to  network, SSID: ");
    delay(5000);
    Serial.println(ssid);
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //wifiStatus = WiFi.status() == WL_CONNECTED;
    Serial.println("");
  }
 
}
