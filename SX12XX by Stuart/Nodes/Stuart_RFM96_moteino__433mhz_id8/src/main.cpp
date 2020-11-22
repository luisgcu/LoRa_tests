//GPS SIDE NODE #5 Angel Updated Oct/14/2020
//#include <Arduino.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <ProgramLT_Definitions.h>
// #include <MyBlinker.h>

#ifdef USE_SOFTSERIAL_GPS
#include <SoftwareSerial.h>
SoftwareSerial GPSserial(RXpin, TXpin);
#else
#define GPSserial HardwareSerialPort //hardware serial port (eg Serial1) is configured in the Settings.h file
#endif

//#define testing                        //para probar
static const int RXPin = 3, TXPin = 4; // Marcado como  D3
static const uint32_t GPSBaud = 9600;
const int pixelPin = 9;                  // Arduino pin where the NeoPixel LED (marcado como D7 en la placa )
const int pixels = 1;                    // Amount of NeoPixel LEDs connected
//***********************************************************************************
// Pixel Led
//***********************************************************************************
// PixelBlinker pixel(pixelPin, pixels);
// PixelColor red(255, 0, 0);              // Red color
// PixelColor red1(229, 61, 120);          // Red1
// PixelColor red2(237, 180, 23);          // Red2

// PixelColor blue(0, 0, 255);            // Blue color
// PixelColor blue1(48, 109, 209);        // Blue color
// PixelColor blue2(129, 215, 239);       // Blue color

// PixelColor green(0, 255, 0);           // Green color
// PixelColor green1(23, 237, 112);       // Green color
// PixelColor green2(69, 209, 48);        // Green color

// PixelColor purple(230, 129, 239);      // Purple color
uint8_t TXStatus = 0;                  //used to store current status flag bits of Tracker transmitter (TX)
uint8_t TXPacketL;                     //length of LoRa packet (TX)
float TXLat;                           //Latitude from GPS on Tracker transmitter (TX)
float TXLon;                           //Longitude from GPS on Tracker transmitter (TX)
float TXAlt;                           //Altitude from GPS on Tracker transmitter (TX)
uint8_t TXSats;                        //number of GPS satellites seen (TX)
uint32_t TXHdop;                       //HDOP from GPS on Tracker transmitter (TX)
uint16_t TXVolts;                      //Volts (battery) level on Tracker transmitter (TX)
uint32_t TXGPSFixTime;                 //GPS fix time in hot fix mode of GPS on Tracker transmitter (TX)
uint32_t TXPacketCount, TXErrorsCount; //keep count of OK packets and send errors
SX127XLT LT;
TinyGPSPlus gps;
unsigned long previousMillis = 0; // will store last time LED was updated
const long interval = 10000; // interval at which to blink (milliseconds)
const int SendInterval = 10; //Check every 15 seconds if position had changed
int SendIntervalCount = 0;
int ledState = HIGH;
int replicar1 = 0;
uint32_t TXpacketCount = 1;
uint32_t startmS, endmS;
float oldlat1 = 0;
float oldlong1 = 0;
#pragma pack(1)
struct Gpsdata // estructura de datos para enviar latitud y longitud
{
  uint8_t trackerid;
  float latitude;
  float longitude;
};
Gpsdata MyGpsdata;
uint8_t *GpsPayload;
void Gps_sending();
void led_Flash(uint16_t flashes, uint16_t delaymS);
void Blink(byte PIN, byte DELAY_MS, byte loops);

void setup()
{
  Serial.begin(115200); //com 1 en el arduino
  GPSserial.begin(gpsbaud);
  //Serial1.begin(gpsbaud, SERIAL_8N1, RXD1, TXD1);
  Serial.println(F("GPS SENDER-915-ID #3 Moteino Mega Stuart-->updated Sept29 2020"));
  //Serial.println(F("Frecuency Adjust ID#2=915600000, Frecuency Gateway =915600000"));
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW); //setup pin as output for indicator LED
  digitalWrite(LED2, LOW); //setup pin as output for indicator LED
  
  GpsPayload = (uint8_t *)(&MyGpsdata);
  SPI.begin();
  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    // pixel.setPixelColor(green);
    // pixel.blink(30, 50, 2, 200, 1, NULL); 
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("Device error"));
    while (1)
    {
    // pixel.setPixelColor(red);
    // pixel.blink(30, 50, 4, 700, 1, NULL); 
    }
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);
   //order is preamble, header type, packet length, CRC, IQ
  LT.setPacketParams(PreAmblelength, LORA_PACKET_FIXED_LENGTH, PacketLength, LORA_CRC_ON, LORA_IQ_NORMAL);
  // LT.setHighSensitivity();
  // Serial.print(" op settings:  ");
  // LT.printOperatingSettings();
  // Serial.println("\n");
  // Serial.println();
  // Serial.print(" Header Mode:  ");
  // LT.getHeaderMode();
  // Serial.println("\n");
  Serial.println(F("Transmitter ready"));
  Serial.println();
  MyGpsdata.latitude = 26.320272;
  MyGpsdata.longitude = -80.258136;
  MyGpsdata.trackerid =NODE_ADDRESS;
  Gps_sending();
}

void loop() 
{
  //pixel.update();
// while (Serial1.available() ){

// Serial.write(Serial1.read());

// }
  while (GPSserial.available() > 0)
    if (gps.encode(GPSserial.read()))
    {
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= interval)
      {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        if (gps.location.isValid())
        {
          double lat1 = gps.location.lat();
          double lon1 = gps.location.lng();
          double distance = gps.distanceBetween(lat1, lon1, oldlat1, oldlong1);
          MyGpsdata.latitude = gps.location.lat();
          MyGpsdata.longitude = gps.location.lng();
          MyGpsdata.trackerid = NODE_ADDRESS;
          if (distance >= 15)
          { // if distance from previous point grearter than 15 meters send the position
            oldlat1 = lat1;
            oldlong1 = lon1;
            Serial.println("enviando... distance changed > +15m..");
            Gps_sending(); // used on 2020
          }
          if (distance < 15)
          {
            valid_less15 = valid_less15 + 1;
            if (valid_less15 > 4)
            {
              Serial.println("enviando...smoke signal I am alive distance change less than 15m..");
              MyGpsdata.latitude = gps.location.lat();
              MyGpsdata.longitude = gps.location.lng();
              MyGpsdata.trackerid = NODE_ADDRESS;
              Gps_sending();
              // pixel.setPixelColor(blue);
              // pixel.blink(30, 50, 3, 200, 1, NULL); 
              valid_less15 = 0;
            }
          }
        }
        else
        { ////no gps fix red  led blink and count

          no_validcount = no_validcount + 1;
          if (no_validcount > no_valid_to_send)
          {
            MyGpsdata.latitude = 26.110272;
            MyGpsdata.longitude = -80.358136;
            MyGpsdata.trackerid = NODE_ADDRESS;
            no_validcount = 0;
            Gps_sending();
          }
          Serial.print(F("Location not valid: "));
          Serial.println(no_validcount);
          // pixel.setPixelColor(red);
          // pixel.blink(30, 50, 2, 100, 1, NULL); 

#ifdef testing
          MyGpsdata.latitude = 26.320272;
          MyGpsdata.longitude = -80.258136;
          Gps_sending();
#endif
        }
      }
    }
}

void Gps_sending()
{
#ifdef testing
  MyGpsdata.trackerid = NODE_ADDRESS;
  MyGpsdata.latitude = 26.120272;
  MyGpsdata.longitude = -80.358136;
#endif
  startmS = millis();
  if (LT.transmit(GpsPayload, sizeof(MyGpsdata), 0, TXpower, WAIT_TX))

  {
    endmS = millis();
    TXpacketCount++;
    Serial.print("Packet ");
    Serial.print(TXpacketCount);
    Serial.print(F(", "));
    Serial.print(sizeof(MyGpsdata));
    Serial.print(F(" Bytes Sent:"));
    Serial.print(F(" "));
    Serial.print(endmS - startmS);
    Serial.print(F("mS"));
    Blink(LED1, 30, 2); //Led1=26, LED2=27
  }
  else
  {
    Serial.println(F("Mensaje has  failed"));
    Serial.print(F("Send Error - IRQreg,"));
    Serial.print(LT.readIrqStatus(), HEX);
    Blink(LED2, 260, 5);
  }
  Serial.println();
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i = 0; i < loops; i++)
  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}

// void beep(byte PIN, byte delayms, byte loops, byte tonee)
// {
//   for (byte i = 0; i < loops; i++)
//   {
//     analogWrite(PIN, tonee); // Almost any value can be used except 0 and 255
//                              // experiment to get the best tone
//     delay(delayms);          // wait for a delayms ms
//     analogWrite(PIN, 0);     // 0 turns it off
//     delay(delayms);          // wait for a delayms ms
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