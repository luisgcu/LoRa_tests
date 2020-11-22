/***************************************************************************
Esta prueba con 433MHZ Moteinos, Node8 
****************************************************************************/
#include <Arduino.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <LedSync.h>
#include "SPI.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "settings.h"
TinyGPSPlus gps;
SoftwareSerial GPSserial(RXPin, TXPin);
static const RH_RF95::ModemConfig radiosetting = {BW_SETTING << 4 | CR_SETTING << 1 | ImplicitHeaderMode_SETTING, SF_SETTING << 4 | CRC_SETTING << 2, LowDataRateOptimize_SETTING << 3 | ACGAUTO_SETTING << 2};
RH_RF95 driver; //Moteino
//RH_RF95 driver(4,2);   //moteino mega
RHReliableDatagram manager(driver, NODE_ADDRESS);
#pragma pack(1)
struct Gpsdata
{
  float latitude;
  float longitude;
};
Gpsdata MyGpsdata;
uint8_t *GpsPayload;


NeoPixel neo;
neoPixelType mNeoPixelType = NEO_RGB + NEO_KHZ800;

void Gps_sending();
void setup()
{
  Serial.begin(115200); //com 1 en el arduino
#ifdef use_gps
  GPSserial.begin(GPSBaud);
#endif
  if (!manager.init())
    Serial.println(F("init failed"));
  delay(1000);
  Serial.println("");
  Serial.println(F("Moteino  GPS SENDER-433-TO-ESP32-ID= #8,RadioHead Lib, SF=10, Bw=62.5khz --> 10/25/2020-"));
  Serial.println("");
 
  driver.setModemRegisters(&radiosetting);
  driver.setTxPower(txpower, false); //5dbm gave me 1.5km
  driver.setFrequency(frequency);    // hay que setear frecuencia para los rfm96 915
  GpsPayload = (uint8_t *)(&MyGpsdata);
  //driver.printRegisters();
  // Serial.println("lat,   log,   Date,   Time,   Altitude,   Rssi");
  LedSync.setNeoPixelPin(pixelPin);
  Color *red = new Color("#AE7DF1");
  neo.setColor("#008AD7"); // color
  neo.setBrightness(50);   // brillo 0 - 255
  LedSync.setNeoPixelType(mNeoPixelType);
  LedSync.add(&neo);
  neo.blink(100 /* on */, 200 /* off */, 4, /* blinks */ 400 /* pause beetween seq*/, 1, NULL);
}

// uint8_t buf[7];
void loop()
{
  LedSync.update();
#ifdef use_gps
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
            if (valid_less15 > 10)
            {
              Serial.println("enviando...smoke signal I am alive distance change less than 15m..");
              MyGpsdata.latitude = gps.location.lat();
              MyGpsdata.longitude = gps.location.lng();
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
            no_validcount = 0;
            Gps_sending();
          }
          Serial.print(F("Location not valid: "));
          Serial.println(no_validcount);
          neo.setColor("#fc9d03"); // color blue
          neo.setBrightness(80);   // brillo 0 - 255
          neo.blink(20 /* on */, 40 /* off */, 2, /* blinks */ 100 /* pause beetween seq*/, 1, NULL);
        }
      }
    }
#endif
#ifndef use_gps
  MyGpsdata.latitude = 26.320272;
  MyGpsdata.longitude = -80.258136;
  Gps_sending();
  delay(delay_);
#endif
}

void Gps_sending()
{
  unsigned long now1 = millis();
  if (manager.sendtoWait(GpsPayload, sizeof(MyGpsdata), GATEWAY_ADR))
  {
    unsigned long now2 = millis();
    unsigned long timeflight = now2 - now1;
    //  Serial.println(F("Mensaje sent OK "));
    Serial.print(F("delay:= "));
    Serial.print(timeflight);
    Serial.print(", ");
    Serial.print("Last Rssi: ");
    Serial.println(driver.lastRssi(), DEC);
    neo.setColor("#03fcfc"); // color blue
    neo.setBrightness(70);   // brillo 0 - 255
    neo.blink(40 /* on */, 80 /* off */, 2, /* blinks */ 300 /* pause beetween seq*/, 1, NULL);
  }
  else
  {
    Serial.println(F("Mensaje has  failed"));
    // Serial.print(F("Latitude: "));
    neo.setColor("#a103fc"); // color  violeta
    neo.setBrightness(70);   // brillo 0 - 255
    neo.blink(40 /* on */, 80 /* off */, 3, /* blinks */ 500 /* pause beetween seq*/, 1, NULL);
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i = 0; i < loops; i++)
  {
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
  }
}
