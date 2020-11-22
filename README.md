# LoRa _tests  (Nodes with GPS & ESP32 Gateways)
Many LoRa test employing different libraries using RFM95/96 and SX1262.

Lora test include Sample code for GPS nodes and single channel gateway .

Single channel gateway need to create a file named credentials.h  into the src directory, edit  that file and put your data as follow:

```
/*
create  a file named credentials.h and add/edit your senstitive data.
//home adress
const float Mylatitude  = xxxxx;       //your "home or base" latitude
const float Mylongitude = -xx.xxxxx;   //your "home or base" llongitude
const char *ssid        = "xxx";       // Wifi ssid
const char *password    = "xxx";       // your wifi password
const char *mqtt_server = "xxxx.net";   //your mqtt server
const int mqttPort      = 1983;         //mqtt port
const char *mqttUser    = "xxx";        //mqtt user
const char *mqttPassword = "xxx";       //mqtt password
*/
```

