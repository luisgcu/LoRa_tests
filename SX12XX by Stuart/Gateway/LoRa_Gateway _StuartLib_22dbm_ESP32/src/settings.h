/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, a ESP32 shield base with my BBF board shield on
//top. Be sure to change the definitions to match your own setup. Some pins such as DIO2, DIO3, BUZZER
//may not be in used by this sketch so they do not need to be connected and should be included and be 
//set to -1.


#define SCK 18                                  //SCK on SPI3
#define MISO 19                                 //MISO on SPI3 
#define MOSI 23                                 //MOSI on SPI3 
#define NSS 5                                   //select pin on LoRa device
#define NRESET 33                               //reset pin on LoRa device
#define RFBUSY 13                               //busy line
#define LED1 27                                  //on board LED, high for on
#define DIO1 32                                 //DIO1 pin on LoRa device, used for RX and TX done 
#define DIO2 -1                                 //DIO2 pin on LoRa device, normally not used so set to -1 
#define DIO3 -1                                 //DIO3 pin on LoRa device, normally not used so set to -1
#define RX_EN -1                                //pin for RX enable, used on some SX126X devices, set to -1 if not used
#define TX_EN -1                                //pin for TX enable, used on some SX126X devices, set to -1 if not used
#define SW -1                                   //SW pin on Dorji devices is used to turn RF switch on\off, set to -1 if not used   was 21  
#define BUZZER -1                               //pin for buzzer, set to -1 if not used 
#define VCCPOWER -1                             //pin controls power to external devices
#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using
#define SX126XDEBUG  
#define  PreAmblelength 8
#define  PacketLength   9


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency    = 915600000;           //frequency of transmissions
const uint32_t Offset       = 0;                   //offset frequency for calibration purposes
const uint8_t Bandwidth     = LORA_BW_062;         //LoRa bandwidth LORA_BW_041, LORA_BW_062
const uint8_t SpreadingFactor = LORA_SF11;         //LoRa spreading factor
const uint8_t CodeRate      = LORA_CR_4_5;         //LoRa coding rate
const uint8_t Optimisation  = LDRO_AUTO;          //low data rate optimisation setting
const int8_t TXpower        = 15;                 //LoRa TX power
const uint16_t packet_delay = 10000;              //mS delay between packets

#define ThisNode '5'                             //a character that identifies this tracker

//**************************************************************************************************
// GPS Settings
//**************************************************************************************************

//#define USE_SOFTSERIAL_GPS                     //need to include this if we are using softserial for GPS     
#define HardwareSerialPort Serial1               //if using hardware serial enable this define for hardware serial port 
#define gpsbaud 9600                             //GPS Baud rate  
#define WaitGPSFixSeconds 30                     //time in seconds to wait for a new GPS fix 
#define WaitFirstGPSFixSeconds 1800              //time to seconds to wait for the first GPS fix at startup
#define Sleepsecs 5                              //seconds between transmissions, this delay is used to set overall transmission cycle time
#define echomS 2000                              //number of mS to run GPS echo at startup  

#define BUFFER_SIZE 512 // Define the payload size here

#define mqtton                                  //enables Mqtt Send

//**************************************************************************************************
// MQTT TOPICS
//**************************************************************************************************
//#define GATEWAY_NAME "esp32_rfm96_1"   //Placa verde con rtc
//#define GATEWAY_NAME "esp32_rfm96_2"   //Placa verde Con sensor de temperatura
#define GATEWAY_NAME    "esp32_sx1262_1"   //Sx1262 protoboard
#define BASE_TOPIC      "v1.0/lora/espgtwy/nodes"
#define lastWillTopic   "v1.0/lora/espgtwy/will"
#define lastWillMessage "{\"lastwill\":\"unexpected exit\"}"
#define cleanSession  true
#define keepAlive       6000              //5


/*
add  a file named credentials.h and add/edit your senstitive data.

//home adress
const float Mylatitude  = xxxxx; //updated September 4/2020
const float Mylongitude = -xx.xx945;
const char *ssid        = "xxx";
const char *password    = "xxx";
const char *mqtt_server = "xxxx.net"; //noderedsyno.ddns.net
const int mqttPort      = 1983;
const char *mqttUser    = "xxx";
const char *mqttPassword = "xxx";

*/
