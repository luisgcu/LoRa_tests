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

//Moteino
#define SCK 13                                //SCK on SPI3
#define MISO 12                                 //MISO on SPI3 
#define MOSI 11                                 //MOSI on SPI3 
#define NSS 10                                  //select pin on LoRa device
#define DIO0 2
#define NRESET -1                              //reset pin on LoRa device
#define RFBUSY -1                               //busy line
#define LED1 -1                                 //Valid Ok, Send ok, high for on
#define LED2 -1                                 //No valid , Send bad, high for on
#define DIO1 21                                 //DIO1 pin on LoRa device, used for RX and TX done 
#define SW -1                                   //SW pin on Dorji devices is used to turn RF switch on\off, set to -1 if not used    
#define BUZZER -1                               //pin for buzzer, set to -1 if not used 
#define VCCPOWER -1                             //pin controls power to external devices
#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using
#define DIO0 2
#define SX126XDEBUG  
#define PreAmblelength 8
#define PacketLength   9


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
//const uint32_t Frequency    = 915614000;           //frequency of transmissions
//const uint32_t Frequency    = 915599500;   //-55
const uint32_t Frequency    = 433601000;     //-59
//const uint32_t Frequency    = 915598000;     //-59
const uint32_t Offset       = 0;                      //offset frequency for calibration purposes
const uint8_t Bandwidth     = LORA_BW_062;          //LoRa bandwidth LORA_BW_041, LORA_BW_062
const uint8_t SpreadingFactor = LORA_SF11;       //LoRa spreading factor
const uint8_t CodeRate      = LORA_CR_4_5;            //LoRa coding rate
const uint8_t Optimisation  = LDRO_AUTO;          //low data rate optimisation setting
const int8_t TXpower        = 20;                       //LoRa TX power
const uint16_t packet_delay = 10000;             //mS delay between packets

//#define ThisNode '5'                             //a character that identifies this tracker

//**************************************************************************************************
// GPS Settings
//**************************************************************************************************

#define USE_SOFTSERIAL_GPS                     //need to include this if we are using softserial for GPS     
//#define HardwareSerialPort Serial1               //if using hardware serial enable this define for hardware serial port 
#define gpsbaud 9600                             //GPS Baud rate  
#define WaitGPSFixSeconds 30                     //time in seconds to wait for a new GPS fix 
#define WaitFirstGPSFixSeconds 1800              //time to seconds to wait for the first GPS fix at startup
#define Sleepsecs 5                              //seconds between transmissions, this delay is used to set overall transmission cycle time
#define echomS 2000                              //number of mS to run GPS echo at startup 
#define RXpin 3
#define TXpin 4
#define RXD1 15
#define TXD1 2
#define GATEWAY_ADDRESS 1  
//#define NODE_ADDRESS 2  //MOTEINO TRACE RFM96
// #define NODE_ADDRESS 3  //MOTEINO MEGA RFM96
// #define NODE_ADDRESS 4  //SX1262-22DBM
// #define NODE_ADDRESS 5  //SX1262-30DBM
//#define NODE_ADDRESS 6  //RFm96 Moteino box grey
#define NODE_ADDRESS 8  //RFm96 moteino 433mhz



//**************************************************************************************************
// Variables settings
//**************************************************************************************************

uint16_t no_validcount = 0;
uint16_t valid_less15  = 0;
const uint8_t no_valid_to_send= 4;

