
/*******************************************************************************
Configure all the Settings for node 9
********************************************************************************/
#define GATEWAY_ADR 1
#define NODE_ADDRESS 9          //Moteino
#define frequency 433.000000    //Moteino ID 9
//#define frequency 432.998100  //Moteino green ID 8
//#define frequency 914.9950    //Moteino Mega
#define txpower 20
#define use_gps

/******************************************************************************* 
  Radio Settings
*********************************************************************************/ 
//BW
#define BW31_25Khz 0x04
#define BW41_7Khz 0x05
#define BW62_5Khz 0x06
#define BW125Khz 0x07
#define BW250Khz 0x08
#define BW500Khz 0x09
//CR
#define CR45 0x01
#define CR46 0x02
#define CR47 0x03
#define CR48 0x04
//ImplicitHeaderMode
#define ImplicitHeaderModeON 0x00
#define ImplicitHeaderModeOFF 0x01
//SF
#define SF6 0x06
#define SF7 0x07
#define SF8 0x08
#define SF9 0x09
#define SF10 0x0A
#define SF11 0x0B
#define SF12 0x0C
//CRC
#define CRCON 0x01
#define CRCOFF 0x00
//LowDataRateOptimize
#define LowDataRateOptimizeON 0x01
#define LowDataRateOptimizeOFF 0x00
//LowDataRateOptimize
#define ACGAUTO_ON 0x01
#define ACGAUTO_OFF 0x00

#define BW_SETTING BW62_5Khz // was BW62_5Khz  BW41_7Khz
#define CR_SETTING CR45
#define SF_SETTING SF10
#define CRC_SETTING CRCON
#define ImplicitHeaderMode_SETTING ImplicitHeaderModeON
#define LowDataRateOptimize_SETTING LowDataRateOptimizeON
#define ACGAUTO_SETTING ACGAUTO_ON
//********************************


const int pixelPin              = 9;      // Arduino pin where the NeoPixel LED (marcado como D7 en la placa )
const int pixels                = 1;      // Amount of NeoPixel LEDs connected
const int buzzerpin             = 6;      // buzzer pin es d6
static const int RXPin          = 3;
static const int TXPin          = 4;      // Marcado como  D3
static const uint16_t GPSBaud   = 9600;
const long interval             = 10000;  // interval at which to blink (milliseconds)
const int delay_                = 6000;
int ledState                    = 0;
int replicar1                   = 0;
float oldlat1                   = 0;
float oldlong1                  = 0;
int16_t no_validcount           = 0;
uint16_t valid_less15           = 0;
const uint8_t no_valid_to_send  = 4;