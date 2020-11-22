/*
Configuration paramters for single channel  LoRa gateway with RFM95/96

*/

#define esp32_lora      //Define gateway UP
#define mqtt_on
#define GATEWAY_NAME "esp32_rfm96_433mhz" //My 433MHZ single channel gateway
#define BASE_TOPIC "v1.0/lora/espgtwy/nodes"
#define cleanSession true
#define lastWillTopic "v1.0/lora/espgtwy/will"
#define lastWillMessage "{\"lastwill\":\"unexpected exit\"}"
#define keepAlive 6000 //5
#define GATEWAY_ADDRESS 1
//#define NODE_ADDRESS 2  //MOTEINO TRACE RFM96
//#define NODE_ADDRESS 3  //MOTEINO MEGA RFM96
//#define NODE_ADDRESS 4  //SX1262-22DBM
//#define NODE_ADDRESS 5  //SX1262-30DBM
//#define NODE_ADDRESS 6  //RFm96 Moteino box grey
//#define NODE_ADDRESS 8  //RFm96 moteino 433mhz  marked as green

/******************************************************************************* 
  Radio Settings opctions
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

#define GATEWAY_ADR 1        //Gateway adress
#define BW_SETTING BW62_5Khz // was BW62_5Khz  BW41_7Khz
#define CR_SETTING CR45
#define SF_SETTING SF10
#define CRC_SETTING CRCON
#define ImplicitHeaderMode_SETTING ImplicitHeaderModeON
#define LowDataRateOptimize_SETTING LowDataRateOptimizeON
#define ACGAUTO_SETTING ACGAUTO_ON

char sendBufSD[200];
char data2buf[70];
char sendBuf[250];
char bufsd[100] = "";
char str_lat[15];
char str_lon[15];
char str_dist[7];
int lastrssi;
int mobid;
unsigned long mymillis = 0;
const int mqttPort = 1983;
long previousMillis = 0;
long interval_batery = 800000; //was 900000
double raw_read;
float volts;

/*
//Add a file credential.h and add/ edit  the following variables.
const char *ssid = "xxxx";
const char *password = "xxxxx";
const char *mqtt_server = "xxxx.xx"; //x
const char *mqttUser = "xxx";
const char *mqttPassword = "xx";
//Add your "home latitude /altitude"
const float Mylatitude = 2x.xx0272; 
const float Mylongitude = -xx.xx8136;
*/