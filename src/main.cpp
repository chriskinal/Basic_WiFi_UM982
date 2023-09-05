/*
 * This is a simple GPS code for AgOpen GPS
 * It can used as single antenna with IMU and send PANDA to AgOpen
 * Or use two F9P and send PAOGI to AgOpen
 */
 
#include "NMEAParser.h"
#include <Arduino.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <WiFiUDP.h>

/************************* User Settings *************************/
#define deBugPin   27
bool deBug = false;

// LED settings
#define pwrLED 32
#define imuLED 26
#define ggaLED 25
#define wifLED 33

//Serial Ports
#define SerialGPS Serial1   //1st F9P 10hz GGA,VTG + 1074,1084,1094,1230,4072.0
#define RX1   18
#define TX1   19
#define SerialGPS2 Serial2  //2nd F9P 10hz relPos
#define RX2   16
#define TX2   17
const int32_t baudGPS = 115200;

#define SerialAOG Serial    //AgOpen / USB
const int32_t baudAOG = 115200;

//is the GGA the second sentence?
const bool isLastSentenceGGA = true;

//I2C pins, SDA = 21, SCL = 22
//Note - Pullup resistors will be needed on both SDA & SCL pins
//Reassign i2c pins to make remote IMU hookup easier on 32 pin MCU's
//#define I2C_SDA 21
//#define I2C_SCL 22

//WiFi

byte WiF_ipDest_ending = 255;        //ending of IP address to send UDP data to
unsigned int portMy = 5544;          //this is port of this module
unsigned int AOGNtripPort = 2233;    //port NTRIP data from AOG comes in
unsigned int portDestination = 9999; //Port of AOG that listens
bool WiF_running = false;
char WiF_NTRIP_packetBuffer[512];    // buffer for receiving and sending data
#define YOUR_WIFI_HOSTNAME "AOG_GPS"

IPAddress WiF_ipDestination;
WiFiUDP WiF_udpPAOGI;
WiFiUDP WiF_udpNtrip;

bool GGAReady = false;

//Dual 

byte CK_A = 0, CK_B = 0;
byte incoming_char;
boolean headerReceived = false;
unsigned long ackWait = millis();
byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;

 /* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false, blink;

// Declare functions

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void doWiFUDPNtrip();
void BuildPANDA();
void checksum();
void CalculateChecksum();

// Begin Setup ----------------------------------------------------------------

void setup()
{
    
    // Setup LEDs
    pinMode(pwrLED, OUTPUT);
    pinMode(imuLED, OUTPUT);
    pinMode(ggaLED, OUTPUT);
    pinMode(wifLED, OUTPUT);
    digitalWrite(pwrLED, HIGH);
    digitalWrite(imuLED, LOW);
    digitalWrite(ggaLED, LOW);
    digitalWrite(wifLED, LOW);
    
    // Setup serial ports
    SerialAOG.setRxBufferSize(512);
    SerialAOG.begin(baudAOG);
    SerialGPS.setRxBufferSize(512);
    SerialGPS.begin(baudGPS, SERIAL_8N1, RX1, TX1);
    Serial.println("Started GPS1 *****************");
    SerialGPS2.setRxBufferSize(512);
    SerialGPS2.begin(baudGPS, SERIAL_8N1, RX2, TX2);
    Serial.println("Started GPS2 ******************");
   
    //the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);


//WiFi

  WiFiManager wm;

  // set dark theme
  wm.setClass("invert");

  //set custom ip for portal
  wm.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //set hostname
  wm.setHostname(YOUR_WIFI_HOSTNAME); 

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "Basic_Dual_AP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result

  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  res = wm.autoConnect("Basic_Dual_AP"); // anonymous ap
  // res = wm.autoConnect("Basic_dual_AP","password"); // password protected ap
  
  // Check for WiFi connection
  if (!res) {
    Serial.println("WiFi failed to connect, sending data via USB only.");
    WiF_running = false;
  }
  else {
    
    Serial.println("WiFi connected, sending data via USB & WiFi UDP");
    
        WiF_ipDestination = WiFi.localIP();
        WiF_ipDestination[3] = WiF_ipDest_ending;
        
      WiF_running = true;
      digitalWrite(2, HIGH);
      digitalWrite(wifLED, HIGH);
      Serial.println();
      Serial.print("WiFi IP of roof module: "); Serial.println(WiFi.localIP());
      Serial.print("WiFi sending to IP: "); Serial.println(WiF_ipDestination);
      //init UPD Port sending to AOG
      if (WiF_udpPAOGI.begin(portMy)) // portMy  portDestination
      {
        Serial.print("WiFi UDP sending from port: ");
        Serial.println(portMy);
        Serial.print("WiFi UDP sending to port: ");
        Serial.println(portDestination);
      }
      //init UPD Port getting NTRIP from AOG
      if (WiF_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
      {
        Serial.print("WiFi NTRIP UDP listening to port: ");
        Serial.println(AOGNtripPort);
      }

  }
  
    Serial.println();
    Serial.println("Basic Dual UM982 GPS for AgOpenGPS"); 
    Serial.println("Setup done, waiting for GPS Data....."); 
    if (WiF_running) Serial.println("Sending Data Via WiFi and USB"); 
    else Serial.println("Sending Data Via USB Only"); 
    Serial.println();
    delay(2000);

}

// end of setup ----------------------------------------------------------------------

// Begin main loop -------------------------------------------------------------------

void loop()
{
//delay(1000);
//Read incoming nmea from GPS
if (SerialGPS.available())
    parser << SerialGPS.read();

//Pass NTRIP etc to GPS
if (SerialAOG.available())
    SerialGPS.write(SerialAOG.read());

if (WiF_running) doWiFUDPNtrip();

deBug = !digitalRead(deBugPin);
//deBug = true;

if(GGAReady == true ) {
  BuildPANDA();
  GGAReady = false;
  digitalWrite(imuLED,millis()%512>256);
}
} 

// End main loop

// Checksum calulation **************************************************************************

void checksum() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 70 ; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (CK_A == ackPacket[70] && CK_B == ackPacket[71]) {
    
  if(deBug) Serial.println("ACK Received! ");
    //Serial.println("ACK Received! ");
  }
  else {
  if(deBug) Serial.println("ACK Checksum Failure: ");
  }
}

// Ntrip handling**************************************************************************

void doWiFUDPNtrip() {
  unsigned int packetLenght = WiF_udpNtrip.parsePacket();
  if (packetLenght > 0) {
    WiF_udpNtrip.read(WiF_NTRIP_packetBuffer, packetLenght);
    SerialGPS.write(WiF_NTRIP_packetBuffer, packetLenght);
  }  
} 

// Other hnadlers ********************************************************************
// zHandlers.cpp

//Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

//the new PANDA sentence buffer
char nme[100];

//GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

//VTG
char vtgHeading[12];
char speedKnots[10];

//imu
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];


// if odd characters showed up.
void errorHandler()
{
    //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    if (parser.getArg(0, fixTime)){}

    //latitude
    if (parser.getArg(1, latitude)){}
    if (parser.getArg(2, latNS)){}

    //longitude
    if (parser.getArg(3, longitude)){}
    if (parser.getArg(4, lonEW)){}

    //fix quality
    if (parser.getArg(5, fixQuality)){}

    //satellite #
    if (parser.getArg(6, numSats)){}

    //HDOP
    if (parser.getArg(7, HDOP)){}

    //altitude
    if (parser.getArg(8, altitude)){}

    //time of last DGPS update
    if (parser.getArg(12, ageDGPS)){}

    digitalWrite(ggaLED,millis()%512>256);

   if(deBug) Serial.println("GGA Ready");
   
   if (isLastSentenceGGA){
    BuildPANDA();
    }
}

void VTG_Handler()
{
    //vtg heading
    if (parser.getArg(0, vtgHeading)){}

    //vtg Speed knots
    if (parser.getArg(4, speedKnots)){}

   if(deBug) Serial.println("VTG Ready");
    
   if (!isLastSentenceGGA){
    BuildPANDA(); 
    }
}

void BuildPANDA(void)
{
    strcpy(nme, "");

    strcat(nme, "$PAOGI,");

    strcat(nme, fixTime);
    strcat(nme, ",");

    strcat(nme, latitude);
    strcat(nme, ",");

    strcat(nme, latNS);
    strcat(nme, ",");

    strcat(nme, longitude);
    strcat(nme, ",");

    strcat(nme, lonEW);
    strcat(nme, ",");

    //6
    strcat(nme, fixQuality);
    strcat(nme, ",");

    strcat(nme, numSats);
    strcat(nme, ",");

    strcat(nme, HDOP);
    strcat(nme, ",");

    strcat(nme, altitude);
    strcat(nme, ",");

    //10
    strcat(nme, ageDGPS);
    strcat(nme, ",");

    //11
    strcat(nme, speedKnots);
    strcat(nme, ",");

    //12    
    strcat(nme, imuHeading);
    strcat(nme, ",");

    //13
    strcat(nme, imuRoll);
    strcat(nme, ",");

    //14
    strcat(nme, imuPitch);
    strcat(nme, ",");

    //15
    strcat(nme, imuYawRate);

    strcat(nme, "*");

    CalculateChecksum();

    strcat(nme, "\r\n");

    isTriggered = true;
    
    if (WiF_running) {
        WiF_udpPAOGI.beginPacket(WiF_ipDestination, portDestination);
        WiF_udpPAOGI.print(nme);
        WiF_udpPAOGI.endPacket();
      }

        SerialAOG.print(nme);

}

void CalculateChecksum(void)
{

    int16_t sum = 0, inx;
    char tmp;

    // The checksum calc starts after '$' and ends before '*'
    for (inx = 1; inx < 200; inx++)
    {
        tmp = nme[inx];
        // * Indicates end of data and start of checksum
        if (tmp == '*')
            break;
        sum ^= tmp;    // Build checksum
    }

    byte chk = (sum>>4);
    char hex[2] = {asciiHex[chk],0};
    strcat(nme,hex);
    
    chk = (sum%16);
    char hex2[2] = { asciiHex[chk],0 };
    strcat(nme,hex2);
}

/*
$PANDA
(1) Time of fix

position
(2,3) 4807.038,N Latitude 48 deg 07.038' N
(4,5) 01131.000,E Longitude 11 deg 31.000' E

(6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
(7) Number of satellites being tracked
(8) 0.9 Horizontal dilution of position
(9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
(10) 1.2 time in seconds since last DGPS update
(11) Speed in knots

FROM IMU:
(12) Heading in degrees
(13) Roll angle in degrees(positive roll = right leaning - right down, left up)

(14) Pitch angle in degrees(Positive pitch = nose up)
(15) Yaw Rate in Degrees / second

* CHKSUM
*/


/*
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
     *47          the checksum data, always begins with *
 *
 *
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *
 *
$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
 *
    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
    *48          Checksum
    */