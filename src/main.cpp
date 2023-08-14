/*
 * This is a simple GPS code for AgOpen GPS
 * It can used as single antenna with IMU and send PANDA to AgOpen
 * Or use two F9P and send PAOGI to AgOpen
 */
 
#include "NMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"
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
// bool WiFi_running = false;
bool WiF_running = false;
//char Eth_NTRIP_packetBuffer[512];    // buffer for receiving and sending data
char WiF_NTRIP_packetBuffer[512];    // buffer for receiving and sending data
#define YOUR_WIFI_HOSTNAME "AOG_GPS"

IPAddress WiF_ipDestination;
WiFiUDP WiF_udpPAOGI;
WiFiUDP WiF_udpNtrip;

//Swap BNO08x roll & pitch?
//const bool swapRollPitch = true;
const bool swapRollPitch = false;

//BNO08x, time after last GPS to load up IMU ready data for the next Panda takeoff
const uint16_t IMU_DELAY_TIME = 90; //Best results seem to be 90-95ms
uint32_t IMU_lastTime = IMU_DELAY_TIME;
uint32_t IMU_currentTime = IMU_DELAY_TIME;

//BNO08x, how offen should we get data from IMU (The above will just grab this data without reading IMU)
const uint16_t GYRO_LOOP_TIME = 10;  
uint32_t lastGyroTime = GYRO_LOOP_TIME;

//CMPS14, how long should we wait with GPS before reading data from IMU then takeoff with Panda
const uint16_t CMPS_DELAY_TIME = 4;  //Best results seem to be around 5ms
uint32_t gpsReadyTime = CMPS_DELAY_TIME;

// Booleans to see if we are using CMPS or BNO08x or Dual
bool useCMPS = false;
bool useBNO08x = false;
bool useDual = false;
bool GGAReady = false;
bool relPosReady = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60 

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A,0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual 
double headingcorr = 900;  //90deg heading correction (90deg*10)

float baseline;
double baseline2;
float rollDual;
float rollDualRaw;
double relPosD;
double relPosDH;
double heading = 0;

byte CK_A = 0, CK_B = 0;
byte incoming_char;
boolean headerReceived = false;
unsigned long ackWait = millis();
byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;

 /* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false, blink;

//100hz summing of gyro
float gyro, gyroSum;
float lastHeading;

float roll, rollSum;
float pitch, pitchSum;

float bno08xHeading = 0;
int16_t bno08xHeading10x = 0;

// Declare functions

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void doWiFUDPNtrip();
void imuHandler();
void BuildPANDA();
void GyroHandler(uint32_t delta);
void checksum();
void relPosDecode();
void CalculateChecksum();

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

    //GPS2 Started below

    //the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);

    //Wire.begin(I2C_SDA, I2C_SCL);
    Wire.begin();
    delay(1000);
    pinMode(26, OUTPUT);
    pinMode(deBugPin, INPUT_PULLUP);
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    deBug = !digitalRead(deBugPin);
    //deBug = true;
    Serial.print("deBug Status: ");
    Serial.println(deBug);
    
    //test if CMPS working
    uint8_t error;
    if(deBug) Serial.println("Checking for CMPS14");
    Wire.beginTransmission(CMPS14_ADDRESS);
    error = Wire.endTransmission();

    if (error == 0)
    {
        if(deBug) {
          Serial.println("Error = 0");
          Serial.print("CMPS14 ADDRESs: 0x");
          Serial.println(CMPS14_ADDRESS, HEX);
          Serial.println("CMPS14 Ok.");
        } 
        useCMPS = true;
        digitalWrite(imuLED, HIGH);
    }
    else
    {
        if(deBug) {
          Serial.println("Error = 4");
          Serial.println("CMPS not Connected or Found"); 
        }
    }

    if (!useCMPS)
    {
        for (int16_t i = 0; i < nrBNO08xAdresses; i++)
        {
            bno08xAddress = bno08xAddresses[i];
          if(deBug) {
            Serial.print("\r\nChecking for BNO08X on ");
            Serial.println(bno08xAddress, HEX);
          }
            Wire.beginTransmission(bno08xAddress);
            error = Wire.endTransmission();

            if (error == 0)
            {
              if(deBug) {
                Serial.println("Error = 0");
                Serial.print("BNO08X ADDRESs: 0x");
                Serial.println(bno08xAddress, HEX);
                Serial.println("BNO08X Ok.");
              }
                          // Initialize BNO080 lib        
                if (bno08x.begin(bno08xAddress))
                {
                    Wire.setClock(400000); //Increase I2C data rate to 400kHz

            // Use gameRotationVector
            bno08x.enableGyro(GYRO_LOOP_TIME);
            bno08x.enableGameRotationVector(GYRO_LOOP_TIME-1);
                       
            // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
            if (bno08x.getFeatureResponseAvailable() == true)
            {
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, (GYRO_LOOP_TIME-1)) == false) bno08x.printGetFeatureResponse();
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME-1)) == false) bno08x.printGetFeatureResponse();

              // Break out of loop
              useBNO08x = true;
              digitalWrite(imuLED, HIGH);
              break;
            }
                    else
                    {
                        if(deBug) Serial.println("BNO08x init fails!!");
                    }
                }
                else
                {
                    if(deBug) Serial.println("BNO080 not detected at given I2C address.");
                }
            }
            else
            {
                if(deBug) {
                  Serial.println("Error = 4");
                  Serial.println("BNO08X not Connected or Found"); 
                }    
            }
        }
    }
    
   if (!useCMPS && !useBNO08x)
    {
      SerialGPS2.setRxBufferSize(512);
      SerialGPS2.begin(baudGPS, SERIAL_8N1, RX2, TX2);
      Serial.println("Started GPS2 ******************");
      useDual = true;
    }

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
    Serial.println("Basic Dual or Single GPS for AgOpenGPS"); 
    Serial.println("Setup done, waiting for GPS Data....."); 
    if (WiF_running) Serial.println("Sending Data Via WiFi and USB"); 
    else Serial.println("Sending Data Via USB Only"); 
    Serial.println();
    delay(2000);

} // end of setup ---------------------------------------------

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
    IMU_currentTime = millis();

if(!useDual){
  
  if (useBNO08x)
    {  
      if (isTriggered && IMU_currentTime - IMU_lastTime >= IMU_DELAY_TIME)
      {
        //Load up BNO08x data from gyro loop ready for takeoff
        imuHandler();

        //reset the timer 
        isTriggered = false;
      }      
    }

  if (useCMPS)
    { 
      if (isTriggered && IMU_currentTime - gpsReadyTime >= CMPS_DELAY_TIME)
      {
        imuHandler(); //Get data from CMPS (Heading, Roll, Pitch) and load up ready for takeoff
        BuildPANDA(); //Send Panda

        //reset the timer 
        isTriggered = false;
      }
    }  

  IMU_currentTime = millis();    

  if (IMU_currentTime - lastGyroTime >= GYRO_LOOP_TIME)
    {
        GyroHandler(IMU_currentTime - lastGyroTime);
    }
    
}//End Not Dual

if(!useCMPS && !useBNO08x){
  // if (deBug)
  //   {
  //     Serial.println("Using dual ............");
  //     Serial.print("GGAReady: ");
  //     Serial.println(GGAReady);
  //     Serial.print("relPosReady: ");
  //     Serial.println(relPosReady);
  //   }
if(GGAReady == true && relPosReady == true) {
  BuildPANDA();
  GGAReady = false;
  relPosReady = false;
  digitalWrite(imuLED,millis()%512>256);
}
  
    if (SerialGPS2.available()) {
    //Serial.println("GPS2 data available ...");
    incoming_char = SerialGPS2.read();
    if (i < 4 && incoming_char == ackPacket[i]) {
      i++;
    }
    else if (i > 3) {
      ackPacket[i] = incoming_char;
      i++;
    }
  }
  if (i > 71) {
    //Serial.println("checksum calcinf");
    checksum();
    i = 0;
  }
 } //Dual

} //Loop

//**************************************************************************

void checksum() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 70 ; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (CK_A == ackPacket[70] && CK_B == ackPacket[71]) {
    
  if(deBug) Serial.println("ACK Received! ");
    useDual = true;
    relPosDecode();
    //Serial.println("ACK Received! ");
  }
  else {
  if(deBug) Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

//**************************************************************************

void doWiFUDPNtrip() {
  unsigned int packetLenght = WiF_udpNtrip.parsePacket();
  if (packetLenght > 0) {
    WiF_udpNtrip.read(WiF_NTRIP_packetBuffer, packetLenght);
    SerialGPS.write(WiF_NTRIP_packetBuffer, packetLenght);
  }  
} 

// ************************************************************************
// zGyro.cpp
void GyroHandler(uint32_t delta)
{      
      if (useCMPS)
        {
      //Get the Z gyro
      Wire.beginTransmission(CMPS14_ADDRESS);
      Wire.write(0x16);
      Wire.endTransmission();

      Wire.requestFrom(CMPS14_ADDRESS, 2);
      while (Wire.available() < 2);

      gyro = int16_t(Wire.read() << 8 | Wire.read());

      //Complementary filter
      gyroSum = 0.96 * gyroSum + 0.04 * gyro;
      }          
      
      else if (useBNO08x)
        {
        if (bno08x.dataAvailable() == true)
          {
            gyro = (bno08x.getGyroZ()) * RAD_TO_DEG; // Get raw yaw rate
            gyro = gyro * -10;

            bno08xHeading = (bno08x.getYaw()) * RAD_TO_DEG; // Convert yaw / heading to degrees
            bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

            if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180�;180�] to [0;360�]
            {
                bno08xHeading = bno08xHeading + 360;
            }

            if (swapRollPitch){
            roll = (bno08x.getPitch()) * RAD_TO_DEG;
            pitch = (bno08x.getRoll()) * RAD_TO_DEG;
            }
            else{
            roll = (bno08x.getRoll()) * RAD_TO_DEG;
            pitch = (bno08x.getPitch()) * RAD_TO_DEG;
            pitch = pitch * -1;
            }

            roll = roll * 10;
            pitch = pitch * 10;
            bno08xHeading10x = (int16_t)(bno08xHeading * 10);

            //Complementary filter
            rollSum = roll;
            pitchSum = pitch;
            gyroSum = 0.96 * gyroSum + 0.04 * gyro;
        }
    }

	//save time to check for 10 msec
	lastGyroTime = millis();
}

// ****************************************************************
// zRelPos.cpp
void relPosDecode() {
  

  int carrSoln;
  bool gnssFixOk, diffSoln, relPosValid, isMoving, refPosMiss, refObsMiss ;
  bool refPosHeadingValid, relPosNormalized;
  double roll;

  heading  =  (long)ackPacket[24 + 6] ;
  heading += (long)ackPacket[25 + 6] << 8;
  heading += (long)ackPacket[26 + 6] << 16 ;
  heading += (long)ackPacket[27 + 6] << 24 ;
  heading = heading / 10000;
  heading = heading + headingcorr;
  if (heading >= 3600) heading -= 3600;
  if (heading < 0) heading += 3600;
  heading = heading / 10;
  //Serial.println(heading);

  baseline  =  (long)ackPacket[20 + 6] ;
  baseline += (long)ackPacket[21 + 6] << 8;
  baseline += (long)ackPacket[22 + 6] << 16 ;
  baseline += (long)ackPacket[23 + 6] << 24 ;
  baseline = baseline / 100;
  baseline2 = (long)ackPacket[35 + 6];
  baseline2 =   baseline2 / 10000;
  baseline = baseline + baseline2;

  relPosD  =  (long)ackPacket[16 + 6] ;
  relPosD += (long)ackPacket[17 + 6] << 8;
  relPosD += (long)ackPacket[18 + 6] << 16 ;
  relPosD += (long)ackPacket[19 + 6] << 24 ;
  relPosD = relPosD / 100;
  relPosDH = (long)ackPacket[34 + 6];
  relPosDH = relPosDH / 100000;
  relPosD = relPosD + relPosDH;

  uint32_t flags = ackPacket[60 + 6];

    // Serial.println(flags, BIN);

  gnssFixOk = flags & (1 << 0);
  diffSoln = flags & (1 << 1);
  relPosValid = flags & (1 << 2);
  carrSoln = (flags & (0b11 << 3)) >> 3;
  isMoving = flags & (1 << 5);
  refPosMiss = flags & (1 << 6);
  refObsMiss = flags & (1 << 7);
  refPosHeadingValid = flags & (1 << 8);
  relPosNormalized = flags & (1 << 9);

  // Serial.println(carrSoln);

  if (gnssFixOk && diffSoln && relPosValid)
  {
     if(deBug) Serial.println("Alles OK! ");
  }
  else
  {
     if (deBug)
      {
        Serial.println("Fehler! ");
        Serial.println(gnssFixOk);
        Serial.println(diffSoln);
        Serial.println(relPosValid);
       }
    return;
  }

double p = sqrt(baseline*baseline-relPosD*relPosD);

  if (carrSoln == 2) {
   // roll = (atan2(relPosD, baseline)) * 180 / 3.141592653589793238;
    rollDualRaw = (atan(relPosD/p)) * 180 / 3.141592653589793238;
    rollDualRaw *= -1;
    //rollDual = rollDualRaw * 0.5 + rollDual * 0.5;
    rollDual = rollDualRaw;
      }
  else rollDual = rollDual * 0.9;
 
  imuHandler();
  if (carrSoln == 2){
    relPosReady = true;
    if(deBug) Serial.println("Dual Ready");
  }
  else{
    if(deBug) Serial.println("Dual Accuracy Not Good");
  }
}

// ********************************************************************
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
    if (parser.getArg(0, fixTime));

    //latitude
    if (parser.getArg(1, latitude));
    if (parser.getArg(2, latNS));

    //longitude
    if (parser.getArg(3, longitude));
    if (parser.getArg(4, lonEW));

    //fix quality
    if (parser.getArg(5, fixQuality));

    //satellite #
    if (parser.getArg(6, numSats));

    //HDOP
    if (parser.getArg(7, HDOP));

    //altitude
    if (parser.getArg(8, altitude));

    //time of last DGPS update
    if (parser.getArg(12, ageDGPS));

    // if (blink)
    //     digitalWrite(ggaLED, HIGH);
    // else digitalWrite(ggaLED, LOW);
    // blink = !blink;
    digitalWrite(ggaLED,millis()%512>256);

   if(deBug) Serial.println("GGA Ready");
   
   if (isLastSentenceGGA){
      if (useCMPS){
        gpsReadyTime = millis();
        isTriggered = true;
      }
      else if (useDual) GGAReady = true;
      else BuildPANDA(); 
    }
}

void VTG_Handler()
{
    //vtg heading
    if (parser.getArg(0, vtgHeading));

    //vtg Speed knots
    if (parser.getArg(4, speedKnots));

   if(deBug) Serial.println("VTG Ready");
    
   if (!isLastSentenceGGA){
      if (useCMPS){
        gpsReadyTime = millis();
        isTriggered = true;
      }
      else if (useDual) GGAReady = true;
      else BuildPANDA(); 
    }
}

void imuHandler()
{
    int16_t temp = 0;

    if (useCMPS)
    {
      //roll
      Wire.beginTransmission(CMPS14_ADDRESS);
      Wire.write(0x1C);
      Wire.endTransmission();

      Wire.requestFrom(CMPS14_ADDRESS, 2);
      while (Wire.available() < 2);

      roll = int16_t(Wire.read() << 8 | Wire.read());

      //Complementary filter
      rollSum = roll;
    
      //the heading x10
      Wire.beginTransmission(CMPS14_ADDRESS);
      Wire.write(0x02);
      Wire.endTransmission();

      Wire.requestFrom(CMPS14_ADDRESS, 3);
      while (Wire.available() < 3);

      temp = Wire.read() << 8 | Wire.read();
      itoa(temp, imuHeading, 10);

      //3rd byte pitch
      int8_t pitch = Wire.read();
      itoa(pitch, imuPitch, 10);

      //the roll x10
      temp = (int16_t)rollSum;
      itoa(temp, imuRoll, 10);

      //YawRate
      temp = (int16_t)gyroSum;
      itoa(temp, imuYawRate, 10);

    }
    else if (useBNO08x)
    {
        //Heading
        temp = bno08xHeading10x;
        itoa(temp, imuHeading, 10);

        //the pitch x10
        temp = (int16_t)pitchSum;
        itoa(pitch, imuPitch, 10);

        //the roll x10
        temp = (int16_t)rollSum;
        itoa(temp, imuRoll, 10);

        //YawRate
        temp = (int16_t)gyroSum;
        itoa(temp, imuYawRate, 10);
    }
    else if (useDual)
    {
        //Heading
        dtostrf(heading, 3, 1, imuHeading);

        //the pitch x10
        temp = 0;
        itoa(pitch, imuPitch, 10);

        //the roll x10
        dtostrf(rollDual, 3, 1, imuRoll);

        //YawRate
        temp = 0; 
        itoa(temp, imuYawRate, 10);
        
    }
}

void BuildPANDA(void)
{
    strcpy(nme, "");

    if(!useDual) strcat(nme, "$PANDA,");
    else strcat(nme, "$PAOGI,");

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

    IMU_lastTime = millis();
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