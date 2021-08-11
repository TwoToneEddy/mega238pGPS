/*
ubx wiring:

Green = GPS Rx  -> Ard Tx
Blue  = GPS Tx  -> Ard Rx

ubx commands:

Sleep:
B5 62 06 57 08 00 01 00 00 00 50 4F 54 53 AC 85

Wake:
B5 62 06 57 08 00 01 00 00 00 20 4E 55 52 7B C3

Disable NMEA messages:

B5 62 06 01 03 00 F0 00 00 FA 0F
B5 62 06 01 03 00 F0 01 00 FB 11
B5 62 06 01 03 00 F0 02 00 FC 13
B5 62 06 01 03 00 F0 03 00 FD 15
B5 62 06 01 03 00 F0 04 00 FE 17
B5 62 06 01 03 00 F0 05 00 FF 19

Poll POSLLH:
B5 62 01 02 00 00 03 0A

*/

#include <Arduino.h>

#include <SoftwareSerial.h>
#include "LowPower.h"

#define LOOP_DELAY  1000
//#define GPS_LOCK_MSG_LIMIT  180  
#define GPS_LOCK_MSG_LIMIT  20  
#define HORIZONTAL_ACC_THRESHOLD  10
#define STAY_AWAKE_CYCLE 60


// Connect the GPS RX/TX to arduino pins 3 and 5
SoftwareSerial gpsPort = SoftwareSerial(8,9);
SoftwareSerial sim800Port = SoftwareSerial(6,7);

const unsigned char gpsConfigCommands[6][11]= {{0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F},
                                               {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11},
                                               {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13},
                                               {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15},
                                               {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17},
                                               {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19}};
const unsigned char gpsSleepCommand[] = {0xB5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x4F, 0x54, 0x53, 0xAC, 0x85};
const unsigned char gpsWakeCommand[] = {0xB5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x4E, 0x55, 0x52, 0x7B, 0xC3};
const unsigned char gpsPOSLLH_CMD[]={0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A};
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
int gpsConfigured,gpsMessageCounter,gpsLock,gpsPositionRequest,gpsAwake,gpsAwakeCounter,gpsCommsErrorCounter,gpsPortActive;
int sim800Configured,sim800Comms,sim800PortActive,newestMsgIndex;
bool useGSM,GSMPassthrough;
String buffer;
String locationMessage;
long lat_con,lon_con;
float lat_float,lon_float;
char bufferArray[64];
char googlePrefix[]="http://maps.google.com/?q=";
char catInt[4];

struct MSG_CONTENTS{
  String senderNumber;
  String message;
};

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;
MSG_CONTENTS sms;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( gpsPort.available() ) {
    byte c = gpsPort.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] ){
        fpos++;
      }
      else{
        fpos = 0;
      }
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

void gpsWake(){
  Serial.println("GPS waking up!");
  gpsPort.write(gpsWakeCommand,sizeof(gpsWakeCommand));
  gpsAwake = 1;
  gpsAwakeCounter = 0;
  delay(500);
  gpsPort.flush();
  delay(100);
}

void gpsSleep(){
  Serial.println("GPS going to sleep...");
  gpsPort.write(gpsSleepCommand,sizeof(gpsSleepCommand));
  gpsAwake = 0;
  gpsAwakeCounter = 0;
  gpsLock = 0;
  delay(500);
  gpsPort.flush();
  delay(100);
}


// Write all configure commands
void gpsConfigure(){
  delay(500);
  for(int i = 0; i < sizeof(gpsConfigCommands)/sizeof(gpsConfigCommands[0]); i++)
    gpsPort.write(gpsConfigCommands[i],sizeof(gpsConfigCommands[i]));
  delay(500);
  gpsSleep();
  gpsPort.flush();
  gpsConfigured = 1;
}

void activateGpsPort(){

  if(sim800PortActive){
    sim800Port.end();
    delay(100);
    sim800PortActive = 0;
  }

  if(gpsPortActive){
    return;
  }else{
    gpsPort.begin(9600);
    delay(200);
    gpsPort.flush();
    gpsPortActive = 1;
  }
  delay(500);
}

void activateSim800Port(){

  if(gpsPortActive){
    gpsPort.end();
    delay(100);
    gpsPortActive = 0;
  }

  if(sim800PortActive){
    return;
  }else{
    sim800Port.begin(9600);
    delay(200);
    sim800Port.flush();
    sim800PortActive = 1;
  }
  delay(500);
}

void sim800Configure(){
  sim800Comms=0;

  sim800Port.flush();
    delay(1000);

  while(!sim800Comms){
    Serial.println("Configuring SIM800, sending AT command");
    sim800Port.write("AT\r\n");
    delay(200);
    while(!sim800Port.available()){
    }

    sim800Comms=1;
    Serial.println("Serial port availbe, got:");
    buffer = sim800Port.readString();
    Serial.println(buffer);
      
    // Match baud
    sim800Port.write("AT\r\n");
    delay(200);
    //Put into text mode
    sim800Port.write("AT+CMGF=1\r\n");
    delay(200);
    //Delete messages
    sim800Port.write("AT+CMGD=2,4\r\n");
    delay(200);
    sim800Configured = 1;

    sim800Port.flush();
  }
}

void sim800Sleep(){
  Serial.println("sim800Sleep()"); 
  activateSim800Port();
  sim800Port.write("AT+CSCLK=2\r\n");
  delay(200);
  sim800Port.flush();
}

void sim800Wake(){
  Serial.println("sim800Wake()"); 
  activateSim800Port();
  sim800Port.write("AT+CSCLK=0\r\n");
  delay(200);
  sim800Port.write("AT+CSCLK=0\r\n");
  delay(200);
  sim800Port.flush();
}

void sendSMSPrefix(String number){
  sim800Wake();
  Serial.println("Sending SMS...");             
  sim800Port.print("AT+CMGF=1\r");                   //Set the module to SMS mode
  delay(100);
  sim800Port.print("AT+CMGS=\"");
  sim800Port.print(sms.senderNumber);
  sim800Port.print("\"\r");

  delay(500);

}

void sendSMSSuffix(){
  delay(500);
  sim800Port.print((char)26);// (required according to the datasheet)
  delay(500);
  sim800Port.println();
  Serial.println("Text Sent.");
  delay(500);
  sim800Sleep();
}

void sendSMS(String msg, String number){
  sendSMSPrefix(number);
  sim800Port.print(msg);       //This is the text to send to the phone number, don't make it too long or you have to modify the SoftwareSerial buffer
  sendSMSSuffix();
}

void sendPositionSMS(String number){
  sendSMSPrefix(number);
  sim800Port.print("http://maps.google.com/?q=");
  sim800Port.print(posllh.lat/10000000.0f,8);
  sim800Port.print(",");
  sim800Port.print(posllh.lon/10000000.0f,8);
  sim800Port.println();
  sim800Port.print(" hAcc: ");    
  sim800Port.print(posllh.hAcc/1000.0f);
  sim800Port.print(" vAcc: ");    
  sim800Port.print(posllh.vAcc/1000.0f);
  sendSMSSuffix();
}

int getMostRecentMSGIndex(String rxString){
  char buf[4];
  rxString.toCharArray(buf,rxString.length());

  if(isdigit(buf[rxString.length()-4])){
    sprintf(catInt,"%c%c",buf[rxString.length()-4],buf[rxString.length()-3]);
  }else{
    sprintf(catInt,"%c",buf[rxString.length()-3]);
  }

  return atoi(catInt);

}

bool processMessage(int index){

  char cmd[24];
  int i = 0;
  int lineCounter = 0;
  int numberStart = 0;
  int numberEnd = 0;
  int bodyStart = 0;
  int bodyEnd = 0;
  String buf;
  String trash;

  sms.message = "";
  sms.senderNumber = "";

  sim800Wake();
  // Get rid of any junk in the buffer
  if(sim800Port.available())
    trash = sim800Port.readString();
  

  sprintf(cmd,"AT+CMGR=%d\r\n",index);
  sim800Port.print(cmd);

  while(!sim800Port.available());
  buf=sim800Port.readString();

  // Find start of number by looking for +4
  while(buf[i] != '+' || buf[i+1] != '4'){
    i++;
  }
  numberStart = i;

  // Find end of number by looking for ,
  while(buf[i] != ','){
    i++;
  }
  numberEnd = i-1;

  // Create a string for the number
  for(i = numberStart; i < numberEnd; i++){
    sms.senderNumber+=buf[i];
  }

  for(i = 0; i < buf.length(); i ++){
  
    if(buf[i]=='\n'){
      lineCounter ++;
    }
    if(lineCounter == 2 && bodyStart == 0){
      bodyStart = i+1;
    }
    if(lineCounter == 3 && bodyEnd == 0){
      bodyEnd = i-1;
    }
  }

  for(i = bodyStart; i < bodyEnd; i++)
    sms.message += buf[i];

  sim800Sleep();
  if(sms.senderNumber.length() == 13 and sms.message.length() > 0)
    return true;
  else 
    return false;
}



void setup() 
{
  Serial.begin(9600);
  Serial.println("Awake");
  gpsPortActive = 0;
  sim800PortActive = 0;
  gpsConfigured = 0;
  gpsMessageCounter = 0;
  gpsLock = 0;
  gpsPositionRequest = 0;
  gpsAwake = 1;
  gpsCommsErrorCounter = 0;

  //Config
  useGSM = true;
  GSMPassthrough = false;
  
  //power_all_disable();
  sim800Port.begin(9600);
}

void loop() {

  while(GSMPassthrough){
    if (Serial.available()) {      // If anything comes in Serial (USB),

        sim800Port.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)

      }

      if (sim800Port.available()) {     // If anything comes in Serial1 (pins 0 & 1)

        Serial.write(sim800Port.read());   // read it and send it out Serial (USB)

      }
  }
  
  if(useGSM){
    if(!sim800Configured){
      activateSim800Port();
      delay(100);
      sim800Configure();
      sim800Sleep();
    }
  }

  if (!gpsConfigured){
    activateGpsPort();
    delay(100);
    gpsConfigure();
  }

  if(useGSM){
    activateSim800Port();
    if(sim800Port.available() && gpsPositionRequest==0){
      buffer=sim800Port.readString();
      
      if(buffer.startsWith("+CMTI:",2)){

        Serial.println("Got a text message");
        newestMsgIndex = getMostRecentMSGIndex(buffer);
        processMessage(newestMsgIndex);
        if(sms.message[0] == 'P'){
          sendSMS("Got position command, waiting for lock...",sms.senderNumber);
          gpsPositionRequest = 1;
        }
      }
      sim800Port.flush();
    }
  }
  
  
  if(Serial.available()){
    char c = Serial.read();
    if(c=='p'){
      Serial.println("Got position command, waiting for lock...");
      gpsPositionRequest = 1;
    }else{
      Serial.flush();
    }
  }

  // Get GPS position
  if(gpsPositionRequest){
    activateGpsPort();

    gpsAwakeCounter = 0;
    if(!gpsAwake)
      gpsWake();

    gpsPort.write(gpsPOSLLH_CMD,sizeof(gpsPOSLLH_CMD));
    delay(200);

  
    if(!gpsPort.available()){
      gpsCommsErrorCounter++;
    }else{
      gpsCommsErrorCounter = 0;
    }    

    if(gpsCommsErrorCounter>5){
      Serial.println("GPS not responding");
      gpsPositionRequest = 0;
      gpsCommsErrorCounter = 0;
      gpsMessageCounter = 0;
    }
    // Send GPS coords if accuracy is good enough, timed out or gps lock has been achieved without subsequent sleep
    if(processGPS()&&((posllh.hAcc/1000.0f <= HORIZONTAL_ACC_THRESHOLD)||(gpsMessageCounter>GPS_LOCK_MSG_LIMIT)||(gpsLock==1))){
      gpsLock = 1;
      gpsMessageCounter = 0;
      gpsPositionRequest = 0;

      if(useGSM){
        activateSim800Port();
        sendPositionSMS(sms.senderNumber);
      }
      
      Serial.print("http://maps.google.com/?q=");Serial.print(posllh.lat/10000000.0f,8);Serial.print(",");Serial.print(posllh.lon/10000000.0f,8);Serial.println();
      Serial.println();
      Serial.print(bufferArray);

      Serial.print(" hAcc: ");    Serial.print(posllh.hAcc/1000.0f);
      Serial.print(" vAcc: ");    Serial.print(posllh.vAcc/1000.0f);
      Serial.println();
    }

    if(!gpsLock)
      gpsMessageCounter++;
  }

  if(gpsLock && gpsAwake)
    gpsAwakeCounter++;

  if(gpsAwakeCounter > STAY_AWAKE_CYCLE){
    activateGpsPort();
    gpsSleep();
    activateSim800Port();
  }

  
  delay(1000);
  
}