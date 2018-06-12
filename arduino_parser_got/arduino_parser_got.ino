/*
******************************************************************************
* Parsing of NMEA positional data to AutoQuad flight controller in UBX format
* Copyright (c) 2018, Kristian Husum Terkildsen <khte@mmmi.sdu.dk>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the copyright holder nor the names of its
*      contributors may be used to endorse or promote products derived from
*      this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************
*/
#define BAUDDEBUG 9600
#define BAUDPOS 57600
#define BAUDAQ 230400  //AQ required baud rate
#define MESSAGE_SIZE 100  //Size of message container MAKE SMALLER

#define DEFAULT_ACCURACY 50//2900//500  //Will be variable at a later stage
#define DEFAULT_DOP 100//100  //Will be variable at a later stage


char receivedMessage[MESSAGE_SIZE];
boolean newMessage = false;
long latitude = 0;
long longitude = 0;
long altitude = 0;
long oldLatitude = 0;
long oldLongitude = 0;
long positionHack = 2;

int counter = 0;

//Initialize UBX messages
byte NAV_POSLLH[] = {0xB5, 0x62, 0x01, 0x02, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte NAV_DOP[] = {0xB5, 0x62, 0x01, 0x04, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup(){
  delay(10000);
  
  //Serial.begin(BAUDDEBUG); ///////DEBUG
  Serial1.begin(BAUDPOS);
  Serial2.begin(BAUDAQ);
  //Serial3.begin(BAUDLASER);

  //Set default accuracies
  byteSplit(&NAV_POSLLH[26], DEFAULT_ACCURACY, 4);
  byteSplit(&NAV_POSLLH[30], DEFAULT_ACCURACY, 4);
  checksum(&NAV_POSLLH[2], sizeof(NAV_POSLLH) - 4);
  //Set default DOPs
  for(int i = 10; i < 24; i = i + 2){
    byteSplit(&NAV_DOP[i], DEFAULT_DOP, 2);
  }
  checksum(&NAV_DOP[2], sizeof(NAV_DOP) - 4);
}

void loop(){
  receiveMessage();
  parseNMEA();
  //messageTiming();
}

////////////////////RECEIVING////////////////////
void receiveMessage(){
  static boolean receiving = false;
  static byte index = 0;
  char startChar = '$';
  char endChar = '\n';
  char readData;
  
  while(Serial1.available() > 0 && newMessage == false){
    readData = Serial1.read();
    //Serial.write(readData);
    if(receiving == true){
      if(readData != endChar){
        receivedMessage[index] = readData;
        index++;
        if(index >= MESSAGE_SIZE){
          index = MESSAGE_SIZE - 1;
        }
      }
      else{
        receivedMessage[index] = '\0';
        receiving = false;
        index = 0;
        newMessage = true;
      }
    }
    else if(readData == startChar){
      receiving = true;
    }
  }
}

void parseNMEA(){
  static char posMSG[] = "GGA";
  char temp[10] = {0};
  if(newMessage == true){
    //NMEAchecksumCalculator();
    char* strtokIndex;
    strtokIndex = strtok(receivedMessage, ",");
    if(posMSG[0] == strtokIndex[0] && posMSG[1] == strtokIndex[1] && posMSG[2] == strtokIndex[2]){
      parseGGAData(strtokIndex);
    }
    newMessage = false;
  }
}

void parseGGAData(char* index){
  index = strtok(NULL, ",");
  latitude = atol(index);
  index = strtok(NULL, ",");
  longitude = atol(index);
  index = strtok(NULL, ",");
  altitude = atol(index);

  //Workaround due to AQ's handling of a uBlox error, where the module sends the same position multiple times.
  if(latitude == oldLatitude && longitude == oldLongitude){
    positionHack = -positionHack;
    latitude += positionHack;
    oldLatitude = latitude;
  }
  else{
    oldLatitude = latitude;
    oldLongitude = longitude;
  }

  counter++;
  positionUpdate();
  sendUBXMessage(&NAV_POSLLH[0], sizeof(NAV_POSLLH));
  if(counter > 50){//100){
    sendUBXMessage(&NAV_DOP[0], sizeof(NAV_DOP));
    counter = 0;
  }
  /*
  Serial.println(latitude);
  Serial.println(longitude);
  Serial.println(altitude);
  */  
}

/*
void NMEAchecksumCalculator(){
  byte start = 0;
  byte endit = 0;
  byte CRC = 0;
  for(int i = 0; i < strlen(receivedMessage); i++){
    if(receivedMessage[i] == '*'){
      endit = i;
    }
  }
  for(byte x = start; x < endit; x++){
    CRC = CRC ^ receivedMessage[x];
  }
  //Serial.println(CRC, HEX);
 // return 1;
}
*/


////////////////////TRANSMITTING////////////////////
//Takes a UBX message and the payload size and calculates the checksum for a UBX message
void checksum(byte *messagePayload, int payloadSize){
  byte CK_A = 0, CK_B = 0;
  for(int i = 0; i < payloadSize ;i++){
    CK_A = CK_A + *messagePayload;
    CK_B = CK_B + CK_A;
    messagePayload++;
  }
  *messagePayload = CK_A;
  messagePayload++;
  *messagePayload = CK_B;
}

//Takes a UBX message array, an integer value and a number of bytes and assigns the integer value to the number of bytes in little endian format
void byteSplit(byte *byteArray, long integerValue, int numberOfBytes){
  *byteArray = integerValue & 0xFF;
  for(int i = 8; i < numberOfBytes * 8; i = i + 8){
    byteArray++;
    *byteArray = (integerValue >> i) & 0xFF;
  }
}

//Takes latitude and longitude with a scaling factor of 1e7, altitude in millimeters and updates the UBX position message
void positionUpdate(){
  byteSplit(&NAV_POSLLH[10], longitude, 4);
  byteSplit(&NAV_POSLLH[14], latitude, 4);
  //byteSplit(&NAV_POSLLH[18], altitude, 4); //TEST IGNORING THE HEIGHT ABOVE ELLIPSOID
  byteSplit(&NAV_POSLLH[22], altitude, 4);
  checksum(&NAV_POSLLH[2], sizeof(NAV_POSLLH)-4);
}

//Takes a UBX message and its length and sends the message over serial
void sendUBXMessage(byte *UBXmessage, byte messageLength){
  for(int i = 0; i < messageLength; i++) {
    Serial2.write(UBXmessage[i]);
    Serial2.flush(); //Wait for data to be sent and buffer to clear
  }
}
