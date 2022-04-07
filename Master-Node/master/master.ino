/*
 This will act as the boombox controller. It will balance transmitting out to the nodes and receiving their data, instead of only doing 1 function
*/


#define ENABLE false

#include <Wire.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <RF24Network.h>

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN 
RF24Network network(radio);
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t baseNode = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;
int nodeCount = 02; //START WITH 02 AND INCREASE
int totalNodes = 0;
int i=0;
unsigned long prevTime = 0;
unsigned long currTime;
unsigned long node02Time = 0;
unsigned long node03Time = 0;
unsigned long node04Time = 0;
bool node02Flag = 0;
bool node03Flag = 0;
bool node04Flag = 0;

#define siren 9


void setup() {
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90, master);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  //pinMode(2, OUTPUT);//LED 1
  pinMode(siren,OUTPUT); //SWITCH
  digitalWrite(siren,0);
  pinMode(10, INPUT);
  //pinMode(4, OUTPUT); //LED 2
  //pinMode(5, OUTPUT); //LED 3
  //pinMode(6, OUTPUT); //LED ALARM RED
}

void loop() {
  network.update();
  //################## Receive #####################
  //RF24NetworkHeader headertemp;
  //network.peek(&headertemp);
  //Serial.println((int)headertemp.from_node);
  while ( network.available() ) {     // Is there any incoming data?
    Serial.println("RX");
    RF24NetworkHeader header;
    uint16_t incomingData;
    network.read(header, &incomingData, sizeof(incomingData)); // Read the incoming data
    uint16_t inCommand = (incomingData & 0xF000)>>12;
    uint16_t inData = incomingData & 0x0FFF;
    Serial.println(inData);
    
    if(header.from_node == 01 ){ //TAKE ACTION ON RECIEVE PAYLOAD
      Serial.println("changing");
      sendData(nodeCount,baseNode);
      if(nodeCount!=02){
        sendData(0x105F,nodeCount);
      }
      nodeCount++; //increment for the next available node
      totalNodes++;
    } else {
      switch(inData) {
        case 0x0001 :
          digitalWrite(siren,1);
          break;
        default :
          break;
      }
    }
    /*
    if(header.from_node == 02){
      digitalWrite(2,incomingData);
      node02Flag = 1;
      node02Time = millis();
    }
    if(header.from_node == 03){
      node03Flag = 1;
      digitalWrite(4,incomingData);
      node03Time = millis();
    }
    if(header.from_node == 04){
      node04Flag = 1;
      digitalWrite(5,incomingData);
      node04Time = millis();
    }

    */
  } 

  
  // ############### Transmit on interval #########
  currTime = millis();

if(ENABLE)
{
  while((currTime - node02Time == 1000)&& node02Flag)
  {
    //destruction?
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
  }
  while((currTime - node03Time == 1000)&& node03Flag)
  {
    //destruction?
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
  }
  while((currTime - node04Time == 1000)&& node04Flag)
  {
    //destruction?
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
  }
}
  /*
  if(currTime -prevTime >= 500){
    sendData(1,(i%totalNodes)+2);
    sendData(0,((i-1)%totalNodes)+2);
    prevTime = currTime;
    i++;
  }
  */

  //############## TRANSMIT ################
  /*
  int switchIn = digitalRead(3);
  sendData(switchIn, node02);
  sendData(switchIn, node03);
  delay(50); */
  if(digitalRead(10)==0){
    digitalWrite(siren,0);
  }
}

void sendData(uint16_t outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
}
