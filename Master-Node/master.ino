/*
 This will act as the boombox controller. It will balance transmitting out to the nodes and receiving their data, instead of only doing 1 function
*/


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
unsigned long prevTime = 0;
unsigned long currTime;
bool state = 1;
void setup() {
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90, master);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  //radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  pinMode(2, OUTPUT);//LED 1
  pinMode(3,INPUT); //SWITCH
  pinMode(4, OUTPUT); //LED 2
  pinMode(5, OUTPUT); //LED 3
}

void loop() {
  network.update();
  //################## Receive #####################
  //RF24NetworkHeader headertemp;
  //network.peek(&headertemp);
  //Serial.println((int)headertemp.from_node);
  while ( network.available() ) {     // Is there any incoming data?
    //Serial.println("RX");
    RF24NetworkHeader header;
    int incomingData;
    network.read(header, &incomingData, sizeof(incomingData)); // Read the incoming data
    
    if(header.from_node == 01 ){ //TAKE ACTION ON RECIEVE PAYLOAD
      Serial.println("changing");
      sendData(nodeCount,baseNode);
      nodeCount++; //increment for the next available node
      totalNodes++;
    }
    if(header.from_node == 02){
      digitalWrite(2,incomingData);
    }
    if(header.from_node == 03){
      digitalWrite(4,incomingData);
    }
  } 

  
  // ############### Transmit on interval #########
  currTime = millis();
  if(currTime -prevTime >= 1000){
    sendData((int)state,node02);
    prevTime = currTime;
    state = !state;
  }

  //############## TRANSMIT ################
  /*
  int switchIn = digitalRead(3);
  sendData(switchIn, node02);
  sendData(switchIn, node03);
  delay(50); */
}

void sendData(int outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
}

void sendAddy(uint16_t outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
}
