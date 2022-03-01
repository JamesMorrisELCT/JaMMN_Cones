/*
 This will act as a general node, although we'll need to change the address for each chip
 TODO
  -functoins (modular)
  -function to count packet failure rate
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
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;
uint16_t currNode = 01;
int state = 0;
void setup() {
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90, currNode);  //(channel, node address) CHANGE NODE ADDRESS FOR EACH DIFFERENT NODE
  radio.setDataRate(RF24_2MBPS);
  pinMode(2, OUTPUT);//LED
  pinMode(3, INPUT); //SWITCH
}

void loop(){
  network.update(); //NEED AT BEGINNING OF EVERY LOOP

  // ############## INITIALIZE ###################
  
  //############### RECEIVE ######################
 // RF24NetworkHeader headertemp;
  //network.peek(&headertemp);
  //Serial.println(headertemp.from_node);
  while(network.available()){
    Serial.println(currNode);
    Serial.println("RX");
    RF24NetworkHeader header;
    int incomingData;
    network.read(header, &incomingData, sizeof(incomingData));
    if(header.from_node == 00){
      if(currNode == 01) { //SHOULD RUN ONCE, SHOULD CHANGE TO 02 BC OF MASTER
        network.begin(90,(uint16_t) incomingData);
        currNode = (uint16_t)incomingData;
      }
      else{
       digitalWrite(2,incomingData); //CHANGE BASED ON DEMO 
      }
    }
  } 
  
  //############## TRANSMIT ######################
  if(currNode == 01)
  {
    sendData(1,master);
  }
  else
  {
    int switchIn = digitalRead(3);
    sendData(switchIn, master); 
  }
  delay(100);
}

void sendData(int outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
}
