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
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90, master);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  pinMode(2, OUTPUT);//LED 1
  pinMode(3,INPUT); //SWITCH
  pinMode(4, OUTPUT); //LED 2
  pinMode(5, OUTPUT); //LED 3
}

void loop() {
  network.update();
  while ( network.available() ) {     // Is there any incoming data?
    RF24NetworkHeader header;
    unsigned long payload;
    network.read(header, &payload, sizeof(payload)); // Read the incoming data
    //Serial.println(payload);
    if(header.from_node == 01 ){
      digitalWrite(2,payload);
    }
    if(header.from_node == 02){
      digitalWrite(4,payload);
    }
    if(header.from_node == 03){
      digitalWrite(5,payload);
    }
  }
}
