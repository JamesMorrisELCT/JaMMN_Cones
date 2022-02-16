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

void setup() {
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90, node03);  //(channel, node address) CHANGE NODE ADDRESS FOR EACH DIFFERENT NODE
  radio.setDataRate(RF24_2MBPS);
  pinMode(2, OUTPUT);//LED
  pinMode(3, INPUT); //SWITCH
}

void loop(){
  int switchIn = digitalRead(3);
  digitalWrite(2,switchIn);
  network.update();
  RF24NetworkHeader header1(master); //destination
  bool ok = network.write(header1, &switchIn, sizeof(switchIn)); //1 means SUCCESS, 0 means PACKET FAILED
  //Serial.println(ok);
}
