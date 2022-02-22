/*
 This will act as a general node, although we'll need to change the address for each chip
 TODO
  -functoins (modular)
  -function to count packet failure rate
 */


#include <Wire.h>
#include <SPI.h>
#include "ADXL345_JaMNN.h"
#include "printf.h"
#include "RF24.h"
#include <RF24Network.h>

#define intPin 2
#define LED 8
#define SWITCH 9
#define sirenOut 4
ADXL345_JaMNN adxl;

uint8_t state;
uint16_t count;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN 
RF24Network network(radio);
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;

void setup() {
  state=0;
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90, node01);  //(channel, node address) CHANGE NODE ADDRESS FOR EACH DIFFERENT NODE
  radio.setDataRate(RF24_2MBPS);
  pinMode(LED, OUTPUT);//LED
  pinMode(SWITCH, INPUT); //SWITCH
  
  adxl.Init(RANGE_16g);
  pinMode(intPin, INPUT);
  adxl.Init_Active_Interrupts();
  attachInterrupt(digitalPinToInterrupt(intPin),interruptFound, RISING);

  count=0;
  state=1; //Normally operating
}

void loop(){
  if(state=2){
    digitalWrite(LED,1);
  } else {
    digitalWrite(LED,0);
  }
  network.update();
  RF24NetworkHeader header1(master); //destination
  bool ok = network.write(header1, &state, sizeof(state)); //1 means SUCCESS, 0 means PACKET FAILED
  //Serial.println(ok);

  count++;
  if(state!=2){
    count=0;
  }
  if(count=3000){
    state=1;
  }
}

void interruptFound(){ //PLACEHOLDER FUNCTION FOR WHAT SHOULD BE DONE WHEN THE COLLLISION IS DETECTED
  state=2; //Collision detected
  adxl.clearInterrupts();
}
