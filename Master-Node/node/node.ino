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
#include <avr/io.h>
#include <avr/interrupt.h>

#define intPin 2
#define LED 4
#define SWITCH 9
#define sirenOut 4
ADXL345_JaMNN adxl;

uint8_t state;
uint16_t count;
bool intFlag;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN 
RF24Network network(radio);
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;
uint16_t currNode = 01;

void setup() {
  state=0;
  Serial.begin(9600);
  SPI.begin();
  radio.begin();
  network.begin(90, currNode);  //(channel, node address) CHANGE NODE ADDRESS FOR EACH DIFFERENT NODE
  radio.setDataRate(RF24_2MBPS);
  pinMode(LED, OUTPUT);//LED
  pinMode(SWITCH, INPUT); //SWITCH

  adxl.Init(RANGE_16g);
  pinMode(intPin, INPUT);
  adxl.Init_Active_Interrupts();
  //attachInterrupt(digitalPinToInterrupt(intPin),interruptFound, RISING);
  setupExtInterrupt();

  digitalWrite(LED,1);
  count=0;
  state=1; //Normally operating
  intFlag=0;
  sei();
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

void interruptFound(){ //PLACEHOLDER FUNCTION FOR WHAT SHOULD BE DONE WHEN THE COLLLISION IS DETECTED
  interrupts();
  state=2; //Collision detected
  adxl.clearInterrupts();
  digitalWrite(LED,0);
}

ISR(INT0_vect)
{
  interruptFound();
}

void setupExtInterrupt(){
    // Configure PD2 as an input using the Data Direction Register D (DDRD)
    DDRD &= ~_BV(DDD2);

    // Enable the pull-up resistor on PD2 using the Port D
    // Data Register (PORTD)
    //PORTD |= _BV(PORTD2);

    // Configure external interrupt 0 to generate an interrupt request on any
    // logical change using External Interrupt Control Register A (EICRA)
    EICRA |= _BV(ISC00|ISC01);

    // Enable external interrupt 0 using the External Interrupt Mask Register
    // (EIMSK)
    EIMSK |= _BV(INT0);
}
