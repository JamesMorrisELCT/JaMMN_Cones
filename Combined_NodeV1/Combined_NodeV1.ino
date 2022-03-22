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

void setup() {
  state=0;
  Serial.begin(9600);
  SPI.begin();
  radio.begin();
  network.begin(90, node01);  //(channel, node address) CHANGE NODE ADDRESS FOR EACH DIFFERENT NODE
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
  /*if(state==2){
    digitalWrite(LED,1);
  } else {
    digitalWrite(LED,0);
  }*/
  network.update();
  RF24NetworkHeader header1(master); //destination
  bool ok = network.write(header1, &state, sizeof(state)); //1 means SUCCESS, 0 means PACKET FAILED
  //digitalWrite(LED,ok|digitalRead(SWITCH));
  //Serial.println(ok);

 /* count++;
  if(state!=2){
    //count=0;
  }
  if(count==20000){
    interruptFound();
    state=1;
    count=0;
  }*/

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
