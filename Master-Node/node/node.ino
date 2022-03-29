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

#define CE 7 //PD7
#define CSN 8 //PB2
#define IRQ PC0

#define intPin 2
#define altInt PC2
#define LED 3
//#define SWITCH 9
ADXL345_JaMNN adxl;

uint8_t state;
uint16_t count;
bool intFlag;
int lightOn;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE, CSN); // using pin 7 for the CE pin, and pin 8 for the CSN 
RF24Network network(radio);
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;
uint16_t currNode = 02;

void setup() {
  state=0;
  Serial.begin(9600);
  Serial.println("Starting");
  SPI.begin();
  radio.begin();
  network.begin(90, currNode);  //(channel, node address) CHANGE NODE ADDRESS FOR EACH DIFFERENT NODE
  radio.setDataRate(RF24_2MBPS);
  pinMode(LED, OUTPUT);//LED
  //pinMode(SWITCH, INPUT); //SWITCH
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  


  //adxl.Init(RANGE_16g);

  digitalWrite(LED,1);
  count=0;
  state=1; //Normally operating
  intFlag=0;
  
  pinMode(intPin, INPUT);
  //adxl.Init_Active_Interrupts();
  //attachInterrupt(digitalPinToInterrupt(intPin),interruptFound, RISING);
  //setupExtInterrupt();
  sei();
  Serial.println("Initialized");
}

void loop(){
  network.update(); //NEED AT BEGINNING OF EVERY LOOP

  // ############## INITIALIZE ###################
  
  
  //############## TRANSMIT ######################
  if(currNode == 01)
  {
    sendData(1,master);
  }
  else
  {
    //uint8_t switchIn = digitalRead(SWITCH);
    uint8_t switchIn=0;
    if(state==2){
      switchIn=1;
    }
    //sendData(switchIn, master);
    Serial.print("Sending: ");
    Serial.println(lightOn);
    sendData(lightOn,master);
  }

lightOn=(lightOn+1)%2;

  digitalWrite(LED,lightOn);
  delay(100);
}

void sendData(int outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
  Serial.print("OK?: ");
  Serial.println(ok);
}

void interruptFound(){ //PLACEHOLDER FUNCTION FOR WHAT SHOULD BE DONE WHEN THE COLLLISION IS DETECTED
  interrupts();
  state=2; //Collision detected
  adxl.clearInterrupts();
  //digitalWrite(LED,0);
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
