/*
 This will act as a general node, although we'll need to change the address for each chip
 TODO
  -functoins (modular)
  -function to count packet failure rate

  -Brown out errors? Could be caused by unused ISRs, try commenting out the ISR enabling things for timers
  -Also figure out how to detect LED on manually without need for a tracking variable
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
#define LED 3 //4
#define SWITCH 9
#define sirenOut 4
ADXL345_JaMNN adxl;
bool ledOn = false;

volatile uint16_t waveTop = 0x00BF;
volatile uint16_t wavePos = 0x0000;
volatile uint16_t pos = 0x0000;

volatile uint16_t& waveSpeed = OCR1A;
volatile uint8_t& ledOut = OCR2B;
const uint16_t maxCycles = 250;
volatile uint16_t cyclesOn = 0;

uint8_t state;
uint16_t count;
bool intFlag;
#define CE 7
#define  CSN  10 //8
// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE, CSN); // using pin 7 for the CE pin, and pin 8 for the CSN 
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

  setupPWM();

  //digitalWrite(LED,1);
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
  while(network.available()){
    Serial.println(currNode);
    Serial.println("RX");
    RF24NetworkHeader header;
    uint16_t incomingData;
    network.read(header, &incomingData, sizeof(incomingData));
    uint16_t inCommand = (incomingData & 0xF000)>>12;
    uint16_t inData = incomingData & 0x0FFF;
    if(header.from_node == 00){
      if(currNode == 01) { //SHOULD RUN ONCE, SHOULD CHANGE TO 02 BC OF MASTER
        network.begin(90,(uint16_t) incomingData);
        currNode = (uint16_t)incomingData;
        continue;
      }
      switch(inCommand) {
        case 1 : // Set WavePos
          wavePos=inData;
          break;
        case 2 : // Set WaveTop
          waveTop=inData;
          break;
        case 3 : // Set Pos
          pos=inData;
          break;
        
      }
      if(inCommand==1){ //Set WavePos
        wavePos=inData;
      } else if(inCommand==2){ //Set WaveTop
        waveTop=inData;
      } else if(inCommand==3){
        
      }
    }
  }
  
  //############## TRANSMIT ######################
  uint16_t outData=0x0000;
  if(currNode == 01)
  {
    outData=0xF001;
    sendData(outData,master);
  }
  else
  {
    if(state==2){
      outData=0x0001;
      sendDataVital(outData,master);
    } else {
      outData=0x0000;
      sendData(outData, master);  
    }
  }
  delay(100);
}

void sendData(uint16_t outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
}

void sendDataVital(uint16_t outGoingData, uint16_t dest) { //Only used for sending a distress signal right now
  RF24NetworkHeader header1(dest); //destination
  bool ok=false;
  uint8_t c=0;
  while(!ok||c<=5){
    ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED 
    c++;
  }
  if(!ok){
    state==2;
  } else {
    state==1;
  }
}

void interruptFound(){ //PLACEHOLDER FUNCTION FOR WHAT SHOULD BE DONE WHEN THE COLLLISION IS DETECTED
  interrupts();
  adxl.clearInterrupts();
  network.update();
  sendDataVital(0x0001,master);
  //digitalWrite(LED,0);
}

ISR(INT0_vect)
{
  interruptFound();
}

ISR(TIMER1_COMPA_vect) //This function runs everytime the TIMER1 CCRB register matches the current timer1 value
{
  if(pos>=waveTop){
    pos=0;
  } else {
    pos++;
  }
  if(wavePos==pos){
    cyclesOn=0;
    TCCR2A |= _BV(COM2B1); //sets light on
    ledOn = true;
  }
}

ISR(TIMER2_OVF_vect) //This function runs everytime the TIMER2 overflows, it might take too long and cause errors, only one way to find out
{
  if(!ledOn) //Checks to see if OC2B is off
  {
    return;  //If off, then leave immediatly
  }
  if(cyclesOn>=maxCycles){ //If over the desired cycles, then turn off light
    TCCR2A&=~_BV(COM2B1); // Sets the light as off
    ledOn=false;
    //digitalWrite(LED,0);
   return; 
  }
  cyclesOn++;
}

void setupPWM(){ //Sets up the Timer2 registers to support the 8 bit fast PWM mode for output B
  TIMSK2 = (TIMSK2 & B11111110) | 0x01; //Enables timer overflow interrupt
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //Mode 3, fast PWM that counts to 0xFF, sets up OC2B as non-inverting output
  TCCR2B = _BV(CS22) | _BV(CS21); // Prescaler = 128, a prescaler of 256 might work, but worried about speed of traffic driving by noticing the strobe, not a large power loss anyways
  TCCR2A&=~_BV(COM2B1); //Turn off led output
  ledOn = false;

  waveSpeed=0xFFFF; //Another name for OCR1A, which determines how long it takes to do a wave cycle
  TCCR1A = _BV(0); //mode 4, CTC for generic timing
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10); // prescaler = 256, estimated to be ~2s, if change prescaler to 1024 it would be ~10s estimated
  TIMSK1 = (TIMSK1 & B11111101) | 0x02; //Enables compare register A interrupt

  ledOut=50;
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
