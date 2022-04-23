/*
 This will act as the boombox controller. It will balance transmitting out to the nodes and receiving their data, instead of only doing 1 function
*/


#define ENABLE false

#include <Wire.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <RF24Network.h>

volatile uint16_t waveTop = 0x00BF;
volatile uint16_t wavePos = 0x0000;
volatile uint16_t pos = 0x0000;

volatile uint16_t& waveSpeed = OCR1A;
volatile uint8_t& ledOut = OCR2B;
const uint16_t maxCycles = 150;
volatile uint16_t cyclesOn = 0;
bool ledOn = false;

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

uint8_t loopCount = 0;

#define SWITCH 2 //10

#define siren 3 //9


void setup() {
  alterClock();
  Serial.begin(9600);
  SPI.begin();
  radio.begin();
  network.begin(90, master);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  setupPWM();
  
  //pinMode(2, OUTPUT);//LED 1
  pinMode(siren,OUTPUT); //SWITCH
  digitalWrite(siren,1);
  pinMode(SWITCH, INPUT);
  
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
      bool ok = sendDataTested(nodeCount,baseNode);
      if(!ok){
        continue;
      }
      delay(3000);
      sendData(0x3000 | (pos & 0x0FFF),nodeCount);
      if(nodeCount!=02){
        //sendData(0x105F,nodeCount);
      }
      nodeCount++; //increment for the next available node
      totalNodes++;
      Serial.println("Changed!");
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


  switch(loopCount){
    case 1 : // These are just arbitrary values to make sure that the "long" time consuming operations are spread out
      synchAllTime();
      break;
    case 100 :
      synchAllPos();
      break;
    case 200 :
      synchAllWaveTop();
      break;
    case 300 :
      //synchAllWaveSpeed();
      break;
    case 400 :
      synchAllMaxCycles();
      break;
    case 500 :
      synchAllLedOut();
      break;
    case 600 :
      loopCount=0;
      break;
    default :
      break;
  }

  if(digitalRead(SWITCH)==0){
    digitalWrite(siren,0);
  }
  Serial.println(loopCount);
  loopCount++;
}
void synchAllPos(){
  uint16_t nDist=waveTop/(totalNodes+1);
  uint16_t currDist=0;
  for(i=02; i<=02+totalNodes-1; i++){
    sendData(0x1000 | (currDist & 0x0FFF),i);
    currDist=currDist+nDist;
  }
}

void synchAllTime(){
  for(i=02; i<=02+totalNodes-1; i++){
    sendData(0x3000 | (pos & 0x0FFF),i);
  }
}

void synchAllWaveTop(){
  for(i=02; i<=02+totalNodes-1; i++){
    sendData(0x2000 | (waveTop & 0x0FFF),i);
  }
}

void synchAllWaveSpeed(){
  for(i=02; i<=02+totalNodes-1; i++){
    sendData(0x4000 | (waveSpeed & 0x0FFF),i);
  }
}

void synchAllMaxCycles(){
  for(i=02; i<=02+totalNodes-1; i++){
    sendData(0x5000 | (maxCycles & 0x0FFF),i);
  }
}

void synchAllLedOut(){
  for(i=02; i<=02+totalNodes-1; i++){
    sendData(0x6000 | (ledOut & 0x0FFF),i);
  }
}

void alterClock(){
  CLKPR=B10000000;
  CLKPR=B00000001;
}

void sendData(uint16_t outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
}

bool sendDataTested(uint16_t outGoingData, uint16_t dest) {
  RF24NetworkHeader header1(dest); //destination
  bool ok = network.write(header1, &outGoingData, sizeof(outGoingData)); //1 means SUCCESS, 0 means PACKET FAILED
  return ok;
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
    //TCCR2A |= _BV(COM2B1); //sets light on
    //ledOn = true;
  }
}

ISR(TIMER2_OVF_vect) //This function runs everytime the TIMER2 overflows, it might take too long and cause errors, only one way to find out
{
  if(!ledOn) //Checks to see if OC2B is off
  {
    return;  //If off, then leave immediatly
  }
  if(cyclesOn>=maxCycles){ //If over the desired cycles, then turn off light
    //TCCR2A&=~_BV(COM2B1); // Sets the light as off
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

  ledOut=254;
}
