/* FILE:    ARD_GY291_ADXL345_Example
   DATE:    02/04/14
   VERSION: 0.1
   
REVISIONS:

02/04/14 Created version 0.1

This is an example of how to use the Hobby Components GY-291 accelerometer module 
(HCMODU0060). This module is based on the Analog Devices ADXL345 triple axis 
accelerometer device. 

This example sketch will demonstrate how perform a basic initialisation and then 
will continually read each of the 3 axis registers and output them to the serial port.

PINOUT:

MODULE`                Arduino
GND                    GND
VCC                    +3.3V
CS                     +3.3V*
INT1                   N/A
INT2                   N/A
SD0                    N/A
SDA                    A4*
SCL                    A5*

*Please note that the ADXL345 opperates at 3.3V (via a 3.3V regulator) and these 
pins should not be driven above 3.6V therefore you may require level shifters to
ensure safe opperation.

You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used for the purpose of promoting or selling products 
that directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.
*/

/* Include the standard wire library */
#include "ADXL345_JaMNN.h"
#include <Wire.h>

/* Alternate I2C address of the module */
#define I2C_Add 0x53

/* ADXL345 register addresses */
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define X_Axis 0x32
#define Y_Axis 0x34
#define Z_Axis 0x36

#define THRESH_ACT 0x24
#define ACT_INACT_CTL 0x27
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30

/* Accelerometer range modes */
#define RANGE_2g 0
#define RANGE_4g 1
#define RANGE_8g 2
#define RANGE_16g 3

#define intPin 2
#define sirenOut 4
ADXL345_JaMNN adxl;

void setup()
{
  /* Initialise the I2C bus */
  Wire.begin();  
  
  /* Initialise the serial interface */
  Serial.begin(9600);
  
  /* Initialise the ADXL345 */  
  adxl.Init(RANGE_16g);
  pinMode(intPin, INPUT);
  pinMode(sirenOut, OUTPUT);
  digitalWrite(sirenOut, 0);
  adxl.Init_Active_Interrupts();
  attachInterrupt(digitalPinToInterrupt(2),interruptFound, RISING);
}

/* Main program */
void loop()
{
  /* Continually read and output all 3 axis to the serial port */
  
  Serial.print("X: ");
  Serial.print(adxl.Read_Axis(X_Axis));
  
  Serial.print(" Y: ");
  Serial.print(adxl.Read_Axis(Y_Axis));
  
  Serial.print(" Z: ");
  Serial.print(adxl.Read_Axis(Z_Axis));

  Serial.print(" I: ");
  Serial.print(digitalRead(intPin));
  
  /*
  Serial.print(" Device ID: ");
  Serial.println(getDevID());
  */
  
  Serial.println();

  if(digitalRead(intPin)==1){
    interruptFound();
  }
}




void interruptFound(){ //PLACEHOLDER FUNCTION FOR WHAT SHOULD BE DONE WHEN THE COLLLISION IS DETECTED
  digitalWrite(sirenOut, 1);
  adxl.clearInterrupts();
  delay(1000);
  digitalWrite(sirenOut, 0);
}
