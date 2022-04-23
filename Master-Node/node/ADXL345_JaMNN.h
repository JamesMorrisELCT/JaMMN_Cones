#include "Arduino.h"
#include <Wire.h>

#ifndef ADXL345_JaMNN_h
#define ADXL345_JaMNN_h

#define DEVID 0x00
#define THRESH_TAP 0x1D
#define OFSX 0x1E
#define OFSY 0x1F
#define OFSZ 0x20
#define DUR 0x21
#define LATENT 0x22
#define WINDOW 0x23
#define THRESH_ACT 0x24
#define THRESH_INACT 0x25
#define TIME_INACT 0x26
#define ACT_INACT_CTL 0x27
#define THRESH_FF 0x28
#define TIME_FF 0x29
#define TAP_AXES 0x2A
#define ACT_TAP_STATUS 0x2B
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define X_Axis 0x32
#define Y_Axis 0x34
#define Z_Axis 0x36
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39


#define I2C_Add 0x53 //Default address if the address pin stays low
//#define I2C_Add 0x1D //Alternate address if the address pin is brought high

#define RANGE_2g 0
#define RANGE_4g 1
#define RANGE_8g 2
#define RANGE_16g 3


class ADXL345_JaMNN{
  public:
ADXL345_JaMNN();
int Read_Axis(byte);
void Init(byte);
void Init_Active_Interrupts();
void clearInterrupts();
uint8_t getDevID();


  private:
  
};



#endif
