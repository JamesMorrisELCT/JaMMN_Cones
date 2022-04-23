#include "ADXL345_JaMNN.h"

ADXL345_JaMNN::ADXL345_JaMNN(){
  return;
}

/* Read one of the 3 axis via the I2C interface */
int ADXL345_JaMNN::Read_Axis(byte axis) //NOTE: this function should be updated, as the axis's of interest should be read all at the same time for time efficiency reasons. Not TOO important
{
  int Data;
   
  Wire.beginTransmission(I2C_Add); 
  Wire.write(axis); 
  Wire.endTransmission(); 
  
  Wire.beginTransmission(I2C_Add);
  Wire.requestFrom(I2C_Add, 2);
  
  /* If data is available then read it (2 bytes) */
  if(Wire.available())     
  { 
    Data = (int)Wire.read();
    Data = Data  | (Wire.read() << 8);
  }else
  {
    Data = 0;
  }
    
  Wire.endTransmission();  
  return Data;
}

void ADXL345_JaMNN::Init(byte range)
{
  Wire.beginTransmission(I2C_Add);
  
  /* Set the sensitivity of the module */
  Wire.write(DATA_FORMAT); 
  Wire.write(range|B00100000); 
  Wire.endTransmission(); 
  
  /* Put the module into measurement mode to start taking measurements */
  Wire.beginTransmission(I2C_Add);
  Wire.write(POWER_CTL); 
  Wire.write(0x08); 
  
  Wire.endTransmission(); 
}

void ADXL345_JaMNN::Init_Active_Interrupts(){
  //TODO: store inital value of registers and only change the ones needed
  
  //Disable interrupts
  Wire.beginTransmission(I2C_Add);
  Wire.write(INT_ENABLE);
  Wire.write(B00000000);
  Wire.endTransmission();
  
  //Setup the control
  Wire.beginTransmission(I2C_Add);
  Wire.write(ACT_INACT_CTL);
  //Wire.write(B01110000);  DC
  Wire.write(B11110000);  //AC
  Wire.endTransmission();

  //Set the threshold value
  Wire.beginTransmission(I2C_Add);
  Wire.write(THRESH_ACT);
  Wire.write(0xFF);
  Wire.endTransmission();

  //Map them
  Wire.beginTransmission(I2C_Add);
  Wire.write(INT_MAP);
  Wire.write(B11101111);
  Wire.endTransmission();
  
  //Enable them
  Wire.beginTransmission(I2C_Add);
  Wire.write(INT_ENABLE);
  Wire.write(B00010000);
  Wire.endTransmission();
}

void ADXL345_JaMNN::clearInterrupts(){
  uint8_t Data;
  //Requests the data stored inside of register 0x30, which is INT_SOURCE
  Wire.beginTransmission(I2C_Add);
  Wire.write(INT_SOURCE);
  Wire.endTransmission();
  Wire.beginTransmission(I2C_Add);
  Wire.requestFrom(I2C_Add,1);
  if(Wire.available()){
    Data=(uint8_t)Wire.read();
  } else {
    Data=0;
  }
  Wire.endTransmission();
}



uint8_t ADXL345_JaMNN::getDevID(){ //Test function to make sure the controller is actually connected
  uint8_t Data;
  //Requests the data stored inside of register 0x00
  Wire.beginTransmission(I2C_Add);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(I2C_Add);
  Wire.requestFrom(I2C_Add,1);
  if(Wire.available()){
    Data=(uint8_t)Wire.read();
  } else {
    Data=0;
  }
  Wire.endTransmission();
  return Data;
}
