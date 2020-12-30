//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include "MPU6050.h"

//-----------------------------------------------------
// Wichtige MPU6050 Register Adressen / Werte
//-----------------------------------------------------

#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WAKEUP          0x00 //Clear all Bits of the Power Managment Register

RunningMedian MedianAccelX = RunningMedian(11);
RunningMedian MedianAccelY = RunningMedian(11);
RunningMedian MedianAccelZ = RunningMedian(11);

//--------------------------------------------------------------------------------------------------
// MPU6050 Class Constructor
//--------------------------------------------------------------------------------------------------

MPU6050::MPU6050(uint8_t address){              //Class Constructor
  _address = address;                       //Save I2C Adress in Class Variable

}

//---------------------------------------------------------------------------------------------------
// MPU6050 Main Setup Fuctions
//---------------------------------------------------------------------------------------------------

void MPU6050::initiallize(){
 MPU6050::wakeup();
 MPU6050::setCofiguration();
 MPU6050::getCalibration();  
}

bool MPU6050::wakeup(){                       //MPU6050 is in sleep Mode on Power-Up
  Wire.beginTransmission(_address);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(MPU6050_WAKEUP);
  Wire.endTransmission() == 0;
  delay(500);
}
void MPU6050::setCofiguration(){
    setRegister(MPU6050_ACCEL_CONFIG,B00001000); // Setting +/- 4g
  setRegister(MPU6050_GYRO_CONFIG,B00000000);  // Setting +/- 250°/s
}



void MPU6050::getCalibration(){
 
 _AccelCalibration[0] = EEPROM.read(0); //Gets alle Offset Values from EEPROM and Stores it in SRAM Variables;
 _AccelCalibration[1] = EEPROM.read(4);
 _AccelCalibration[2] = EEPROM.read(8);

 _GyroCalibration[0] = EEPROM.read(12);
 _GyroCalibration[1] = EEPROM.read(14);
 _GyroCalibration[2] = EEPROM.read(16);
}

//----------------------------------------------------------------------------------------------------
// GY-521 Main Loop Fuctions
//----------------------------------------------------------------------------------------------------
void MPU6050::cycle(){             //Main Class Fuction to Call in Main Loop() 
  MPU6050::readMeauserment();      //Reading 14 Dataregister and Store Values
  MPU6050::convertMeauserment();   //Converting Values
}

void MPU6050::readMeauserment(){
  
  Wire.beginTransmission(_address);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission() != 0);
  
  // Get the data
  Wire.requestFrom(_address, (uint8_t)14);
  // ACCELEROMETER
  _Accel[0] = ( ( ((int)Wire.read()) << 8) | Wire.read() );  // ACCEL_XOUT_H  ACCEL_XOUT_L
  _Accel[1] = ( ( ((int)Wire.read()) << 8) | Wire.read() );  // ACCEL_YOUT_H  ACCEL_YOUT_L
  _Accel[2] = ( ( ((int)Wire.read()) << 8) | Wire.read() );  // ACCEL_ZOUT_H  ACCEL_ZOUT_L
  
  // TEMPERATURE
  _temperature = ( ((int)Wire.read()) << 8) | Wire.read();  // TEMP_OUT_H Register    TEMP_OUT_L Register
  
  // GYROSCOPE
  _Gyro[0] = (( ((int)Wire.read()) << 8) | Wire.read());  // GYRO_XOUT_H Register   GYRO_XOUT_L Register
  _Gyro[1] = (( ((int)Wire.read()) << 8) | Wire.read());  // GYRO_YOUT_H Register   GYRO_YOUT_L Register
  _Gyro[2] = (( ((int)Wire.read()) << 8) | Wire.read());  // GYRO_ZOUT_H Register   GYRO_ZOUT_L Register

}

void MPU6050::convertMeauserment(){

  //time interval
  _duration = (micros() - _gyro_last_update) * 0.000001; //Cycle Time in Seconds
  _gyro_last_update = micros();
  
//Offset the Values acc. Calibration
for (int i = 0; i <= 1; i++){ 
  _Accel[i] = (_Accel[i] - _AccelCalibration[i] );  
}

  MedianAccelX.add(_Accel[0]);
  MedianAccelY.add(_Accel[1]);
  MedianAccelZ.add(_Accel[2]);
  
  _Accel[0] = MedianAccelX.getMedian();
  _Accel[1] = MedianAccelY.getMedian();
  _Accel[2] = MedianAccelZ.getMedian();

//Offset the Values acc. Calibration and convert it to "Degress/S"
 for (int i = 0; i <= 1; i++){
  _Gyro[i] = (_Gyro[i] - _GyroCalibration[i] )*_raw2dps;
 }

  _AccelAngle[0] = atan2(_Accel[1] , _Accel[2]) * _rad2degress;
  _AccelAngle[1] = atan2(-1.0*_Accel[0] , (hypot(_Accel[1],_Accel[2]))) * _rad2degress;
  
  _pitch = _AccelAngle[0] * 0.1 + (_pitch+ _Gyro[0] * _duration) * 0.9;
  _roll  = _AccelAngle[1] * 0.1 + (_roll + _Gyro[1] * _duration) * 0.9;
  _yaw   = _Gyro[2] * _duration;
 /* 
  Serial.print(_pitch);
  Serial.print('\t');
  Serial.print(_roll);
  Serial.print('\t');  
  Serial.print(_AccelAngle[0]);
  Serial.print('\t');
  Serial.print(_AccelAngle[1]);
  Serial.print('\t');
  Serial.print(_Gyro[0]);
  Serial.print('\t');
  Serial.print(_Gyro[1]);
  Serial.println();
*/
  _temperature = (_temperature + 12412.0) * 0.00294117647;  //  == /340.0  + 36.53;
}

//----------------------------------------------------------------------------------------------------------------

void MPU6050::setRegister(uint8_t RegisterAdress, uint8_t Value){
  Wire.beginTransmission(_address);
  Wire.write(RegisterAdress);
  Wire.write(Value);
  // no need to do anything if not connected.
  if (Wire.endTransmission() != 0);
}

uint8_t MPU6050::getRegister(uint8_t RegisterAdress){
  Wire.beginTransmission(_address);
  Wire.write(RegisterAdress);
  if (Wire.endTransmission() != 0);
  Wire.requestFrom(_address, (uint8_t) 1);
  uint8_t Value = Wire.read();
  return Value;
}

void MPU6050::setCalibration(){
  
 for (int i = 0; i <= 9 ; i++) {
  MPU6050::readMeauserment();

  _AccelCalibration[0] += _Accel[0];
  _AccelCalibration[1] += _Accel[1];
  _AccelCalibration[2] += _Accel[2];
  
  _GyroCalibration[0] += _Gyro[0];
  _GyroCalibration[1] += _Gyro[1];
  _GyroCalibration[2] += _Gyro[2];
}

  _AccelCalibration[0] /= 10;
  _AccelCalibration[1] /= 10;
  _AccelCalibration[2] /= 10;
  _AccelCalibration[2] -= 8192;

  _GyroCalibration[0] /= 10;
  _GyroCalibration[1] /= 10;
  _GyroCalibration[2] /= 10;
    
  EEPROM.write(0, _AccelCalibration[0] );
  EEPROM.write(4, _AccelCalibration[1] );
  EEPROM.write(8, _AccelCalibration[2] );

  EEPROM.write(12, _GyroCalibration[0]);
  EEPROM.write(14, _GyroCalibration[1]);
  EEPROM.write(16, _GyroCalibration[2]);
}
// END OF FILE
