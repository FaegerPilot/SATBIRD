//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RunningMedian.h>

//------------------------------------------------------------
//MPU6050 Class
//------------------------------------------------------------

class MPU6050{
 public:
  void    initiallize();
  void    cycle();

  void     setCofiguration();
  uint16_t getStatus();
  void     setCalibration();
  void     getCalibration();
  void     readMeauserment();
  void     convertMeauserment();

  void    setRegister(uint8_t reg, uint8_t value);
  byte    getRegister(uint8_t reg);

  //Speziall Funktion für MPU6050
  MPU6050(uint8_t address);
  bool     wakeup();
 
  float    getAccelX()      { return _Accel[0]; };
  float    getAccelY()      { return _Accel[1]; };
  float    getAccelZ()      { return _Accel[2]; };
  
  float    getGyroX()       { return _Gyro[0]; };
  float    getGyroY()       { return _Gyro[1]; };
  float    getGyroZ()       { return _Gyro[2]; };

  float    getPitch()       { return _pitch; };
  float    getRoll()        { return _roll; };
  float    getYaw()         { return _yaw; };

  int	   getTemperature()   { return _temperature; };
  
 private:
  uint8_t   _address;         // I2C Address 
  uint16_t  _status;          //Sensor Status
  uint16_t  _configuration;   //Sensor Configuration

  //Speziall Variabel für MPU6050
  float    _duration;                              // time intervall to sum up Gyro Data
  float    _gyro_last_update = 0;                  // Stores the last Cycle Time
                                                   // 0=X,1=Y,2=Z
  long    _AccelCalibration[3]= {0, 0, 0};         // Stores the calibration Values (long because Data Overflow)
  int     _GyroCalibration[3] = {0, 0, 0};         // Stores the calibration Values
  
  long     _AccelAngle[2] = {0, 0};                // Calculatet Angles from Acceleration
  long     _Accel[3]      = {0, 0, 0};             // Accelerometer processed
  float    _GyroAngle[3]  = {0, 0, 0};;            // Calculatet Angels from Gyroscope
  float    _Gyro[3]       = {0, 0, 0};             // Gyro processed.
  int      _pitch, _roll, _yaw;                    // Filtert Angels

  float    _temperature = 0;
  
  float    _raw2dps = 1.0/131.0;
  float    _rad2degress = 180.0 / PI;
  float    _raw2g = 1.0/8192.0;                   // raw data to gravity g's
};
// END OF FILE
