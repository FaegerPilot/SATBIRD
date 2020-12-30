//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

//------------------------------------------------------------
//BMP180 Class
//------------------------------------------------------------

class BMP180{
  public:
  void      initiallize();
  void      cycle();

  void      setCofiguration();
  uint16_t  getStatus();
  void      setCalibration();
  void      getCalibration();
  void      readMeauserment();
  void      convertMeauserment();

  void      setRegister(uint8_t reg, uint8_t value);
  byte      getRegister(uint8_t reg);

  //Speziall Funktion für BMP180
  BMP180(uint8_t adress);
  void      setQNH();
  int       getAltitude();
  
  private:
  uint8_t   _address;         // I2C Address 
  uint16_t  _status;          //Sensor Status
  uint16_t  _configuration;   //Sensor Configuration

  //Speziall Variabel für BMP180
  int       _QNH;
  int       _Altitude;
  float     _raw2alt;
};
// END OF FILE
