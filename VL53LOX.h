//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

//------------------------------------------------------------
//VL53LOX Class
//------------------------------------------------------------

class VL53LOX{
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

  //Speziall Funktion für VL53LOX
  VL53LOX(uint8_t adress);
  int       getDistance();
  bool      getAirborneStatus();
  
  private:
  uint8_t   _address;         // I2C Address 
  uint16_t  _status;          //Sensor Status
  uint16_t  _configuration;   //Sensor Configuration

  //Speziall Variabel für VL53LOX
  uint32_t _DistanceRaw;
  int      _Distance;
  bool     _isAirborne;
};
// END OF FILE
