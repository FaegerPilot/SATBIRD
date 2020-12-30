//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

//------------------------------------------------------------
//HMC5883L Class
//------------------------------------------------------------

class HMC5883L{
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

  //Speziall Funktion für HMC5883L
  HMC5883L(uint8_t adress);
  int       getHeading();
  
  private:
  uint8_t   _address;         // I2C Address 
  uint16_t  _status;          //Sensor Status
  uint16_t  _configuration;   //Sensor Configuration

  //Speziall Variabel für HMC5883L
  uint32_t   _Compass[3] = {};
  uint32_t   _CompassCalibration[3] = {};  
};
// END OF FILE
