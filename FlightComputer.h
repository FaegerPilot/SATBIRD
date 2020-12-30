//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

//-----------------------------------------------------
// Defines for FlightComputer Class
//-----------------------------------------------------
#define MAX_CYCLE_TIME 1000
#define MAX_PWM_INPUT_THRESHOLD 100000

// I/O's
#define I_THROTTLE              22
#define I_ELEVATOR              23
#define I_AILERON               24
#define I_RUDDER                25
#define I_SET_FLIGHT_MODE       26
#define I_LIGHT_MODE            27
#define I_RTB_AND_CALLIBRATION  28
#define I_FLAPS                 29
#define I_SET_WAYPOINT          30
#define I_SET_ALT               31
#define I_SET_HDG               32
#define I_SET_ATHR              33

#define I_BAT_VOLTAGE           A0
#define I_BAT_CURRENT           A1

#define O_THROTTLE              2
#define O_AILREON               3
#define O_ELEVATOR              4
#define O_RUDDER                5
#define O_BOW_WHEEL_STEERING    6
#define O_FLAPS                 7
#define O_LANDING_GEAR          8

#define O_BEACON_LIGHT          9
#define O_STROBE_LIGHT          10
#define O_NAV_LIGHT             11
#define O_LNG_LIGHT             12

//EEPROM Adressen
#define FTC_REGISTER_SEK		0x24
#define FTC_REGISTER_MIN		0x25
#define FTC_REGISTER_STD		0x26

//-----------------------------------------------------
// FlightComputer Class
//-----------------------------------------------------
class FlightComputer{
  public:

  FlightComputer();
  
  void initiallize();
  
  void    cycle();
  int     getPWMInput(int Pin,int In_Max,int In_Min, int Out_Max, int Out_Min);
  void    getAnalogInput();
  void    handlePWMInput(bool Prio);
  void    setPWMOutput();
  void    getSensorData();
  void    compareSensorData();
  
  void    setFlightMode(int FlightMode, bool FailSave);
  bool    FailSave();
  void    modeManual();
  void    modeCWS();
  void    modeCMD();
  void    modeCMD3AxisControl();
  int     modeCMD_HDG_GPS();
  int     modeCMD_HoldingRoute();
  int     a_thrSystem();
  
  void    getWaypoint();
  void    setWaypoint();
  void    setRTBPoint();
  void    countFTC();
  void    saveFTC();
  void    setCalibration();

  int     pidThrottle();
  int	    pidPitch();
  int     pidRoll();
  int     pidYaw();

  void    controlLights();
  void    controlFlapsAndGear();

  void    comp_FilterAltitude();
  void    comp_Filter3Axis();
  
  void    comHMI_FC();
  void    reciveSerialData();
  void    sendSerialData();
  void    comParseDataOut();
  void    comParseDataIn();
  void    comInt2Byte(int Value, byte AdressArray, byte ByteArray[]);
  int     comByte2Int(byte AdressArray, byte ByteArray[]);
  void    comStatusHMI_FC();
  
  void    batteryStatus();
  void    timeHandler();
  void    warnings();
  
  private:

  bool ControlViaSerial = true;   // Steuerung über HMI

  unsigned long oldmillis;        // Variablen von/für Timehandler
  int cycleTime = 0;
  int second = 0;
  int oldsecond = 0;
    
  int PWM_Input_Pin[12]           = {I_THROTTLE,I_AILERON,I_ELEVATOR,I_RUDDER,I_SET_FLIGHT_MODE,I_LIGHT_MODE,I_RTB_AND_CALLIBRATION,I_FLAPS,I_SET_WAYPOINT,I_SET_ALT,I_SET_HDG,I_SET_ATHR}; //PWM Input Adressen in Array gespeichert
  int PWM_Input_Impluse_Max[12]   = {1883,1885,1886,1886,1880,1883,1885,1886,1886,1886,1886,1886};     //Max Impulsdauer vom Empfänger
  int PWM_Input_Impluse_Min[12]   = {1093,1092,1093,1093,1093,1093,1092,1093,1093,1093,1093,1092};      //Min Impulsdauer vom Empfänger
  int PWM_Input_Max[12]           = {100,100,100,100,2,1,1,1,1,1,1,1};             //Max Wert des verarbeiteten Kanal's
  int PWM_Input_Min[12]           = {0,-100,-100,-100,0,0,0,0,0,0,0,0};            //Min Wert des verarbeiteten Kanal's
  int PWM_Input_Value[12]         = {0,0,0,0,0,0,0,0,0,0,0,0};                     //Ausgabe Wert Input Kanal

  int Analog_Input_Pin[2]         = {I_BAT_VOLTAGE,I_BAT_CURRENT};   //Analoge Inputs
  float Analog_Input_Value[2]     = {0,0};                           //Analoger Input Wert
     
  int PWM_Output_Pin[7]           = {O_THROTTLE,O_AILREON,O_ELEVATOR,O_RUDDER,O_BOW_WHEEL_STEERING,O_FLAPS,O_LANDING_GEAR}; //PWM Output Adressen in Array gespechert     
  int PWM_Output_Offset[7]        = {0,0,10,2,0,0,0};                //Offset resp. Trimmung für Servos
  int PWM_Output_Value[7]         = {0,90,90,90,90,60,40};                //PWM Wert zur Ausgabe

  int  Digital_Output_Pin[4]      = {O_BEACON_LIGHT,O_STROBE_LIGHT,O_NAV_LIGHT,O_LNG_LIGHT}; //Digitale Pins
  bool Digital_Output_State[4]    = {true,true,true,true};        //Digitaler Ausgabe Wert
  unsigned long oldMillis_LightFucition = 0;

  int SerialDataOut[96];          // Ausgangs Array Serial
  int SerialDataOutOld[96];       // Ausgangs Array Serial um gesendet Werte zu speichern und zu vergleichen
  int SerialDataIn[96];           // Eingangs Array Serial

  byte SerialLenghtDataArray = 95;
  byte SerialBuffer = 0;          // Anzahl Byte zum Buffern befor Daten gelesen werden
  int SerialDataAdress = 0;

  uint32_t StromZaehler = 0;
  uint8_t StromDifferenz = 0;
};
//END OF FILE
