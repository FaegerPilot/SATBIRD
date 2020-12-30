//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include "FlightComputer.h"

  Servo _Servo[7];              // Erzeugen von 7 Servo Objekten in einem Array
  
//---------------------------------------------------------------------------------------------------
// FlightComputer Constructor
//---------------------------------------------------------------------------------------------------

FlightComputer::FlightComputer(){
    // Leerer Constructor ist nicht empfohlen, doch den Compiler mag ihn. :)
}

//---------------------------------------------------------------------------------------------------
// FlightComputer Main Setup Fuctions
//---------------------------------------------------------------------------------------------------

void FlightComputer::initiallize(){   // Initiallisieren der In- und Outputs
  
  for (int i = 0; i < 7; i++){
    _Servo[i].attach(PWM_Output_Pin[i]);
  }
  
 if(false){ //false wenn mit UNO gebootet
  for (int Pin = 0; Pin < 12; Pin++){   //Initillisiert alle PWM Inputs
    pinMode(PWM_Input_Pin[Pin], INPUT);
  }
 }
  for (int Pin = 0; Pin < 4; Pin++){   //Initiallisiert alle Digitalen Outputs
    pinMode(Digital_Output_Pin[Pin], OUTPUT);
  }
}

//----------------------------------------------------------------------------------------------------
// FlightComputer Main Loop Fuctions
//----------------------------------------------------------------------------------------------------

void FlightComputer::cycle(){   // Sammelfunktion für alle Funktionen, welche im Zyklus abgearbeitet werden müssen (Reihenfolge beachten) 
  ControlViaSerial = bool(SerialDataIn[2]);                     // Umschaltung HMI und Empfänger
  FlightComputer::timeHandler();                                // Zeiten generieren
  bool FailSave = FlightComputer::FailSave();                   // Failsaife erkennung (Zb zuhohe Zykluszeit)
  FlightComputer::handlePWMInput(true);                         // Einlesen der PWM Signale (Mit Priorisierung)
  FlightComputer::getAnalogInput();                             // Lesen der Analog Inputs
  FlightComputer::setFlightMode(SerialDataIn[33],FailSave);     // Setzen des Flugmodis
  FlightComputer::a_thrSystem();                                // Motor Steuerung
  FlightComputer::controlFlapsAndGear();                        // Klappen und Fahrwerksteuerung
  FlightComputer::controlLights();                              // Lichtsteuerung
  FlightComputer::setPWMOutput();                               // Steuern der Servos / ESCs
  FlightComputer::batteryStatus();                              // Batterie Status 
  FlightComputer::warnings();                                   // Warnings überprüfen / generieren
  FlightComputer::comHMI_FC();                                  // Sammelfunktion für alle Kommunikationsfunktionen
}

void FlightComputer::handlePWMInput(bool Prio){            // Bei prio = True werden nur alle notwendigen Pins gehandelt, bei false alle Zusammen.
  if(Prio){
    for (int Pin = 0; Pin <= 4; Pin++){
    PWM_Input_Value[Pin] = FlightComputer::getPWMInput(PWM_Input_Pin[Pin], PWM_Input_Impluse_Max[Pin], PWM_Input_Impluse_Min[Pin], PWM_Input_Max[Pin], PWM_Input_Min[Pin]);  //Ruft die getInput Funktion auf mit Pin dem Max und Min Wert und schreibt den Return wert in ein Array
    }
  }
  else{
    for (int Pin = 0; Pin <= 11; Pin++){
    PWM_Input_Value[Pin] = FlightComputer::getPWMInput(PWM_Input_Pin[Pin], PWM_Input_Impluse_Max[Pin], PWM_Input_Impluse_Min[Pin], PWM_Input_Max[Pin], PWM_Input_Min[Pin]);  
    }
  }
}

int FlightComputer::getPWMInput(int Pin,int In_Max,int In_Min, int Out_Max, int Out_Min){    // PWM Input Pin und Bereich zum Map();
    unsigned long duration = pulseIn(Pin,HIGH,MAX_PWM_INPUT_THRESHOLD);
    duration = constrain(duration,In_Min,In_Max);
    int value;
    
    if(Out_Max<=10) // Wenn der Max Wert unnter 10 ist, wird dieser Block ausgeführt der die Schalter "Mapt". Map() ist dabei zu ungenau
     int sector=(In_Max-In_Min)/(Out_Max+1)+1;
     for(int i=0; i<=Out_Max; i++){
       int SectorBottom = sector*i+In_Min;
       int SectorTop    = sector*(i+1)+In_Min;

       if(duration>=SectorBottom && duration<SectorTop){
        value = i;
        break;
      }
     }
    }
    
    else{
      value = map(duration,In_Min,In_Max,Out_Min,Out_Max);
    }
    value = constrain(value,Out_Min,Out_Max);
 return value;
}

void FlightComputer::getAnalogInput(){
  Analog_Input_Value[0] = float(analogRead(Analog_Input_Pin[0]))/205.0;
  Analog_Input_Value[1] = float(analogRead(Analog_Input_Pin[1]))/205.0;
}

void FlightComputer::setPWMOutput(){
  for (int i = 0; i < 7; i++){
    PWM_Output_Value[i] += PWM_Output_Offset[i];
    _Servo[i].write(PWM_Output_Value[i]);
  }
}

void FlightComputer::getSensorData(){

}

void FlightComputer::compareSensorData(){

}

void FlightComputer::setFlightMode(int FlightMode, bool FailSave){
 if(FailSave){            // Bei aktievem Failsaife kann nur im Manuel geflogen werden
  FlightMode=0;
 }
 switch(FlightMode){
  case 0:
  FlightComputer::modeManual();
  break;
  case 1:
  FlightComputer::modeCWS();
  break;
  case 2:
  FlightComputer::modeCMD();
  break;
  case 3:
  FlightComputer::modeCMD(); // RTB Funkiton hier einfügen
  break;
 }
}

bool FlightComputer::FailSave(){    // Überprüfung der Zyklus Zeit / True bei Überschreitung des Thresholds
  if(cycleTime >= MAX_CYCLE_TIME){
    return true;
  }else{
    return false;
  }
}
void FlightComputer::modeManual(){
  if(ControlViaSerial){           // Steuerung über HMI = Bool Werte der Tasten zu Ausgabewerten übertragen
    if(SerialDataIn[15]){
      PWM_Output_Value[1] = 60;
      PWM_Output_Value[3] = 100;
    }else if(SerialDataIn[16]){
      PWM_Output_Value[1] = 120;
      PWM_Output_Value[3] = 80;
    }else{
      PWM_Output_Value[1] = 90;
      PWM_Output_Value[3] = 90;
    }
    if(SerialDataIn[13]){
      PWM_Output_Value[2] = 120;
    }else if(SerialDataIn[14]){
      PWM_Output_Value[2] = 60; 
    }else{
      PWM_Output_Value[2] = 90;
    }
  }else{
    PWM_Output_Value[1] = 90;
    PWM_Output_Value[2] = 90;
    PWM_Output_Value[3] = 90;
  }
}

void FlightComputer::modeCWS(){
    PWM_Output_Value[1] = 90;
    PWM_Output_Value[2] = 90;
    PWM_Output_Value[3] = 90;
}

void FlightComputer::modeCMD(){
    PWM_Output_Value[1] = 90;
    PWM_Output_Value[2] = 90;
    PWM_Output_Value[3] = 90;
}

void FlightComputer::modeCMD3AxisControl(){

}

int FlightComputer::modeCMD_HDG_GPS(){

}

int FlightComputer::modeCMD_HoldingRoute(){

}

int FlightComputer::a_thrSystem(){
 if(ControlViaSerial){
  if(SerialDataIn[10]){
    PWM_Output_Value[0] = 0;
  }else if(SerialDataIn[11]){
    PWM_Output_Value[0] += 5;
  }else if(SerialDataIn[12]){
    PWM_Output_Value[0] -= 5;
  }
  PWM_Output_Value[0] = constrain(PWM_Output_Value[0],0,180);
 }else{
  PWM_Output_Value[0] = 0;
 }
}

void FlightComputer::getWaypoint(){

}

void FlightComputer::setWaypoint(){

}

void FlightComputer::countFTC(){

}

void FlightComputer::saveFTC(){

}

void FlightComputer::setCalibration(){

}

int FlightComputer::pidThrottle(){

}

int FlightComputer::pidPitch(){

}
int FlightComputer::pidRoll(){

}

int FlightComputer::pidYaw(){

}

void FlightComputer::controlLights(){
  if(ControlViaSerial){
    if(millis()-oldMillis_LightFucition > 500){
      Digital_Output_State[1] = !Digital_Output_State[1];
    } 
    if(millis()-oldMillis_LightFucition > 1000){
      Digital_Output_State[0] = !Digital_Output_State[0];
      Digital_Output_State[1] = !Digital_Output_State[1];
      oldMillis_LightFucition = millis(); 
    }
    Digital_Output_State[0] = Digital_Output_State[0] && bool(SerialDataIn[25]);
    digitalWrite(Digital_Output_Pin[0],Digital_Output_State[0]);
    digitalWrite(Digital_Output_Pin[2],SerialDataIn[26]);
    Digital_Output_State[1] = Digital_Output_State[1] &&  bool(SerialDataIn[27]);
    digitalWrite(Digital_Output_Pin[1],Digital_Output_State[1]);
    digitalWrite(Digital_Output_Pin[3],SerialDataIn[28]);
  }
}

void FlightComputer::controlFlapsAndGear(){
 if(ControlViaSerial){
   byte FlapControlValue[] = {40,60,80,120};
   byte FlapPosition = SerialDataIn[24];
   PWM_Output_Value[5] = FlapControlValue[FlapPosition];
   if(SerialDataIn[20]){
    PWM_Output_Value[6] = 40;
   }else{
   PWM_Output_Value[6] = 140;
  }
 }
}

void FlightComputer::comp_FilterAltitude(){

}

void FlightComputer::comp_Filter3Axis(){

}

void FlightComputer::comHMI_FC(){   //Sammelfunktion für Serielle Kommunikation
  FlightComputer::reciveSerialData();
  FlightComputer::comStatusHMI_FC();
  FlightComputer::sendSerialData();
}
void FlightComputer::reciveSerialData(){
 SerialBuffer = Serial.available();
 if(SerialBuffer%2==0){
  for(int i = 0; i<SerialBuffer; i++){
   int inByte(Serial.read());
   if(i%2==0){
    SerialDataAdress = inByte;
   }else{
    SerialDataIn[SerialDataAdress] = inByte;
   }
  }
 }  
}

void FlightComputer::sendSerialData(){
  for(int i = 0;  i < SerialLenghtDataArray ; i++){
    if(SerialDataOut[i] != SerialDataOutOld[i]){
     Serial.write(i);
     Serial.write(SerialDataOut[i]);
    SerialDataOutOld[i] = SerialDataOut[i];
    }
  } 
}

void FlightComputer::comParseDataOut(){

}

void FlightComputer::comParseDataIn(){

}

void FlightComputer::comInt2Byte(int Value, byte AdressArray, byte ByteArray[]){
  
}

int FlightComputer::comByte2Int(byte AdressArray , byte ByteArray[]){
  
}

void FlightComputer::comStatusHMI_FC(){
  SerialDataOut[0] = SerialDataIn[0];
  SerialDataOut[1] = SerialDataIn[1];
}

void FlightComputer::batteryStatus(){
  SerialDataOut[41] = byte(Analog_Input_Value[0]*30.0);               // Akku Spannung
  SerialDataOut[41] = constrain(SerialDataOut[41],0,150);             // Begrenzung um im HMI kein Overflow zu haben
  if(second != oldsecond){
      StromDifferenz = Analog_Input_Value[1]*600/36;                 // Stromdifferenz pro sekunde 0,0mAh
      StromZaehler += StromDifferenz;                                // Akku Strom
    }
  StromZaehler = constrain(StromZaehler,0,200000);
  SerialDataOut[45] =  byte(StromZaehler/1000);
}

void FlightComputer::timeHandler(){
  oldsecond = second;
  cycleTime = millis()-oldmillis;
  oldmillis = millis();
  second = int(millis()/1000);
}

void FlightComputer::warnings(){
  if(SerialDataOut[41] < 110){
  SerialDataOut[62] = 1;  
  }else{
  SerialDataOut[62] = 0;
  }
  
}
// END OF FILE
