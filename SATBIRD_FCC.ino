//------------------------------------------------------------
// SAT Bird Project
// Christoph Messerli
//------------------------------------------------------------

#include "FlightComputer.h" // FlightComputer einbinden

FlightComputer FCS;         // FlightComputer Obiekt erzeugen

void setup() {
  Serial.begin(9600);       // Kommunikation starten
  FCS.initiallize();        // Initiallisierungs Funktion FlightComputer
}

void loop() {  
  FCS.cycle();              // Sammelfunktion für FlighComputer Zyklus
} 
//END OF FILE
