//Name: General Purpuse Stream Protocol
//Date: 10 June 2016
//Modiefed: 17 June 2016
//Auther: Jeroen Ruiten

//maak configuratie voor including GPSTP, kan ook weggelaten worden. Dit zijn de standaard waardes

//hoeveel handlers er aangemeld kunnen worden
#define MAX_HANDLER_CALLBACK 20
//hoe groot de payload maximaal mag zijn
#define MAX_PAYLOAD 100
//hoe lang er geen bericht kan worden ontvangen voordat er een fout komt (in ms)
#define TIMEOUT 150

//Voor AVR boorden gebruik "GPSTP_OTHER.h" voor alle andere boorden
#include "GPSTP_AVR.h"

GPSTP gpstp;

const uint32_t serialSpeed = 9600;
const uint8_t onboardLED = 13;

boolean ledState = false;

//wordt uitgevoerd als er een bericht binnenkomt voor poort 1
//Geef true terug als alles goed is gegaan en false als dat niet zo was, zo kan GPSTP dit terug koppelen
boolean ledEvent(uint8_t payload[], uint16_t payloadLenght){
  if(ledState){
    ledState = false;
  }
  else{
    ledState = true;
  }

  digitalWrite(onboardLED, ledState);
  return true;
}

void setup(){
  pinMode(onboardLED, OUTPUT);
  digitalWrite(onboardLED, LOW);

  //gpstp heeft serial nodig, dus start deze op. Deze serial NIET zelf gebruiken
  Serial.begin(serialSpeed);
  //geef serial aan gpstp
  gpstp.begin(Serial);

  //voeg functie ledEvent toe als handler die luisterd naar poort 1
  gpstp.addHandler(ledEvent, 1);
}

void loop(){
  //De loop moet vaak genoeg aangeroepen worden om alles soepel te laten werken
  //dit is alles wat je hoeft te doen, als je berichten wilt ontvangen of versturen
  gpstp.loop(); //MOET erin zitten, dit is de runtime
}
