//Name: General Purpuse Stream Protocol
//Date: 10 June 2016
//Modiefed: 17 June 2016
//Auther: Jeroen Ruiten

//TODO
// - Het testen van een binnengekregen payload
// - het mogelijk maken van het versturen van een payload
// - Voor het versturen rekening houden of er een ACK binnen is gekomen
// - opnieuw versturen van bericht als NAK binnen is gekomen
// - Een manier verzinnen om NAK en ACK te schijden van data
// - kleine berichten buffer voor versturen

#include "GPSTP.h"
#include "Arduino.h"
//######################//
//## Public functions ##//
//######################//

//constructer
GPSTP::GPSTP() {
  _ownHandlerManager = false;
  _handlerCallbackCount = 0;
  _reset();
}

//Plaats Handler in array, de handler moet een functie boolean(byte payload, uint16_t payloadLenght)
//addHandler(function, port)
void GPSTP::addHandler(boolean (*handlerCallback)(uint8_t payload[], uint16_t payloadLenght), uint16_t handlerPort) {
  if (_handlerCallbackCount < MAX_HANDLER_CALLBACK) {
    _handlerCallback[_handlerCallbackCount] = handlerCallback;
    _handlerCallbackPort[_handlerCallbackCount] = handlerPort;
    _handlerCallbackCount++;
  }
}

//Main loop, wordt door gebruiker in loop aangeroepen. Voert 1 taak per keer uit, mag geen grote loops hebben, verdelen over meerdere runs
void GPSTP::loop() {
  //snel uitvoerende code, snel weer klaar zijn, 1 handeling per keer
  //Loop voor de verschilende staten van het afhandelen van een frame
  switch (_state) {
    //begin van frame
    case EXPECT_HEADER:
      if (_serial->available() > 0) {
        _expectHeader();
      }
      break;
    //Ontvang de daatwerkelijke header
    case RECIEVE_HEADER:
      if (_serial->available() > 0) {
        _startTime = millis();
        _recieveHeader();
      }
      //no serial data, but expected. Timeout countdown
      else {
        _timeOutHandler();
      }
      break;
    //verwerk header
    case PROCESS_HEADER:
      _processHeader();
      break;
    //ontvang payload (hier draait het om)
    case RECIEVE_PAYLOAD:
      if (_serial->available() > 0) {
        _startTime = millis();
        _recievePayload();
      }
      //no serial data, but expected. Timeout countdown
      else {
        _timeOutHandler();
      }
      break;
    //verwerk payload, het aan de handler geven van de payload
    case PROCESS_PAYLOAD:
      _processPayload();
      break;
    //ontvang de 16 bit (als 2 bytes) CRC
    case RECIEVE_CRC:
      if (_serial->available() > 0) {
        _startTime = millis();
        _recieveCRC();
      }
      //no serial data, but expected. Timeout countdown
      else {
        _timeOutHandler();
      }
      break;
    //bereken en controlleer de CRC
    case PROCESS_CRC:
      _processCRC();
      break;
  }
  //Niks hier plaatsen, alles in functies die worden aangeroepen boven, in de state switch
}

//laat de gebruiker de stream, wel Serial, bepalen
void GPSTP::begin(Stream &serial) {
  _serial = &serial;
}

//zelfde als begin, maar misschien willen ze setSerial gebruiken ipv begin
void GPSTP::setSerial(Stream &serial) {
  _serial = &serial;
}

//standaard wordt de ingebouwde handlerManager gebruikt, maar je kan er zelf een aanmelden die in jouw programma zit
//meld eigen handlerManager aan. De handlerManager in de sckatsh wordt aangeroepen ipv de aangemelde handelers
//De handlerManager moet er zo uitzien boolean functie(byte payload, uint16_t payloadLenght, uint16_t port)
//hij geeft true terug als het goed is gegaan en anders false
void GPSTP::setHandlerManager(boolean (*handlerManagerCallback)(uint8_t payload[], uint16_t payloadLenght, uint16_t handlerPort)) {
  _handlerManagerCallback = handlerManagerCallback;
  _ownHandlerManager = true;
}

//sendData for all data types
void GPSTP::sendData(uint8_t data[], uint16_t handlerPort){
  
}

//overload sendData, make everything so that it is a byte

//#######################//
//## Private functions ##//
//#######################//

//assamble bytes back into int16
uint16_t GPSTP::_byteToInt16(uint8_t first, uint8_t second) {
  return second | (uint16_t)first << 8;
}

//resets all vars, calls aftere proccesing or on error
void GPSTP::_reset() {
  _state = EXPECT_HEADER;
  _preHeader = PRE_HEADER_1;
  _crcCalculaded = 0xFFFF;
  _cursor = 0;
  _payloadLen = 0;
}

//verstuur NAK, met foutcode (1 char) en reset
void GPSTP::_errorHandler(_Error error) {
  //schrijf 3 byte watermark
  _serial->write(_watermark, 3);
  //scrhijf ALTIJD watermark voor command
  _serial->write(_NAK); //gaat altijd samen met 1 byte als error code
  switch (error) { //TODO, verzin een vaste set van errors, en verzin er juiste waardes voor
    case TIMED_OUT:
      _serial->print('0');
      break;
    case PAYLOAD_TOO_LONG:
      _serial->print('1');
      break;
    case PAYLOAD_INVALLID_LENGTH:
      _serial->print('2');
      break;
    case MORE_THEN_MAX_HANDLERS:
      _serial->print('3');
      break;
    case NO_HANDLER_FOUND:
      _serial->print('4');
      break;;
    case CRC_ERROR:
      _serial->print('5');
      break;
    case HANDLER_ERROR:
      _serial->print('6');
      break;
    case HEADER_ERROR:
      _serial->print("7");
      break;
  }
  _reset();
}

//verstuur ACK en reset
void GPSTP::_successHandler() {
  //schrijf 3 byte watermark weg, voor een command
  _serial->write(_watermark, 3);
  //Schrijf ALTIJD watermark voor command!
  _serial->write(_ACK); //staat los, bevestigt dat alles goed is gegaan
  _reset();
}

//bekijk of de tijd om is, zo ja, error(time_out)
void GPSTP::_timeOutHandler() {
  if (_startTime + TIMEOUT < millis()) {
    _errorHandler(TIMED_OUT);
  }
}

//ALLEEN aanroepen als er meer dan of gelijk aan 3 bytes in de _serial buffer zitten
//geeft true terug als watermark overeen komt
//geeft false terug als dat niet zo is, of er teweining in de buffer zit om dat te controleren (watermark is alijd 3 bytes)
boolean GPSTP::_checkWatermark() {
  //er is genoeg aanwezig in de buffer
  if (_serial->available() <= 3) {
    if (_serial->read() != _watermark[0]) {
      return false;
    }
    if (_serial->read() != _watermark[1]) {
      return false;
    }
    if (_serial->read() != _watermark[2]) {
      return false;
    }
    //watermark komt overeen, de volgende byte is een vallid command byte
    return true;
  }
  //niet genoeg
  else {
    return false;
  }
}

//NEW
//controlleerd of het begin van een frame er is, zo ja, begin met ontvangen van frame
void GPSTP::_expectHeader() {
  if (_checkWatermark()) {
    if (_serial->read() == _SOH) {
      _header[0] = _SOH;
      _cursor = 1;
      _state = RECIEVE_HEADER;
      _startTime = millis();
    }
    else {
      _errorHandler(HEADER_ERROR);
    }
  }
}

/*
//OLD
//stage 1: look for stage 1 byte, is found change to stage 3, if not, dump byte
//stage 2: look for stage 2 byte, is found change to stage 3, if not, error(header_error)
//stage 3: look for SOH, if so change state to RECIEVE_HEADER and store SOH if not, error(header_error)
void GPSTP::_expectHeader() {
  switch (_preHeader) {
    case PRE_HEADER_1:
      if (_serial->read() == _preHeaderData[0]) {
        _preHeader = PRE_HEADER_2;
      }
      break;
    case PRE_HEADER_2:
      if (_serial->read() == _preHeaderData[1]) {
        _preHeader = SOH;
      }
      else {
        _errorHandler(HEADER_ERROR);
      }
      break;
    case SOH:
      if (_serial->read() == _SOH) {
        _header[0] = _SOH;
        _cursor = 1;
        _state = RECIEVE_HEADER;
        _startTime = millis();
      }
      else {
        _errorHandler(HEADER_ERROR);
      }
      break;
  }
}
*/

//ontvang header, als cursor op header lenght is, chage state to PROCESS_HEADER
void GPSTP::_recieveHeader() {
  _header[_cursor] = _serial->read();
  _cursor++;

  if (_cursor == HEADER_LENGHT) {
    _state = PROCESS_HEADER;
  }
}

//bekijk of wat er in de header staat, ook klopt
//zo ja, change state to RECIEVE_PAYLOAD
//zo nee, om 2 redenen (handler niet aangemeld of payload te lang) geef error
void GPSTP::_processHeader() {
  //extract de waardes uit de header
  _payloadLen = _byteToInt16(_header[1], _header[2]);
  _handlerPort = _byteToInt16(_header[3], _header[4]);

  //check of handler is aangemeld
  if (!_handlerManager(false)) {
    //_handlerManager geeft al de juiste error, hoeft niet 2 keer
    return;
  }

  //payload len mag niet groter zijn dan MAX_PAYLOAD
  if (_payloadLen > MAX_PAYLOAD) {
    _errorHandler(PAYLOAD_TOO_LONG);
    return;
  }

  //niets mis, op naar de volgende state
  _state = RECIEVE_PAYLOAD;
  _cursor = 0;
}

//lees payload uit, in buffer
//cursor is op 0 gezet
//als er geen payload is, zet state direct op RECIEVE_CRC
//anders net zo lang blijven lezen totdat payloadLen is bereikt en dan state op RECIEVE_CRC
void GPSTP::_recievePayload() {
  if (_payloadLen == 0) {
    _state = RECIEVE_CRC;
    return;
  }

  //lees volgende payload byte
  _payload[_cursor] = _serial->read();
  _cursor++;

  //payload binnen? ja: zet state op RECIEVE_CRC
  if (_cursor == _payloadLen) {
    _state = RECIEVE_CRC;
    _cursor = 0;
  }
}

//lees 2 bytes CRC
//cursor is op 0 gezet
//als alles gelezen is wordt state op PROCESS_CRC gezet
void GPSTP::_recieveCRC() {
  //lees volgende CRC
  _crc[_cursor] = _serial->read();
  _cursor++;

  //is alle CRC gelezen? ja: state wordt PROCESS_CRC
  if (_cursor == CRC_LENGHT) {
    _state = PROCESS_CRC;
    _cursor = 0;
    _crcRecieved = _byteToInt16(_crc[0], _crc[1]);
  }
}

//controlleer de CRC
//cursor is op 0 gezet
//process eerst de header
//dan de payload, als die er is
//dan de ontvangen en berekende CRC met elkaar vegelijk
//zijn ze hetzelfde, dan state = PROCESS_PAYLOAD
//anders error
void GPSTP::_processCRC() {
  //crc of header part
  if (_cursor < HEADER_LENGHT) {
    _crcCalculaded = _crc16_update(_crcCalculaded, _header[_cursor]);
    //_serial->println("CRC Header proccesing");
  }

  //crc of payload part
  else if (_cursor < HEADER_LENGHT + _payloadLen) {
    _crcCalculaded = _crc16_update(_crcCalculaded, _payload[_cursor - HEADER_LENGHT]);
  }

  //check CRC
  else {
    if (_crcCalculaded == _crcRecieved) {
      _state = PROCESS_PAYLOAD;
    }
    else {
      _errorHandler(CRC_ERROR);
    }
  }

  //de volgende locatie, voor de volgende run
  _cursor++;
}

//bekijk of de handler goed word uitgevoerd, zo niet geef error
//zo wel: voer de successHandler uit
//dit keer wordt tijdens het controleren, direct de handler uitgevoerd en krijg je de waarde terug van de handler
void GPSTP::_processPayload() {
  //_serial->println("Processing payload");

  if (!_handlerManager(true)) {
    _errorHandler(HANDLER_ERROR);
    return;
  }
  _successHandler();
}

//bekijk of er een handler is geregistreed op de handler port, zet getallen om in simpele booleans
//elk getal is true
//-1 is false
boolean GPSTP::_isHandlerCallback(uint16_t handlerPort) {
  if (_getHandlerCallback(handlerPort) == -1) {
    return false;
  }
  return true;
}

//return location van handlerCallback, -1 als niet gevonden
int32_t GPSTP::_getHandlerCallback(uint16_t handlerPort) {
  //loop de array na met de poorten erin, of de ongevraagte poort ertussen zit, de locatie komt 1:1 overeen met de array waar de handlers in zitten
  for (uint16_t i = 0; i < _handlerCallbackCount; i++) {
    if (_handlerCallbackPort[i] == handlerPort) {
      return i;
    }
  }

  //not found
  return -1;
}

//roep handler aan, of kijk of deze bestaat
//ALS gebruiker een eigen handlerManager heeft voer die uit als execute is true geef anders true terug
//Als execute false is, kijk of de handler bestaat, maar doe verder niks
//Als handler niet bestaat, geef false en error
//Als handler wel bestaat, return true
//Als execute true is, doe het zelfde, maar voer nu wel de handler uit
boolean GPSTP::_handlerManager(boolean execute) {
  //gebruiker heeft eigen manager aangemeld, doe verder niks
  if (_ownHandlerManager) {
    if (execute) {
      return _handlerManagerCallback(_payload, _payloadLen, _handlerPort);
    }
    else {
      return true;
    }
  }

  //GPSTP manager (ingebouwde manager)
  //bestaat handler?
  if (_isHandlerCallback(_handlerPort)) {
    //uitvoeren?
    if (execute) {
      return _handlerCallback[_getHandlerCallback(_handlerPort)](_payload, _payloadLen);
    }
    return true;
  }
  //bestaat niet
  else {
    _errorHandler(NO_HANDLER_FOUND);
    return false;
  }
}

//Hulp functie CRC16 update voor NIET AVR boorden
#ifndef _crc16_update
uint16_t GPSTP::_crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; ++i) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    }
    else {
      crc = (crc >> 1);
    }
  }
  return crc;
}
#endif
