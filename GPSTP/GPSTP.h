//Name: General Purpuse Stream Protocol
//Date: 10 June 2016
//Modiefed: 17 June 2016
//Auther: Jeroen Ruiten

//opbouw protocol:
// ; toegevoegt voor leesbaarheid, alle bytes achter elkaar sturen. Voor elk command carachter (bv SOH) moet de 3 bytes watermark verstuurd worden
// (OLD) 2bytes pre-header; SOH; 16bit payload len; 16bit handler port; payload; 16 bit checksum
// 3bytes watermark; SOH; bericht nummer(uint8_t as 1 byte); payload len(uint16_t as 2 bytes); handler port(unit16_t as 2 bytes); payload; crc16 checksum (uint16_t as 2 bytes)

#include <Arduino.h>

//Maximale invoeren direct gekoppeld aan het geheugen gebruik, als niet gedefineerd in de schetch, neem standaard waarde
#ifndef MAX_HANDLER_CALLBACK
#define MAX_HANDLER_CALLBACK 20
#endif
#ifndef MAX_PAYLOAD
#define MAX_PAYLOAD 100
#endif

//Protocol specificatie, zou niet veranderd hoeven worden
#define HEADER_LENGHT 5
#define CRC_LENGHT 2

//timeout to next byte in ms, als niet gedefineerd in de schetch, neem standaard waarde
#ifndef TIMEOUT
#define TIMEOUT 100
#endif

class GPSTP {
  public:
    //Standaard regeld GPSTP de handlers, met addHandler() 
    GPSTP();
    void begin(Stream &serial);
    void setSerial(Stream &serial);
    void addHandler(boolean (*handlerCallback)(uint8_t payload[], uint16_t payloadLenght), uint16_t handlerPort);
    void setHandlerManager(boolean (*handlerManagerCallback)(uint8_t payload[], uint16_t payloadLenght, uint16_t handlerPort));
    void loop();

    //setSendHandler; krijgt een event puls als het goed of fout is gegaan
    void setSendHandler(void (*sendHandlerCallback)(boolean successful));

    //Send data (meerdere functies, 1 naam. Allemaal hebben ze de handler port) returned void
    //Data byte array (byte array and lenght en handler poort)
    void sendData(uint8_t data[], uint16_t handlerPort);
    //Data int array (int array and lenght en handler poort)
    void sendData(int data[], uint16_t handlerPort);
    //long array
    void sendData(long data[], uint16_t handlerPort);
    //float array
    void sendData(float data[], uint16_t handlerPort);
    //dubble array
    void sendData(double data[], uint16_t handlerPort);
    //char array (only the array en handler poort)
    void sendData(char data[], uint16_t handlerPort);
    //String object (only String object en handler poort)
    void sendData(String data, uint16_t handlerPort);
    
  private:
    //enums
    enum _State {
      EXPECT_HEADER,
      RECIEVE_HEADER,
      PROCESS_HEADER,
      RECIEVE_PAYLOAD,
      PROCESS_PAYLOAD,
      RECIEVE_CRC,
      PROCESS_CRC
    };

    enum _Error {
      TIMED_OUT,
      PAYLOAD_TOO_LONG,
      PAYLOAD_INVALLID_LENGTH,
      MORE_THEN_MAX_HANDLERS,
      NO_HANDLER_FOUND,
      CRC_ERROR,
      HANDLER_ERROR,
      HEADER_ERROR
    };

    enum _PreHeader {
      PRE_HEADER_1,
      PRE_HEADER_2,
      SOH
    };

    //data type unions, elk bestaand uit 1 datatype en een byte array met dezelfde aantal bytes. De naam is het data type met een hoofdletter.
    // t is het data type
    // b is de byte array

    //int8_t // same as char
    union Int8_t{
      int8_t t;
      uint8_t b;
    };
    //int16_t
    union Int16_t{
      int16_t t;
      uint8_t b[2];
    };
    //int32_t
    union Int32_t{
      int32_t t;
      uint8_t b[4];
    };
    //int64_t
    union Int64_t{
      int64_t t;
      uint8_t b[8];
    };
    //uint8_t // same as byte
    //uint16_t
    //uint32_t
    //uint64_t

    //float
    //double

    //watermark. Voor elke control byte is dit te vinden (3 bytes aan  'random' data):
    const uint8_t _watermark[3] = {
      0xF2,
      0x3F,
      0x5D
    };
    
    //vars
    Stream* _serial;
    uint16_t _handlerCallbackPort[MAX_HANDLER_CALLBACK];
    boolean _ownHandlerManager;
    uint32_t _startTime;
    uint8_t _header[HEADER_LENGHT];
    uint8_t _payload[MAX_PAYLOAD];
    uint8_t _crc[CRC_LENGHT];
    uint16_t _payloadLen;
    uint16_t _handlerPort;
    _State _state;
    uint16_t _cursor;
    uint16_t _crcCalculaded;
    uint16_t _crcRecieved;

    uint16_t _handlerCallbackCount;
    
    const uint8_t _NAK = 0x15;
    const uint8_t _ACK = 0x06;
    const uint8_t _SOH = 0x01;
    const uint8_t _STX = 0x02;
    const uint8_t _ETX = 0x03;
    const uint8_t _EOT = 0x04;

    _PreHeader _preHeader;
    const uint8_t _preHeaderData[2] = {0xF2, 0x28};

    //functions
    uint16_t _byteToInt16(uint8_t first, uint8_t second);
    void _reset();
    void _errorHandler(_Error error);
    void _successHandler();
    void _timeOutHandler();
    
    void _expectHeader();
    void _recieveHeader();
    void _processHeader();
    void _recievePayload();
    void _processPayload();
    void _recieveCRC();
    void _processCRC();

    boolean _isHandlerCallback(uint16_t handlerPort);
    int32_t _getHandlerCallback(uint16_t handlerPort);
    boolean _handlerManager(boolean execute = true);

    boolean _checkWatermark();

    //CRC for NOT AVR
    #ifndef _crc16_update
    uint16_t _crc16_update(uint16_t crc, uint8_t data);
    #endif

    //callbacks
    boolean (*_handlerCallback[MAX_HANDLER_CALLBACK])(uint8_t payload[], uint16_t payloadLenght);
    boolean (*_handlerManagerCallback)(uint8_t payload[], uint16_t payloadLenght, uint16_t handlerPort);
};

