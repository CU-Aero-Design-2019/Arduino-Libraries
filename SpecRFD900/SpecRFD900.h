#ifndef SPECRFD900_H
#define SPECRFD900_H

#include "SpecBMP180.h"

extern SpecBMP180 bmp;

namespace SpecRFD900
{
int UpdateTimer = 0;
const int UpdatePeriod = 1;

int baudrate = 9600;
String incoming = "";
bool waitingForSerial = false;
long firstSerialAvailableTime;
HardwareSerial *RFD900;

void setup(HardwareSerial *serial)
{
    RFD900 = serial;
    RFD900->begin(baudrate);
#ifdef DEBUG
    RFD900->println("RFD900 Setup");
    Serial.println("RFD900 Setup");
#endif
}

void send(String str)
{
    RFD900->println(str);
}

// does stuff with the incoming string
// should be called from update()
void parse()
{
    if (incoming.substring(0, 4).equals("STAR"))
    {
        Settings::setTarget("implementme!", "implementme!", "implementme!");
    }
    else if (incoming.substring(0, 4).equals("SRVO"))
    {
        Serial.println(incoming.substring(4));
        Serial.println(incoming.substring(4).toInt());
    }
}

struct TelemetryStruct
{
    long sendTime;
    int altitude;
};

void sendTelemetry()
{
    // gather everything to send
    TelemetryStruct packet;
    packet.sendTime = millis();
    packet.altitude = bmp.readAltitude();
    #ifdef DEBUG
    //Serial.println(packet.altitude);
    #endif

    // send message header
    //RFD900->print((char)sizeof(packet));

    // send message
    // for (int addressOffset = 0; addressOffset < sizeof(packet); addressOffset++)
    // {
    //     #ifdef DEBUG
    //     //Serial.println("Writing EEPROM");
    //     #endif
    //     RFD900->write(*((char *)&packet + addressOffset));
    // }
    RFD900->print(packet.sendTime);
    RFD900->print(packet.altitude);
}

// to be called at a regular interval
// updates incoming string
void update()
{
    //parse serial1 input
    if (Serial.available() && !waitingForSerial)
    {
        waitingForSerial = true;
        firstSerialAvailableTime = millis();
    }
    // if serial has something available and we've waited serialDelay
    if (waitingForSerial && (firstSerialAvailableTime + serialDelay >= millis()))
    {
        // set this pesky thing back to false since we're not waiting anymore
        waitingForSerial = false;
        incoming = "";
        while (Serial.available())
        {
            incoming += (char)Serial.read();
        }
#ifdef DEBUG
        Serial.print("Incoming USB Serial Data: ");
        Serial.print(incoming);
#endif
        // do something with serial input
        USB::parse();
    }
}
}; // namespace SpecRFD900

#endif