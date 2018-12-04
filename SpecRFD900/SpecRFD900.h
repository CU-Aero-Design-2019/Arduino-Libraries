#ifndef SPECRFD900_H
#define SPECRFD900_H

#include "SpecBMP180.h"
#include "SpecMPU6050.h"

extern SpecBMP180 bmp;

namespace SpecRFD900
{
unsigned long UpdateTimer = 0;
const unsigned long UpdatePeriod = 10;

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

void sendTelemetry(String data)
{
    // // make string that everything will be added to
    // String data = "";

    // // add time
    // //data += "time";
    // data += millis();
    // data += " ";
    
    // // add altitude
    // //data += "alt";
    // data += bmp.readAltitude();
    // data += " ";
    
    // // add angleX
    // //data += "angx";
    // data += SpecMPU6050::angleX;
    // data += " ";

    // //data += "temp";
    // data += SpecMPU6050::temp;
    // data += " ";

    // data += "\n";

    // // add ending char
    // //data += 0x17;
    // Serial.print("writing to rfd: ");
    // Serial.println(data);
    const char* charData = data.c_str();
    RFD900->write(charData, data.length());
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