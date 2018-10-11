#ifndef SPECGPS_H
#define SPECGPS_H

#define GPSSerial Serial2

namespace SpecGPS{
    void setup(){
        GPSSerial.begin(GPSSerialBaudrate);
    }

};

#endif