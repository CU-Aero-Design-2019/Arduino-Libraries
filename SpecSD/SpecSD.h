#ifndef SPECSD_H
#define SPECSD_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

namespace SpecSD{

    unsigned long UpdateTimer = 0;
    const unsigned long UpdatePeriod = 1;
    String fileName;
    File dataFile;

    void setup(String theFileName = "telemetryFile1") {
        fileName = theFileName;
        // PB12 is needed as a CS pin. We shouldn't need to use it in hardware, but it's needed for the lib
        if(!SD.begin(PB12)){
            Serial.println("SD begin failed!");
        }else{
            Serial.println("SD begin pass");
        }
    }
	
	void writeTelemetry(float data, int digits) {
		dataFile = SD.open(fileName, FILE_WRITE);
        
        if(dataFile){
            // Serial.print("writing to sd: ");
            // Serial.println(data);
            dataFile.print(data, digits);
			dataFile.print(" ");
            dataFile.close();
        }
	}
	
	void writeTelemetry(int data) {
		dataFile = SD.open(fileName, FILE_WRITE);
        
        if(dataFile){
            // Serial.print("writing to sd: ");
            // Serial.println(data);
            dataFile.print(data);
			dataFile.print(" ");
            dataFile.close();
        }
	}

    void writeTelemetry(String data) {
        dataFile = SD.open(fileName, FILE_WRITE);
        
        if(dataFile){
            // Serial.print("writing to sd: ");
            // Serial.println(data);
            dataFile.print(data);
			dataFile.print(" ");
            dataFile.close();
        }
    }

};

#endif