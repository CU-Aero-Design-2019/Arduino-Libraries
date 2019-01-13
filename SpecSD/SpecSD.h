#ifndef SPECSD_H
#define SPECSD_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

namespace SpecSD{

    unsigned long UpdateTimer = 0;
    const unsigned long UpdatePeriod = 10;
    String fileName;
    File dataFile;

    void setup(String theFileName) {
        // PB12 is needed as a CS pin. We shouldn't need to use it in hardware, but it's needed for the lib
        if(!SD.begin(PB12)){
            Serial.println("SD begin failed!");
        }else{
            //Serial.println("SD begin pass");
        }
		
		File indexFile;
		indexFile = SD.open((theFileName + ".txt"), FILE_READ);
		int currentIndex = 0;
		while (indexFile.available()) {
			currentIndex = ((int)indexFile.read());
		}
		indexFile.close();
		
		// open file and write new index
		indexFile = SD.open((theFileName + ".txt"), FILE_WRITE);
		indexFile.write(++currentIndex);
		indexFile.close();
		
		fileName = theFileName + String(currentIndex) + ".txt";

	}

    void writeTelemetry(String data) {
        dataFile = SD.open(fileName, FILE_WRITE);
        //Serial.println(fileName);
        if(dataFile){
            dataFile.print(data);
			dataFile.print(" ");
            dataFile.close();
        }
    }

};

#endif