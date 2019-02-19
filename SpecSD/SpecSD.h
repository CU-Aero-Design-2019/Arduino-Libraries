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

        writeTelemetry("lat,lng,gpsAlt,speedMPS,bmpAlt,cHead,wE,wN,hE,hW,cE,cN,cU,gpsT,millis");

        /*
        sdt += String(SpecGPS::gps.location.lat(), 6);
        sdt += " ";
        sdt += String(SpecGPS::gps.location.lng(), 6); // deg
        sdt += " ";
        sdt += String(SpecGPS::getOffsetAlt(), 1); // m
        sdt += " ";
        sdt += String(SpecGPS::gps.speed.mps(), 2); // m/s
        sdt += " ";
        sdt += String(bmp.getKAlt(), 2); // m
        sdt += " ";
        sdt += String(SpecHMC5883::heading, 2); // deg
        sdt += " ";
        sdt += String(Prediction::watPrediction.e, 2);
        sdt += " ";
        sdt += String(Prediction::watPrediction.n, 2);
        sdt += " ";
        sdt += String(Prediction::habPrediction.e, 2);
        sdt += " ";
        sdt += String(Prediction::habPrediction.n, 2);
        sdt += " ";
        sdt += String(SpecGPS.currentENU.e);
        sdt += " ";
        sdt += String(SpecGPS.currentENU.n);
        sdt += " ";
        sdt += String(SpecGPS.currentENU.u);
        sdt += " ";
        sdt += String(SpecGPS::gps.time.value());
        sdt += " ";
        sdt += String(millis() / 100);
        */

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