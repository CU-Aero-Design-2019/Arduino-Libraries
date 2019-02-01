// Code base taken from https://www.sparkfun.com/tutorials/301

#ifndef SPECHMC5883_H
#define SPECHMC5883_H

#include <Arduino.h>
#include <Wire.h>
#include <QMC5883L.h>

#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.0174532925199

namespace SpecQMC5883{
        
    const int address = 0x0D;

    const long UpdatePeriod = 100;
    unsigned long UpdateTimer = 0;

    int16_t x, y, z, t;

    int heading;
	int headingAverage;
	
	const int numSamps = 10;
	
	float cosines[numSamps];
	float sines[numSamps];
	
	float cosineSum = 0;
	float sineSum = 0;
	
	int sampleCount = 0;
	
    // void setup(){
    //     Wire.begin();
  
    //     //Put the HMC5883 IC into the correct operating mode
    //     Wire.beginTransmission(address); //open communication with HMC5883
    //     Wire.write(0x02); //select mode register
    //     Wire.write(0x00); //continuous measurement mode
    //     Wire.endTransmission(false);
    // }
    
    // void update(){
    //     //Tell the HMC5883L where to begin reading data
    //     Wire.beginTransmission(address);
    //     Wire.write(0x03); //select register 3, X MSB register
    //     Wire.endTransmission(false);
        
        
    //     //Read data from each axis, 2 registers per axis
    //     Wire.requestFrom(address, 6);
    //     if(6<=Wire.available()){
    //         x = Wire.read()<<8; //X msb
    //         x |= Wire.read(); //X lsb
    //         z = Wire.read()<<8; //Z msb
    //         z |= Wire.read(); //Z lsb
    //         y = Wire.read()<<8; //Y msb
    //         y |= Wire.read(); //Y lsb
    //     }
    // }

    //Vector data;
    QMC5883L compass;

    void setup(){
        compass.init();
    }

	void updateAverageHeading(){
		float rad_heading = heading * DEG_TO_RAD;
		
		cosineSum -= cosines[sampleCount];
		sineSum -= sines[sampleCount];
		
		cosines[sampleCount] = cos(rad_heading);
		sines[sampleCount] = sin(rad_heading);
		
		cosineSum += cosines[sampleCount];
		sineSum += sines[sampleCount];
		
		float avgCos = cosineSum/numSamps;
		float avgSin = sineSum/numSamps;
		
		float theta = atan(avgSin/avgCos) * RAD_TO_DEG;
		
		if(avgCos < 0.0){
			theta += 180;
		}else if(avgSin < 0.0){
			theta +=360;
		}
		sampleCount++;
		sampleCount = sampleCount%numSamps;
		headingAverage = theta;
	}
	
    void update(){
        // data = compass.readRaw();
		if(compass.ready()){
			heading = compass.readHeading();
			updateAverageHeading();
		}
		
    }

};

#endif