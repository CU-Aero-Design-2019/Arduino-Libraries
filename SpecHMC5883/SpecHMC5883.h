// Code base taken from https://www.sparkfun.com/tutorials/301

#ifndef SPECHMC5883_H
#define SPECHMC5883_H

#include <Arduino.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include <HMC5883L.h>

namespace SpecHMC5883{
        
    HMC5883L compass;
	
	int error = 0;
	
	MagnetometerScaled valueOffset;
	
	SimpleKalmanFilter headingFilter(0.1, 20, 0.01);
	
	float headingDegrees;

    void setup(){
        Wire.begin();
		error = compass.setScale(1.3);
		if(error != 0) // If there is an error, print it out.
			Serial.println(compass.getErrorText(error));
			
		error = compass.setMeasurementMode(MEASUREMENT_CONTINUOUS);
		
		if(error != 0) // If there is an error, print it out.
			Serial.println(compass.getErrorText(error));
			
			
		Serial.println("calibrate the compass");
		MagnetometerScaled valueMax = {0, 0, 0};
		MagnetometerScaled valueMin = {0, 0, 0};

		// calculate x, y and z offset

		//Serial << "please rotate the compass" << endl;
		Serial.println("please rotate the compass");
		int xcount = 0;
		int ycount = 0;
		int zcount = 0;
		boolean xZero = false;
		boolean yZero = false;
		boolean zZero = false;
		MagnetometerScaled value;
		while (xcount < 3 || ycount < 3 || zcount < 3) {
			value = compass.readScaledAxis();
			if ((fabs(value.XAxis) > 600) || (fabs(value.YAxis) > 600) || (fabs(value.ZAxis) > 600)) {
			  continue;
			}

			if (valueMin.XAxis > value.XAxis) {
			  valueMin.XAxis = value.XAxis;
			} else if (valueMax.XAxis < value.XAxis) {
			  valueMax.XAxis = value.XAxis;
			}

			if (valueMin.YAxis > value.YAxis) {
			  valueMin.YAxis = value.YAxis;
			} else if (valueMax.YAxis < value.YAxis) {
			  valueMax.YAxis = value.YAxis;
			}

			if (valueMin.ZAxis > value.ZAxis) {
			  valueMin.ZAxis = value.ZAxis;
			} else if (valueMax.ZAxis < value.ZAxis) {
			  valueMax.ZAxis = value.ZAxis;
			}


			if (xZero) {
			  if (fabs(value.XAxis) > 50) {
				xZero = false;
				xcount++;
			  }
			} else {
			  if (fabs(value.XAxis) < 40) {
				xZero = true;
			  }
			}

			if (yZero) {
			  if (fabs(value.YAxis) > 50) {
				yZero = false;
				ycount++;
			  }
			} else {
			  if (fabs(value.YAxis) < 40) {
				yZero = true;
			  }
			}

			if (zZero) {
			  if (fabs(value.ZAxis) > 50) {
				zZero = false;
				zcount++;
			  }
			} else {
			  if (fabs(value.ZAxis) < 40) {
				zZero = true;
			  }
			}
			delay(30);
		}
		
		valueOffset.XAxis = (valueMax.XAxis + valueMin.XAxis) / 2;
		valueOffset.YAxis = (valueMax.YAxis + valueMin.YAxis) / 2;
		valueOffset.ZAxis = (valueMax.ZAxis + valueMin.ZAxis) / 2;
	}
    
    void update(){
		// Retrive the raw values from the compass (not scaled).
		MagnetometerRaw raw = compass.readRawAxis();
		// Retrived the scaled values from the compass (scaled to the configured scale).
		MagnetometerScaled scaled = compass.readScaledAxis();

		scaled.XAxis -= valueOffset.XAxis;
		scaled.YAxis -= valueOffset.YAxis;
		scaled.ZAxis -= valueOffset.ZAxis;

		// Values are accessed like so:
		int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

		// Calculate heading when the magnetometer is level, then correct for signs of axis.
		float yxHeading = atan2(scaled.YAxis, scaled.XAxis);
		float zxHeading = atan2(scaled.ZAxis, scaled.XAxis);

		float heading = yxHeading;

		// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
		// Find yours here: http://www.magnetic-declination.com/
		// Mine is: -2��37' which is -2.617 Degrees, or (which we need) -0.0456752665 radians, I will use -0.0457
		// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
		// float declinationAngle = -0.0457;
		// heading += declinationAngle;

		// Correct for when signs are reversed.
		if(heading < 0)
		heading += 2*PI;

		// Check for wrap due to addition of declination.
		if(heading > 2*PI)
		heading -= 2*PI;

		// Convert radians to degrees for readability.
		headingDegrees = heading * 180/M_PI;
    }

};

#endif