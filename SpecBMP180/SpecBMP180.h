// Mostly taken from https://github.com/adafruit/Adafruit-BMP085-Library

#ifndef SPECBMP180_H
#define SPECBMP180_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#define BMP085_DEBUG 0

#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD          0x2E
#define BMP085_READPRESSURECMD            0x34

#define NUMBEROFSAMPLES 10

class SpecBMP180
{
    public:
        SpecBMP180();
        boolean begin(uint8_t nInitSamples = 100, uint8_t mode = BMP085_ULTRAHIGHRES); // by default go highres
        float readTemperature(void);
        int32_t readPressure(void);
        int32_t readSealevelPressure(float altitude_meters = 0);
		float readAvgOffsetAltitude(float sealevelPressure = 101325);
        float readAltitude(float sealevelPressure = 101325); // std atmosphere
        float readOffsetAltitude(float sealevelPressure = 101325);
        uint16_t readRawTemperature(void);
        uint32_t readRawPressure(void);

        unsigned long UpdateTimer = 0;
        const unsigned long UpdatePeriod = 10;

        float baselineAlt = 0;
        int nAverageSamples = 0;
        float averageSum = 0;

    private:
        int32_t computeB5(int32_t UT);
        uint8_t read8(uint8_t addr);
        uint16_t read16(uint8_t addr);
        void write8(uint8_t addr, uint8_t data);
		
		uint8_t currentSample = 0;
		float samples[NUMBEROFSAMPLES];
		float avgSum = 0;

    uint8_t oversampling;

    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
};

#endif
