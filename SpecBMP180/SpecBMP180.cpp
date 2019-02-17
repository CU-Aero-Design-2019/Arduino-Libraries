#include "SpecBMP180.h"

SpecBMP180::SpecBMP180() : altFilter(0.1, 20, 0.01) {
//SpecBMP180::SpecBMP180() {

}

// updates kalman filter, altitudes
void SpecBMP180::update() {
	rawAlt = updateOffsetAltitude();
	filteredAlt = altFilter.updateEstimate(rawAlt);
	avgAlt = updateAvgOffsetAltitude(rawAlt);
}

// returns altitude after kalman filter
float SpecBMP180::getKAlt() {
	return filteredAlt;
}

float SpecBMP180::readAvgOffsetAltitude(){
	return avgAlt;
}

void SpecBMP180::resetOffset(int nSamples) {
	int sum = 0;
	for (int i = 0; i < nSamples; i++) {
		sum += this->readAltitude();
	}
	sum /= nSamples;
	this->baselineAlt = sum;
	Serial.println("Resetting baseline alt");
}

boolean SpecBMP180::begin(uint8_t nInitSamples, uint8_t mode) {
    if (mode > BMP085_ULTRAHIGHRES)
        mode = BMP085_ULTRAHIGHRES;
    oversampling = mode;

    Wire.begin();

    if (read8(0xD0) != 0x55)
        return false;
	
    /* read calibration data */
    ac1 = read16(BMP085_CAL_AC1);
    ac2 = read16(BMP085_CAL_AC2);
    ac3 = read16(BMP085_CAL_AC3);
    ac4 = read16(BMP085_CAL_AC4);
    ac5 = read16(BMP085_CAL_AC5);
    ac6 = read16(BMP085_CAL_AC6);
	
	

    b1 = read16(BMP085_CAL_B1);
    b2 = read16(BMP085_CAL_B2);

    mb = read16(BMP085_CAL_MB);
    mc = read16(BMP085_CAL_MC);
    md = read16(BMP085_CAL_MD);
#if (BMP085_DEBUG == 1)
    Serial.print("ac1 = ");
    Serial.println(ac1, DEC);
    Serial.print("ac2 = ");
    Serial.println(ac2, DEC);
    Serial.print("ac3 = ");
    Serial.println(ac3, DEC);
    Serial.print("ac4 = ");
    Serial.println(ac4, DEC);
    Serial.print("ac5 = ");
    Serial.println(ac5, DEC);
    Serial.print("ac6 = ");
    Serial.println(ac6, DEC);

    Serial.print("b1 = ");
    Serial.println(b1, DEC);
    Serial.print("b2 = ");
    Serial.println(b2, DEC);

    Serial.print("mb = ");
    Serial.println(mb, DEC);
    Serial.print("mc = ");
    Serial.println(mc, DEC);
    Serial.print("md = ");
    Serial.println(md, DEC);
#endif

    for(int i = 0; i < nInitSamples; i++){
        this->baselineAlt += this->readAltitude();
    }
    this->baselineAlt /= nInitSamples;

	for(int i = 0; i < NUMBEROFSAMPLES; i++){
		samples[i] = 0;
	}
	
    return true;
}

int32_t SpecBMP180::computeB5(int32_t UT) {
    int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
    int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
    return X1 + X2;
}

uint16_t SpecBMP180::readRawTemperature(void) {
    write8(BMP085_CONTROL, BMP085_READTEMPCMD);
    delay(5);
#if BMP085_DEBUG == 1
    Serial.print("Raw temp: ");
    Serial.println(read16(BMP085_TEMPDATA));
#endif
    return read16(BMP085_TEMPDATA);
}

uint32_t SpecBMP180::readRawPressure(void) {
    uint32_t raw;

    write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

    if (oversampling == BMP085_ULTRALOWPOWER)
        delay(5);
    else if (oversampling == BMP085_STANDARD)
        delay(8);
    else if (oversampling == BMP085_HIGHRES)
        delay(14);
    else
        delay(26);

    raw = read16(BMP085_PRESSUREDATA);

    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA + 2);
    raw >>= (8 - oversampling);

    /* this pull broke stuff, look at it later?
  if (oversampling==0) {
    raw <<= 8;
    raw |= read8(BMP085_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
  }
 */

#if BMP085_DEBUG == 1
    Serial.print("Raw pressure: ");
    Serial.println(raw);
#endif
    return raw;
}

int32_t SpecBMP180::readPressure(void) {
    int32_t B3, B5, B6, X1, X2, X3;
    uint32_t B4, B7;

    switch (state) {
        case 0:
            write8(BMP085_CONTROL, BMP085_READTEMPCMD);
            nextTime = millis() + 5;
            state++;
            break;
        case 1:
            if (millis() >= nextTime) {
                UT = read16(BMP085_TEMPDATA);
                write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
                switch (oversampling) {
                    case BMP085_ULTRALOWPOWER:
                        nextTime = millis() + 5;
                        break;
                    case BMP085_STANDARD:
                        nextTime = millis() + 8;
                        break;
                    case BMP085_HIGHRES:
                        nextTime = millis() + 14;
                        break;
                    default:
                        nextTime = millis() + 26;
                        break;
                }
                B5 = computeB5(UT);
                state++;
            }
            break;
        case 2:
            if (millis() >= nextTime) {
                UP = read16(BMP085_PRESSUREDATA);
                UP <<= 8;
                UP |= read8(BMP085_PRESSUREDATA + 2);
                UP >>= (8 - oversampling);

                // do pressure calcs
                B6 = B5 - 4000;
                X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
                X2 = ((int32_t)ac2 * B6) >> 11;
                X3 = X1 + X2;
                B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

                X1 = ((int32_t)ac3 * B6) >> 13;
                X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
                X3 = ((X1 + X2) + 2) >> 2;
                B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
                B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oversampling);

                if (B7 < 0x80000000) {
                    p = (B7 * 2) / B4;
                } else {
                    p = (B7 / B4) * 2;
                }
                X1 = (p >> 8) * (p >> 8);
                X1 = (X1 * 3038) >> 16;
                X2 = (-7357 * p) >> 16;

                p = p + ((X1 + X2 + (int32_t)3791) >> 4);

                state = 0;
            }
            break;
    }

    


    return p;
}

int32_t SpecBMP180::readSealevelPressure(float altitude_meters) {
    float pressure = readPressure();
    return (int32_t)(pressure / pow(1.0 - altitude_meters / 44330, 5.255));
}

float SpecBMP180::readTemperature(void) {
    int32_t UT, B5; // following ds convention
    float temp;

    UT = readRawTemperature();

#if BMP085_DEBUG == 1
    // use datasheet numbers!
    UT = 27898;
    ac6 = 23153;
    ac5 = 32757;
    mc = -8711;
    md = 2868;
#endif

    B5 = computeB5(UT);
    temp = (B5 + 8) >> 4;
    temp /= 10;

    return temp;
}

float SpecBMP180::readAltitude(float sealevelPressure) {
    float altitude;

    float pressure = readPressure();
    
    altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));

    return altitude;
}

float SpecBMP180::updateOffsetAltitude(float sealevelPressure) {
    return (readAltitude(sealevelPressure) - baselineAlt);
}

float SpecBMP180::readOffsetAltitude(float sealevelPressure) {
    return rawAlt;
}

float SpecBMP180::updateAvgOffsetAltitude(float alt) {
	avgSum -= samples[currentSample];                              
	
	samples[currentSample] = alt;
	
	avgSum += samples[currentSample];
	currentSample++;
	currentSample = currentSample % NUMBEROFSAMPLES;
	return avgSum/NUMBEROFSAMPLES;
}

/*********************************************************************/

uint8_t SpecBMP180::read8(uint8_t a) {
    uint8_t ret;

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
#if (ARDUINO >= 100)
    Wire.write(a); // sends register address to read from
#else
    Wire.send(a);         // sends register address to read from
#endif
    Wire.endTransmission(); // end transmission

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 1);    // send data n-bytes read
#if (ARDUINO >= 100)
    ret = Wire.read(); // receive DATA
#else
    ret = Wire.receive(); // receive DATA
#endif
    Wire.endTransmission(); // end transmission

    return ret;
}

uint16_t SpecBMP180::read16(uint8_t a) {
    uint16_t ret;

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
#if (ARDUINO >= 100)
    Wire.write(a); // sends register address to read from
#else
    Wire.send(a);         // sends register address to read from
#endif
    Wire.endTransmission(); // end transmission

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 2);    // send data n-bytes read
#if (ARDUINO >= 100)
    ret = Wire.read(); // receive DATA
    ret <<= 8;
    ret |= Wire.read(); // receive DATA
#else
    ret = Wire.receive(); // receive DATA
    ret <<= 8;
    ret |= Wire.receive(); // receive DATA
#endif
    Wire.endTransmission(); // end transmission

    return ret;
}

void SpecBMP180::write8(uint8_t a, uint8_t d) {
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
#if (ARDUINO >= 100)
    Wire.write(a); // sends register address to read from
    Wire.write(d); // write data
#else
    Wire.send(a);          // sends register address to read from
    Wire.send(d);          // write data
#endif
    Wire.endTransmission(); // end transmission
}