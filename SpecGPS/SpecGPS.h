#ifndef SPECGPS_H
#define SPECGPS_H

#define GPSSerial Serial2

#include <TinyGPS++.h>

namespace SpecGPS
{
	
struct LLA{
	float lat;
	float lng;
	float alt;
};

struct ENU{
	float e;
	float n;
	float u;
};

struct ECEF{
	float x;
	float y;
	float z;
};

// timer
const long UpdatePeriod = 100;
unsigned long UpdateTimer = 0;

bool hasLock = false;

// The TinyGPS++ object
TinyGPSPlus gps;

void setup()
{
	GPSSerial.begin(GPSSerialBaudrate);
	
	// tell the GPS to update at 10Hz
	// to get the correct message, copy the hex part from u-center's hex view in the message view window.
	uint8_t setupGPS1[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64};
	uint8_t setupGPS2[] = {0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
	
	// 1Hz alternative
	// uint8_t setupGPS1[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8};
	// uint8_t setupGPS2[] = {0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};
	
	GPSSerial.write(setupGPS1, sizeof(setupGPS1));
	GPSSerial.write(setupGPS2, sizeof(setupGPS1));
}

void displayInfo()
{
	Serial.print(F("Location: "));
	if (gps.location.isValid())
	{
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid())
	{
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.println();
}

void update() {
	while (GPSSerial.available() > 0){
		gps.encode(GPSSerial.read());
	}
	if (gps.location.age() < 1000) {
		hasLock = true;
	} else {
		hasLock = false;
	}
}

void lla_to_ecef(LLA& in, ECEF& out) {
	float a = 6378137.0; //semi major axis of earth in meters
	float b = 6356752.314245; //semi minor axis of earth in meters

	float N_phi = (pow(a, 2)) / sqrt(pow(a, 2)*pow(cos(in.lat), 2) + pow(b, 2)*pow(sin(in.lat), 2));

	out.x = (N_phi + in.alt)*cos(in.lat)*cos(in.lng);
	out.y = (N_phi + in.alt)*cos(in.lat)*sin(in.lng);
	out.z = ((pow(b, 2) / pow(a, 2))*N_phi + in.alt)*sin(in.lat);
}

void ecef_to_enu(LLA lla_ref, ECEF ecef_ref, ECEF ecef_data, ENU& out) {
	float matrix_a[3][3] = {
		{ -sin(lla_ref.lng), cos(lla_ref.lng), 0 },
		{ -sin(lla_ref.lat)*cos(lla_ref.lng), -sin(lla_ref.lat)*sin(lla_ref.lng), cos(lla_ref.lat) },
		{ cos(lla_ref.lat)*cos(lla_ref.lng), cos(lla_ref.lat)*sin(lla_ref.lng), sin(lla_ref.lat) }
	};

	float delta_x = ecef_data.x - ecef_ref.x;
	float delta_y = ecef_data.y - ecef_ref.y;
	float delta_z = ecef_data.z - ecef_ref.z;

	float delta_vec[3] = { delta_x , delta_y , delta_z };

	int i, j;
	float tempEnu[3];
	for (i = 0; i < 3; i++) {
		tempEnu[i] = 0;
		for (j = 0; j < 3; j++) {
			tempEnu[i] += matrix_a[i][j] * delta_vec[j];
		}
	}
	
	out.e = tempEnu[0];
	out.n = tempEnu[1];
	out.u = tempEnu[2];
}

void lla_to_enu(LLA& in, LLA lla_ref, ECEF ecef_ref, ENU& out){
	ECEF temp;
	lla_to_ecef(in,temp);
	ecef_to_enu(lla_ref, ecef_ref, temp, out);
}

};

#endif