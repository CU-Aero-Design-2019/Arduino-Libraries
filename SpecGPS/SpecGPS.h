#ifndef SPECGPS_H
#define SPECGPS_H

#define GPSSerial Serial2

#include <TinyGPS++.h>

namespace SpecGPS {
	
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

const int GPSSerialBaudrate = 9600;

bool hasLock = false;

const float deg_to_rad = 0.01745329251;

// The TinyGPS++ object
TinyGPSPlus gps;

void setup() {
	GPSSerial.begin(GPSSerialBaudrate);
	
	// tell the GPS to update at 10Hz
	// to get the correct message, copy the hex part from u-center's hex view in the message view window.
	uint8_t setupGPS1[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64};
	uint8_t setupGPS2[] = {0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
	// 1Hz alternative
	// uint8_t setupGPS1[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8};
	// uint8_t setupGPS2[] = {0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};
	
	//#ifdef GLIDER
		// Disable all but GPS and GLONASS
		uint8_t setupGPS3[] = {0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00, 0x00, 0x00, 0x20, 0x04, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00};
		uint8_t setupGPS4[] = {0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0xD7, 0x89};
		GPSSerial.write(setupGPS3, sizeof(setupGPS3));
		GPSSerial.write(setupGPS4, sizeof(setupGPS4));
	//#endif
	
	GPSSerial.write(setupGPS1, sizeof(setupGPS1));
	GPSSerial.write(setupGPS2, sizeof(setupGPS1));
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

int bearing(float lat, float lon, float lat2, float lon2) {

	float teta1 = radians(lat);
	float teta2 = radians(lat2);
	float delta1 = radians(lat2-lat);
	float delta2 = radians(lon2-lon);

	float y = sin(delta2) * cos(teta2);
	float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
	float brng = atan2(y,x);
	brng = degrees(brng);// radians to degrees
	
	while (brng >= 360) {
		brng -= 360.0;
	}

	return round(brng);
}

void lla_to_ecef(LLA& in, ECEF& out) {
	const int a = 6378137; //semi major axis of earth in meters
	const float b = 6356752.314245; //semi minor axis of earth in meters
	const float e_sqrd = 0.0066943799902;
	
	float N_phi = a/ sqrt(1 - (e_sqrd*pow(sin(in.lat * deg_to_rad),2)));

	out.x = (N_phi + in.alt)*cos(in.lat * deg_to_rad)*cos(in.lng * deg_to_rad);
	out.y = (N_phi + in.alt)*cos(in.lat * deg_to_rad)*sin(in.lng * deg_to_rad);
	out.z = ((N_phi*(1-e_sqrd))+in.alt) * sin(in.lat * deg_to_rad);
}

void ecef_to_enu(LLA lla_ref, ECEF ecef_ref, ECEF ecef_data, ENU& out) {
	float matrix_a[3][3] = {
		{ -sin(lla_ref.lng * deg_to_rad), cos(lla_ref.lng * deg_to_rad), 0 },
		{ -sin(lla_ref.lat * deg_to_rad)*cos(lla_ref.lng * deg_to_rad), -sin(lla_ref.lat * deg_to_rad)*sin(lla_ref.lng * deg_to_rad), cos(lla_ref.lat * deg_to_rad) },
		{ cos(lla_ref.lat * deg_to_rad)*cos(lla_ref.lng * deg_to_rad), cos(lla_ref.lat * deg_to_rad)*sin(lla_ref.lng * deg_to_rad), sin(lla_ref.lat * deg_to_rad) }
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
	lla_to_ecef(in, temp);
	ecef_to_enu(lla_ref, ecef_ref, temp, out);
}

void lla_to_enu(LLA& in, LLA lla_ref, ENU& out){
	ECEF temp;
	lla_to_ecef(in, temp);
	ECEF ecef_ref;
	lla_to_ecef(lla_ref, ecef_ref);
	ecef_to_enu(lla_ref, ecef_ref, temp, out);
}

bool equals(LLA a, LLA b){
	return (a.lat == b.lat && a.lng == b.lng && a.alt == b.alt);
}

};

#endif