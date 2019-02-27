#ifndef SPECGPS_H
#define SPECGPS_H

#define GPSSerial Serial2

//#include <TinyGPS++.h>

#include <UBLOX.h>

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

//const int GPSSerialBaudrate = 115200;

bool hasLock = false;

float baselineAlt = 0;

const float deg_to_rad = 0.01745329251;

ENU currentENU;

// The TinyGPS++ object
//TinyGPSPlus gps;

UBLOX ubg(GPSSerial, 57600);

void setup() {
	#ifdef GLIDER
	//Is a glider build
	GPSSerial.begin(9600);
	#else
	//Is a mothership build
	GPSSerial.begin(57600);
	#endif
	
	
	byte setups[] = {
		
	// GNSS configure all, only GLONASS and GPS on
	0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2D, 0x51,
	// GNSS only GPS
	//0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2C, 0x4D,

	// 10 Hz
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
	// 5 Hz
	//0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,
	
	//#ifdef GLIDER
	// GxGGA on
	//0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,
	// GxRMC on
	//0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x44,
	//#else
	// GxGGA off
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23,
	// GxRMC off
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F,
	//Turn on NAV-PVT
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1,
	//#endif

	//TurnOffGLL
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A,
	//TurnOffGSA
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31,
	//TurnOffGSV
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38,
	//TurnOffVTG
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,
	
	// 57600 bps
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xC9
	
	};
		  
	GPSSerial.write(setups, sizeof(setups));
	
	GPSSerial.end();
	delay(20);

	#ifdef GLIDER
	//	Serial.begin(57600);
	#endif
		ubg.begin();
	
}

void resetOffset() {
	baselineAlt = ubg.getMSLHeight_m();
}

void update() {
	ubg.readSensor();
}

float getOffsetAlt() {
	return ubg.getMSLHeight_m() - baselineAlt;
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

void lla_to_enu(double &a, double &b, double &c, double targLat, double targLng) {
	LLA in;
	in.lat = a;
	in.lng = b;
	in.alt = c;
	LLA target;
	target.lat = targLat;
	target.lng = targLng;
	target.alt = 0;
	ENU out;
	lla_to_enu(in, target, out);
	a = out.e;
	//Serial.print("a = "); Serial.println(a);
	b = out.n;
	//Serial.print("b = "); Serial.println(b);
	c = out.u;
}

bool equals(LLA a, LLA b){
	return (a.lat == b.lat && a.lng == b.lng && a.alt == b.alt);
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

};
#endif