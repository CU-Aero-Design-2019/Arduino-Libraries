#ifndef SPECMPU6050_H
#define SPECMPU6050_H

#include <Arduino.h>
#include <Wire.h>

namespace SpecMPU6050{

    // timer
    const long UpdatePeriod = 100;
    long UpdateTimer = 0;

    // communication stuff
    const int MPU6050_ADDR        = 0x68;
    const int MPU6050_SMPLRT_DIV  = 0x19;
    const int MPU6050_CONFIG      = 0x1a;
    const int MPU6050_GYRO_CONFIG = 0x1b;
    const int MPU6050_ACCEL_CONFIG= 0x1c;
    const int MPU6050_WHO_AM_I    = 0x75;
    const int MPU6050_PWR_MGMT_1  = 0x6b;
    const int MPU6050_TEMP_H      = 0x41;
    const int MPU6050_TEMP_L      = 0x42;

    const int MagAddr = 0x1E;

    // vars to store the raw data
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
	float gyroXoffset, gyroYoffset, gyroZoffset;
	float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
	float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY, angleAccZ;
	float angleX, angleY, angleZ;
    float interval;
	long preInterval;
	float accCoef;
    float gyroCoef;

    void writeIMU(byte reg, byte data){
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(reg);
        Wire.write(data);
        Wire.endTransmission();
    }

    void update(){
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom((int)MPU6050_ADDR, 14);

        rawAccX = Wire.read() << 8 | Wire.read();
        rawAccY = Wire.read() << 8 | Wire.read();
        rawAccZ = Wire.read() << 8 | Wire.read();
        rawTemp = Wire.read() << 8 | Wire.read();
        rawGyroX = Wire.read() << 8 | Wire.read();
        rawGyroY = Wire.read() << 8 | Wire.read();
        rawGyroZ = Wire.read() << 8 | Wire.read();

        temp = (rawTemp + 12412.0) / 340.0;

        accX = ((float)rawAccX) / 16384.0;
        accY = ((float)rawAccY) / 16384.0;
        accZ = ((float)rawAccZ) / 16384.0;

        angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
        angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

        gyroX = ((float)rawGyroX) / 65.5;
        gyroY = ((float)rawGyroY) / 65.5;
        gyroZ = ((float)rawGyroZ) / 65.5;

        gyroX -= gyroXoffset;
        gyroY -= gyroYoffset;
        gyroZ -= gyroZoffset;

        interval = (millis() - preInterval) * 0.001;

        angleGyroX += gyroX * interval;
        angleGyroY += gyroY * interval;
        angleGyroZ += gyroZ * interval;

        angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
        angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
        angleZ = angleGyroZ;

        preInterval = millis();
    }

    void setup(){
        Wire.begin();
        writeIMU(MPU6050_SMPLRT_DIV, 0x00);
        writeIMU(MPU6050_CONFIG, 0x00);
        writeIMU(MPU6050_GYRO_CONFIG, 0x08);
        writeIMU(MPU6050_ACCEL_CONFIG, 0x00);
        writeIMU(MPU6050_PWR_MGMT_1, 0x01);
        update();
    }    

};

#endif
