#include "IMU_v5.h"
#include "Wire.h"
#include <LSM6.h>

IMU_v5::IMU_v5(){
}

/////////////////////////////////////////////////////////////////////////////
void IMU_v5::begin(LSM6* imu) {
	Wire.begin();

	_IMU = imu;
	if (!_IMU->init()) {
		Serial.println ("Failed to detect and initialize IMU!");
		while (1);
	}
	_IMU -> enableDefault();

	// Pololu AltIMU LSM6DS33 accelerometer and gyro sensor
	// Accelerometer +/- 4 g scale: CTRL1_XL register: xxxx10xx
	byte LSM6_CTRL1 = _IMU -> readReg(LSM6::CTRL1_XL);
	LSM6_CTRL1 |= _BV(3);       // Write 1 in the register
	LSM6_CTRL1 &= ~(_BV(2));    // write 0 in the register
	_IMU -> writeReg(LSM6::CTRL1_XL, LSM6_CTRL1);

	// Gyro 500 dps full-scale: CTRL2_G register: xxxx01xx
	byte LSM6_CTRL2 = _IMU -> readReg(LSM6::CTRL2_G);
	LSM6_CTRL2 |= _BV(2);       // Write 1 in the register
	LSM6_CTRL2 &= ~(_BV(3));    // write 0 in the register
	_IMU -> writeReg(LSM6::CTRL2_G, LSM6_CTRL2);
}	
/////////////////////////////////////////////////////////////////////////////
void IMU_v5::getOffsetNoise(){
	int sampleNum = 100;

	for(int i=0; i < sampleNum; i++) {
		Read_Acc();
		Read_Gyro();
		for (int j = 0; j < 6; j++){
			ArrayOffset[j] += ArrayAngles[j]; 
		}
		delay(10);
	}

	for(int j=0; j < 6; j++)
		ArrayOffset[j] = ArrayOffset[j] / sampleNum;

	for(int j=0; j < sampleNum; j++) { 
		Read_Acc();
		Read_Gyro();
		for (int j = 0; j < 6; j++) {
			int noise = abs(ArrayAngles[j] - ArrayOffset[j]);
			if (noise > ArrayNoise[j])
				ArrayNoise[j] = noise;
		}
		delay(10);
	}
}
/////////////////////////////////////////////////////////////////////////////
int* IMU_v5::Read_Acc(){
	_IMU -> readAcc();
	
	ArrayAngles[0] = _IMU -> a.x;
	ArrayAngles[1] = _IMU -> a.y;
	ArrayAngles[2] = _IMU -> a.z;  

	int a_x = ArrayAngles[0] - ArrayOffset[0];
	int a_y = ArrayAngles[1] - ArrayOffset[1];
	int a_z = ArrayAngles[2] - ArrayOffset[2];
	
	//Low Pass Filter
	float alpha = 0.25;
    acc_data[0] = (alpha * a_x) + ((1 - alpha) * acc_data[0]);
    acc_data[1] = (alpha * a_y) + ((1 - alpha) * acc_data[1]);
    acc_data[2] = (alpha * a_z) + ((1 - alpha) * acc_data[2]);
	
	return acc_data;
}
/////////////////////////////////////////////////////////////////////////////
int* IMU_v5::Read_Gyro(){
	_IMU -> readGyro();

	ArrayAngles[3] = _IMU -> g.x;
	ArrayAngles[4] = _IMU -> g.y;
	ArrayAngles[5] = _IMU -> g.z;

	gyro_data[0] = ArrayAngles[3] - ArrayOffset[3];
	gyro_data[1] = ArrayAngles[4] - ArrayOffset[4];
	gyro_data[2] = ArrayAngles[5] - ArrayOffset[5];

	return gyro_data;
}
/////////////////////////////////////////////////////////////////////////////
int* IMU_v5::Get_Noise() { 
	
	return ArrayNoise;
}

