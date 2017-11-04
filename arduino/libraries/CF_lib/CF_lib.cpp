/*
  CF_lib.h
  Created by Oskar Casquero, March 28, 2017.
  Released into the public domain.
*/

#include "CF_lib.h"

CF_lib::CF_lib() {
	timer = micros();
}

void CF_lib::begin(int* accel) {
	int accel_x = *accel;
	int accel_y = *(accel+1);
	int accel_z = *(accel+2);
	
    long squaresum = (long)square(accel_y) + (long)square(accel_z);
	pitch = atan(accel_x/sqrt(squaresum))*RAD_TO_DEG;
	pitchGyro = pitch;
}

float CF_lib::Compute_Pitch(int* accel, int gyro_y, int data_noise) {
	int accel_x = *accel;
	int accel_y = *(accel+1);
	int accel_z = *(accel+2);

	float dt = (micros() - timer) / 1000000.0f;
	if(gyro_y > data_noise || gyro_y < -data_noise) {
		// Trapezoidal rule for integrating angular velocity
		// 17.50mdps/digit is L3G20H gyro's sensitivity for 500 dps scale
		float rotAng = 0.0175f * ((gyro_y + prev_gyro_y) / 2) * dt;
		pitch += rotAng; 
		pitchGyro += rotAng;
		prev_gyro_y = gyro_y;    
	}
	timer = micros();
	
	long squaresum = (long)square(accel_y) + (long)square(accel_z);
	float pitchAccel = -atan(accel_x/sqrt(squaresum))*RAD_TO_DEG;
 	
	pitch = compCoeff*pitch + (1.0f-compCoeff)*pitchAccel;

  	//Serial.print(pitch,2); 
	//Serial.print(" "); Serial.print(pitchAccel,2); 
	//Serial.print(" "); Serial.println(pitchGyro,2);
	
	return pitch;
}