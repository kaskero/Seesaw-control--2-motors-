/*
  CF_lib.h
  Created by Oskar Casquero, March 28, 2017.
  Released into the public domain.
*/

#ifndef CF_lib_h
#define CF_lib_h

#include "Arduino.h"

class CF_lib {
	public:
		CF_lib();
        void begin(int* accel);
		float Compute_Pitch(int* accel, int gyro_y, int data_noise);
	
	private:
		float compCoeff = 0.98;
		int prev_gyro_y = 0;
		float pitch;
		float pitchGyro;
		unsigned long timer;
};

#endif