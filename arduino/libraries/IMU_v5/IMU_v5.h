#ifndef IMU_v5_h
#define IMU_v5_h
#include "Arduino.h"
#include <LSM6.h>


class IMU_v5 {
	public:
		IMU_v5();
		
		void begin(LSM6* IMU);
		void getOffsetNoise();
				
		int* Read_Acc();
		int* Read_Gyro();
		int* Get_Noise();

	private:
		LSM6* _IMU;

		int acc_data[3] = {0, 0, 0};
		int gyro_data[3] = {0, 0, 0};

		int ArrayAngles[6] = {0, 0, 0, 0, 0, 0,};
		int ArrayOffset[6] = {0, 0, 0, 0, 0, 0,};
		int ArrayNoise[6] = {0, 0, 0, 0, 0, 0,};
};

#endif 