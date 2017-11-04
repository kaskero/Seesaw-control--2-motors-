#include <LSM6.h>
#include <IMU_v5.h> 

LSM6 lsm6;
IMU_v5 IMU;

void setup(){
	Serial.begin(115200);
	Serial.println("enter setup()");
	
	IMU.begin(&lsm6);
	IMU.getOffsetNoise();

	Serial.println("exit setup()");
}

void loop(){
	int* acc_data = IMU.Read_Acc();
	Serial.print("accel_x: "); Serial.println(*acc_data);
	Serial.print("accel_y: "); Serial.println(*(acc_data+1));
	Serial.print("accel_z: "); Serial.println(*(acc_data+2));
	
	int* gyro_data = IMU.Read_Gyro();
	Serial.print("gyro_x: "); Serial.println(*gyro_data);
	Serial.print("gyro_y: "); Serial.println(*(gyro_data+1));
	Serial.print("gyro_z: "); Serial.println(*(gyro_data+2));
}