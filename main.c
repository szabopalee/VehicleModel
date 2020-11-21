#include <stdio.h>

#include "VehicleModel.h"


#define ACC_PEDAL_R 60.0
#define BRAKE_PEDAL_R 81.0
#define STEERING_R 13.7
#define MAX_SPEED 62.0
#define WHEELBASE 2.825
#define MAX_SWANGLE 8.0

// accPedalPos		acc (m/s^2)   -> [0,1] * 60
// 0				0
// 1				60

// brakePedalPos	dec (m/s^2)   -> [0,1] * 81
// 0				0
// 1				81


int main()
{
	float app, bpp, swa;
	int g;
	vehicleModel_t v1;
	vehicleModel_init(&v1, ACC_PEDAL_R, BRAKE_PEDAL_R, STEERING_R, MAX_SWANGLE, MAX_SPEED, WHEELBASE);
	while (1)
	{
		scanf_s("%f %f %f %d", &app, &bpp, &swa, &g);
		vehicleModel_update(&v1);
		vehicleModel_printState(v1);
	}
}