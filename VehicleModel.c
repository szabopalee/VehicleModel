#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "VehicleModel.h"

float timeduration = 50e-3;

static void update_longAcc(vehicleModel_t *VehicleModel)
{
	if (VehicleModel->gear == VehicleGear_Park)
	{
		VehicleModel->longAcc = 0;
	}
	else if (VehicleModel->gear == VehicleGear_Reverse)
	{
		VehicleModel->longAcc = -(VehicleModel->accPedalPos * VehicleModel->accPedalAccelerationRatio) + (VehicleModel->brakePedalPos * VehicleModel->brakePedalDecelerationRatio);
		if (VehicleModel->longAcc > 0)
		{
			VehicleModel->longAcc = 0;
		}
	}
	else if(VehicleModel->gear == VehicleGear_Drive || VehicleModel->gear == VehicleGear_EngineBrake || VehicleModel->gear == VehicleGear_Sport)
	{
		VehicleModel->longAcc = (VehicleModel->accPedalPos * VehicleModel->accPedalAccelerationRatio) - (VehicleModel->brakePedalPos * VehicleModel->brakePedalDecelerationRatio);
	}
	else if (VehicleModel->gear == VehicleGear_INVALID || VehicleModel->gear ==  VehicleGear_Neutral)
	{
		VehicleModel->longAcc = - (VehicleModel->brakePedalPos * VehicleModel->brakePedalDecelerationRatio);
	}
}

static void update_velocity(vehicleModel_t *VehicleModel)
{
	if (VehicleModel->velocity + timeduration * VehicleModel->longAcc > VehicleModel->maxSpeed)
	{
		VehicleModel->velocity = VehicleModel->maxSpeed;
	}
	else if (VehicleModel->velocity + timeduration * VehicleModel->longAcc < 0 && VehicleModel->gear != VehicleGear_Reverse)
	{
		VehicleModel->velocity = 0;
	}
	else
	{
		VehicleModel->velocity += timeduration * VehicleModel->longAcc;
	}
}

static void update_latAcc(vehicleModel_t *VehicleModel)
{
	float swa = VehicleModel->swAngle;

	float wheelAngle = swa / VehicleModel->steeringRatio;
	float r = VehicleModel->wheelbase / sin(wheelAngle);
	float acp = ( (VehicleModel->velocity) * (VehicleModel->velocity) ) / r;

	VehicleModel->latAcc = acp;
}

static void update_odometer(vehicleModel_t *VehicleModel)
{
	VehicleModel->odometer += (VehicleModel->velocity) * timeduration;
}

static void update_yawrate(vehicleModel_t *VehicleModel)
{
	float swa = VehicleModel->swAngle;
	
	float wheelAngle = swa / VehicleModel->steeringRatio;
	float r = VehicleModel->wheelbase / sin(wheelAngle);
	float yaw = (VehicleModel->velocity) / r;

	VehicleModel->yawrate = yaw;

}

void vehicleModel_update(vehicleModel_t *VehicleModel)
{
	update_longAcc(VehicleModel);
	update_velocity(VehicleModel);
	update_latAcc(VehicleModel);
	update_odometer(VehicleModel);
	update_yawrate(VehicleModel);
}

void vehicleModel_init(vehicleModel_t *VehicleModel, float acc_pedal_ratio, float brake_pedal_ratio, float steering_ratio, float max_swangle, float max_speed, float wheelbase)
{
	VehicleModel->accPedalPos = 0.0;
	VehicleModel->brakePedalPos = 0.0;
	VehicleModel->swAngle = 0.0;
	VehicleModel->gear = VehicleGear_Neutral;
	
	VehicleModel->longAcc = 0.0;
	VehicleModel->velocity = 0.0;
	VehicleModel->latAcc = 0.0;
	VehicleModel->odometer = 0.0;
	VehicleModel->yawrate = 0.0;
	
	VehicleModel->accPedalAccelerationRatio = acc_pedal_ratio;
	VehicleModel->brakePedalDecelerationRatio = brake_pedal_ratio;
	VehicleModel->steeringRatio = steering_ratio;
	VehicleModel->maxSteeringWheelAngle = max_swangle;
	VehicleModel->maxSpeed = max_speed;
	VehicleModel->wheelbase = wheelbase;
}

void vehicleModel_printState(vehicleModel_t VehicleModel)
{
	printf("AccelPedalPos: %f\n", VehicleModel.accPedalPos);
	printf("BrakePedalPos: %f\n", VehicleModel.brakePedalPos);
	printf("SteeringWheelAngle: %f rad\n", VehicleModel.swAngle);
	printf("Gear: %d\n", VehicleModel.gear);
	printf("LongAcc: %f m/s^2\n", VehicleModel.longAcc);
	printf("Velocity: %f km/h\n", VehicleModel.velocity*3.6);
	printf("LatAcc: %f m/s^2\n", VehicleModel.latAcc);
	printf("Odometer: %f m\n", VehicleModel.odometer);
	printf("YawRate: %f rad/s\n", VehicleModel.yawrate);
}

void vehicleModel_setAccPedalPos(vehicleModel_t *VehicleModel, float acc_pedal_pos)
{
	if (acc_pedal_pos <= 1 && acc_pedal_pos >= 0)
	{
		VehicleModel->accPedalPos = acc_pedal_pos;
	}
}

void vehicleModel_setBrakePedalPos(vehicleModel_t *VehicleModel, float brake_pedal_pos)
{
	if (brake_pedal_pos <= 1 && brake_pedal_pos >= 0)
	{
		VehicleModel->brakePedalPos = brake_pedal_pos;
	}
}

void vehicleModel_setSteeringWheelAngle(vehicleModel_t *VehicleModel, float sw_angle)
{

	if (sw_angle <= VehicleModel->maxSteeringWheelAngle && sw_angle >= -(VehicleModel->maxSteeringWheelAngle))
	{
		VehicleModel->swAngle = sw_angle;
	}
}

void vehicleModel_setGear(vehicleModel_t *VehicleModel, vehicleGear_t gear)
{
	if (gear == VehicleGear_Park || gear == VehicleGear_Reverse)
	{
		if(VehicleModel->velocity == 0.0) // valamilyen kicsi rangen belül kéne nézni?
			VehicleModel->gear = gear;
	}
	else
	{
		if (VehicleModel->velocity >= 0) // ez jó így?
		{
			VehicleModel->gear = gear;
		}

		//-3 és 3 között
	}
}