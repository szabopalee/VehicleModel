#ifndef VEHICLEMODEL_T
#define VEHICLEMODEL_T

typedef enum
{
	VehicleGear_INVALID = 0x00u,
	VehicleGear_Park = 0x01u,
	VehicleGear_Reverse = 0x02u,
	VehicleGear_Drive = 0x03u,
	VehicleGear_Neutral = 0x04u,
	VehicleGear_EngineBrake = 0x05u,
	VehicleGear_Sport = 0x06u
}vehicleGear_t;

// typedef enum kocsi unknown camry

typedef struct
{
	float accPedalAccelerationRatio;
	float brakePedalDecelerationRatio;
	float steeringRatio;
	float maxSteeringWheelAngle;
	float maxSpeed;
	float wheelbase;
	
	// ide, hogy milyen auto? kulonbozo mukodesekhez...

	float accPedalPos;
	float brakePedalPos;
	float swAngle;
	vehicleGear_t gear;

	float longAcc;
	float velocity;
	float odometer;
	float yawrate;
	float latAcc;
}vehicleModel_t;



void vehicleModel_update(vehicleModel_t *VehicleModel);
void vehicleModel_init(vehicleModel_t *VehicleModel, float acc_pedal_ratio, float brake_pedal_ratio, float steering_ratio, float max_swangle, float max_speed, float wheelbase);
void vehicleModel_printState(vehicleModel_t VehicleModel);
// ezek visszajelezzenek valahogy visszatérési értékben, hogy valami hiba volt?
// fn_return_t-vel

void vehicleModel_setAccPedalPos(vehicleModel_t *VehicleModel, float acc_pedal_pos);
void vehicleModel_setBrakePedalPos(vehicleModel_t *VehicleModel, float brake_pedal_pos);
void vehicleModel_setSteeringWheelAngle(vehicleModel_t *VehicleModel, float sw_angle);
void vehicleModel_setGear(vehicleModel_t *VehicleModel, vehicleGear_t gear);

#endif