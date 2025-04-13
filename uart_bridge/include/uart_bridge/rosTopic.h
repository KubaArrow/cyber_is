/*
 * rosTopic.h
 *
 *  Created on: Mar 24, 2025
 *      Author: bkfcb
 */

#ifndef INC_ROSTOPIC_H_
#define INC_ROSTOPIC_H_

#include <stdint.h>

typedef enum
{
    IMU_MSG = 0x30,
    TRACKER_SENSOR_MSG,
    CMD_VEL_MSG,
	BATTERY_STATE_MSG,
	MAGNET_MSG,
	ODOMETRY_MSG,
	LED_MSG,
	DIAG_MSG,
 	POSE_MSG
}msgID_t;
#pragma pack(1)

typedef struct
{
    uint32_t seq;
    uint32_t stamp_sec;
    uint32_t stamp_nsec;
} Header_t;

typedef struct
{
    double x;
    double y;
    double z;
    double w;
} Quaternion_t;

typedef struct
{
    double x;
    double y;
    double z;
} Vector3_t;

typedef struct
{
    uint8_t msgID;
    Header_t header;
    Quaternion_t orientation;
    Vector3_t angular_velocity;
    Vector3_t linear_acceleration;
} ImuMsg_t;

typedef struct
{
    uint8_t msgID;
    uint16_t data[5];
} TrackerSensorMsg_t;

typedef struct
{
    uint8_t msgID;
    Vector3_t linear;
    Vector3_t angular;
}CmdVelMsg_t;

typedef struct
{
	uint8_t msgID;
	float voltage;
	float current;
	double energy;
	float percentage;
}BatteryStateMsg_t;

typedef struct
{
	uint8_t msgID;
	Vector3_t magnet;
}MagnetMsg_t;

typedef struct
{
	uint8_t msgID;
    Header_t header;
	Vector3_t position;
	Quaternion_t orientation;
	Vector3_t linear;
	Vector3_t angular;
} OdometryMsg_t;

typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
	uint8_t A;
}Color_t;

typedef struct
{
	uint8_t msgID;
	Color_t leds[12];
}LEDMsg_t;

typedef struct
{
	uint8_t msgID;
	uint8_t level; // 0 OK/2 ERROR
	uint8_t nameLen;
	uint8_t name[10];
	uint8_t msgLen;
	uint8_t msg[100];
}DiagMsg_t;

typedef struct
{
	uint8_t msgID;
	double x;
	double y;
	double yaw;
}PoseMsg_t;
#pragma pack()


uint8_t publishTrackerSensor();
uint8_t publishIMU();
uint8_t publishBatteryState();
uint8_t publishMagnet();
void updateOdometry();
uint8_t publishOdometry();

void processCmdVel(uint8_t *newCmdVelMsg);
void processLED(uint8_t *newLEDMsg);


#endif /* INC_ROSTOPIC_H_ */