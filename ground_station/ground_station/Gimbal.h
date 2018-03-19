#pragma once
#include "Serial.h"
#include <iostream>
#include "MAVlink_include_checksum.h"

class Gimbal
{
public:
	Gimbal(CSerial &_serial);

	bool setAngle(float pitch, float roll, float yaw);
    bool setRCInputs( uint16_t pitch, uint16_t roll, uint16_t yaw );
    bool getParameter( uint16_t &parametervalue, uint16_t index );
	void requestAttitude();
	void requestMountStatusInterval(uint16_t interval_us);
	void readMessage();
	void requestAttitudeInterval(int32_t interval_us);
	float gimbalYaw;
	float gimbalPitch;
	float gimbalRoll;

private:
	CSerial * serial;
};