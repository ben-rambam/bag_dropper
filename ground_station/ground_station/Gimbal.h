#pragma once
#include "Serial.h"
#include <iostream>
#include "MAVlink_include_checksum.h"

class Gimbal
{
public:
	Gimbal(CSerial &_serial);

	bool setAngle(float pitch, float roll, float yaw);

private:
	CSerial * serial;
};