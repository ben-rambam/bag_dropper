#include "Gimbal.h"

Gimbal::Gimbal(CSerial &_serial)
{
	serial = &_serial;
}

bool Gimbal::setAngle(float pitch, float roll, float yaw)
{
	char* message;

	union {
		char c[19];
		struct {
			char startsign;
			char length;
			char command;
			float pitch;
			float roll;
			float yaw;
			char flagsByte;
			char typeByte;
			char crcLo;
			char crcHi;
		} s;
	} angleMessage;

	angleMessage.s.startsign = 0xFA;
	angleMessage.s.length = 0x0E;
	angleMessage.s.command = 0x11;
	angleMessage.s.pitch = pitch;
	angleMessage.s.roll = roll;
	angleMessage.s.yaw = yaw;
	angleMessage.s.flagsByte = 0x07;
	angleMessage.s.typeByte = 0;

	uint16_t crc = crc_calculate((uint8_t*)&angleMessage.c[3], 14);
	angleMessage.s.crcHi = crc >> 8;
	angleMessage.s.crcLo = (uint8_t)(crc & 0xff);

	message = angleMessage.c;

	if (serial->IsOpened())
	{
		char returned[50];
		serial->SendData(message, strlen(message));
		int numWaiting = serial->ReadDataWaiting();
		while (numWaiting < 1)
		{
			std::cout << "got " << numWaiting << " bytes of ack" << std::endl;
			//continue;
			numWaiting = serial->ReadDataWaiting();
		}
		serial->ReadData(returned, 50);
		std::cout << "returned: " << returned << std::endl;
		return true;
	}
	return false;
}