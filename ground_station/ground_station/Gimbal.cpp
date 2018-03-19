#include "Gimbal.h"
#include <stdio.h>
#include "mavlink\include\mavlink_types.h"
#include "mavlink\include\mavlink.h"
#include <iostream>
using namespace std;

#define ToDeg(x) (x*57.2957795131)  // *180/pi

Gimbal::Gimbal(CSerial &_serial)
{
	serial = &_serial;
}

bool Gimbal::setAngle(float pitch, float roll, float yaw)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 205, 0, pitch, roll, yaw, 0.0, 0.0, 0.0, 0.0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	serial->SendData(buf, len);

	return true;
}

void Gimbal::requestAttitude() 
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 1234, 0, 0, 0, 0, 0, 0, 0, 0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	serial->SendData(buf, len);
}

void Gimbal::requestAttitudeInterval(int32_t interval_us)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	union Accessor
	{
		float f;
		uint16_t u16;
		int32_t i32;
	} messageIndex, messageInterval;
	messageInterval.i32 = interval_us;
	messageIndex.f = 30;
	mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 244, 0, messageIndex.f, messageInterval.f, 0, 0, 0, 0, 0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	serial->SendData(buf, len);
}

void Gimbal::requestMountStatusInterval(uint16_t interval_us)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_request_data_stream_pack(255, 1, &msg, 71, 67, 158, interval_us, 1);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	serial->SendData(buf, len);
}

void Gimbal::readMessage() 
{

	mavlink_message_t msg;
	mavlink_status_t status;

	while ( serial->ReadDataWaiting() > 0) 
	{
		uint8_t buf[256];
		serial->ReadData(buf, 1);
		uint8_t c = buf[0];
		//cout << "c:\t" << c << endl;
		//trying to grab msg
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			//printf("msg.msgid: %d", msg.msgid);
			switch (msg.msgid) {
			case MAVLINK_MSG_ID_ATTITUDE:
			{
				printf("ATTITUDE: %d\n", MAVLINK_MSG_ID_ATTITUDE);
				//get pitch and yaw angle from storm (requestAttitude() must be executed first)
				gimbalYaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
				gimbalPitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
				gimbalRoll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
			}
			break;

			case MAVLINK_MSG_ID_PARAM_VALUE:
			{
				//get parameter value from storm (parameter 66 is pan mode, requestParameter(int id) must be executed first)
				if (mavlink_msg_param_value_get_param_index(&msg) == 66)
					int panMode = mavlink_msg_param_value_get_param_value(&msg);
			}
			break;
			case 158:
			{

			}
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				cout << "HEARTBEAT\n";
			}
			break;
			default:
				break;
			}
		}
	}

}

/*
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
	angleMessage.s.length = 0x00;
	angleMessage.s.command = 0x02;
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
        std::cout << strlen(message) << std::endl;
		serial->SendData(message, strlen(message));
		int numWaiting = serial->ReadDataWaiting();
        //numWaiting is 
		while (numWaiting < 1)
		{
			//std::cout << "got " << numWaiting << " bytes of ack" << std::endl;
			//continue;
			numWaiting = serial->ReadDataWaiting();
		}
		serial->ReadData(returned, 50);
		std::cout << "returned: " << returned << std::endl;
		return true;
	}
	return false;
}

*/
bool Gimbal::setRCInputs(uint16_t pitch, uint16_t roll, uint16_t yaw)
{
	uint8_t* message;

	union {
		char c[11];
        uint8_t u[11];
		struct {
			uint8_t startsign;
			uint8_t length;
			uint8_t command;
			uint8_t pitch_lo;
            uint8_t pitch_hi;
            uint8_t roll_lo;
            uint8_t roll_hi;
            uint8_t yaw_lo;
            uint8_t yaw_hi;
			uint8_t crc_lo;
			uint8_t crc_hi;
            
		} s;
        unsigned long long i;
	} angleMessage;

	angleMessage.s.startsign = 0xFA;
	angleMessage.s.length = 0x06;
	angleMessage.s.command = 0x12;
	angleMessage.s.pitch_lo = pitch;
    angleMessage.s.pitch_hi = pitch>>8;

	angleMessage.s.roll_lo = roll;
    angleMessage.s.roll_hi = roll>>8;
	angleMessage.s.yaw_lo = yaw;
    angleMessage.s.yaw_hi = yaw>>8;

	uint16_t crc = crc_calculate((uint8_t*)&angleMessage.c[3], 6);
	angleMessage.s.crc_hi = crc >> 8;
	angleMessage.s.crc_lo = (uint8_t)(crc & 0xff);

	message = angleMessage.u;

	if (serial->IsOpened())
	{

		char returned[50];
   
       

        for( int i = 0; i < 11; i++ )
        printf( "%x \n", angleMessage.u[i] );
        
		serial->SendData(message, 11);
		int numWaiting = serial->ReadDataWaiting();
        //numWaiting is 
		while (numWaiting < 1)
		{
			//std::cout << "got " << numWaiting << " bytes of ack" << std::endl;
			//continue;
			numWaiting = serial->ReadDataWaiting();
		}
		serial->ReadData(returned, 50);
        std::cout << "returned message: " << std::endl;
        for( int i = 0; i < 6; i++)
        {
            
            printf( "%x \n", returned[i]);

        }
		//std::cout << "returned: " << returned << std::endl;
		return true;
	}
	return false;
}


bool Gimbal::getParameter(uint16_t &parametervalue, uint16_t index)
{
    uint8_t* message;

	union {
		char c[11];
        uint8_t u[11];
		struct {
			uint8_t startsign;
			uint8_t length;
			uint8_t command;
			uint8_t datalo;
            uint8_t datahi;
			uint8_t crc_lo;
			uint8_t crc_hi;
            
		} s;
        unsigned long long i;
	} angleMessage;

	angleMessage.s.startsign = 0xFA;
	angleMessage.s.length = 0x02;
	angleMessage.s.command = 0x03;
    angleMessage.s.datahi = index >> 8;
    angleMessage.s.datalo = index;
	uint16_t crc = crc_calculate((uint8_t*)&angleMessage.c[3], 6);
	angleMessage.s.crc_hi = crc >> 8;
	angleMessage.s.crc_lo = (uint8_t)(crc & 0xff);

	message = (uint8_t*)(angleMessage.c);

	if (serial->IsOpened())
	{

		char returned[50];
   
       

        for( int i = 0; i < 11; i++ )
        printf( "%x \n", angleMessage.u[i] );
        
		serial->SendData(message, 11);
		int numWaiting = serial->ReadDataWaiting();
        //numWaiting is 
		while (numWaiting < 1)
		{
			//std::cout << "got " << numWaiting << " bytes of ack" << std::endl;
			//continue;
			numWaiting = serial->ReadDataWaiting();
		}
		serial->ReadData(returned, 50);
        parametervalue = returned[5] | (returned[6]<<8);
        printf( "%u \n ", returned[5] );
        printf( "%u \n", returned[6] );
        std::cout << "returned message: " << std::endl;
        for( int i = 0; i < 6; i++)
        {
            
            printf( "%x \n", returned[i]);

        }

		//std::cout << "returned: " << returned << std::endl;
		return true;
	}
	return false;
}
