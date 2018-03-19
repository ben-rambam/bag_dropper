#include "mavlink/include/mavlink_types.h"
#include "mavlink/include/mavlink.h"

#define ToDeg(x) (x*57.2957795131)  // *180/pi
/*
void setup(){
  Serial.begin(115200);
}

void loop(){   

  read_mavlink_storm32();
}


void read_mavlink_storm32(){ 
  
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while (Serial.available() > 0) {
    
    uint8_t c = Serial.read();
    //trying to grab msg
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {   
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_ATTITUDE:
          {
            //get pitch and yaw angle from storm (requestAttitude() must be executed first)
            double gimbalYaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
            double gimbalPitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
          }
          break;
          
        case MAVLINK_MSG_ID_PARAM_VALUE:
          {
            //get parameter value from storm (parameter 66 is pan mode, requestParameter(int id) must be executed first)
            if(mavlink_msg_param_value_get_param_index(&msg) == 66)
              int panMode = mavlink_msg_param_value_get_param_value(&msg);
          }
          break;
        default:
          break;
      }
    }  
  }
  
}

void requestAttitude(){

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 1234, 0, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
 
}

void requestParameter(int id){
     
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_request_read_pack(255, 1, &msg, 71, 67, "", id);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);   
  
}

void setParameter(int id, int val){

    union intFloat
    {
        int i;
        float f;
    } parameterValue;
    parameterValue.i = val;
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 180, 0, id, parameterValue.f, 0.0, 0.0, 0.0, 0.0, 0.0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len); 
      
}


void setAngles(float roll, float pitch, float yaw){
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 205, 0, pitch, roll, yaw, 0.0, 0.0, 0.0, 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
}

void recenter(){
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(255, 1, &msg, 71, 67, 204, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  
}

*/
