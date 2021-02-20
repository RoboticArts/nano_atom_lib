#ifndef _ROS_SERVICE_SetLeds_h
#define _ROS_SERVICE_SetLeds_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roboticarts_leds
{

static const char SETLEDS[] = "roboticarts_leds/SetLeds";

  class SetLedsRequest : public ros::Msg
  {
    public:
      typedef const char* _signal_type;
      _signal_type signal;
      typedef bool _enable_type;
      _enable_type enable;

    SetLedsRequest():
      signal(""),
      enable(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_signal = strlen(this->signal);
      varToArr(outbuffer + offset, length_signal);
      offset += 4;
      memcpy(outbuffer + offset, this->signal, length_signal);
      offset += length_signal;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_signal;
      arrToVar(length_signal, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_signal; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_signal-1]=0;
      this->signal = (char *)(inbuffer + offset-1);
      offset += length_signal;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
     return offset;
    }

    const char * getType(){ return SETLEDS; };
    const char * getMD5(){ return "b163e5cf74e2286cd22431636c2fb1d6"; };

  };

  class SetLedsResponse : public ros::Msg
  {
    public:
      typedef const char* _message_type;
      _message_type message;
      typedef bool _success_type;
      _success_type success;

    SetLedsResponse():
      message(""),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETLEDS; };
    const char * getMD5(){ return "9bf829f07d795d3f9e541a07897da2c4"; };

  };

  class SetLeds {
    public:
    typedef SetLedsRequest Request;
    typedef SetLedsResponse Response;
  };

}
#endif
