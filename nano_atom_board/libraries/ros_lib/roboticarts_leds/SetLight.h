#ifndef _ROS_SERVICE_SetLight_h
#define _ROS_SERVICE_SetLight_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roboticarts_leds
{

static const char SETLIGHT[] = "roboticarts_leds/SetLight";

  class SetLightRequest : public ros::Msg
  {
    public:
      typedef uint8_t _brightness_type;
      _brightness_type brightness;

    SetLightRequest():
      brightness(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->brightness >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brightness);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->brightness =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->brightness);
     return offset;
    }

    const char * getType(){ return SETLIGHT; };
    const char * getMD5(){ return "0374114287947f81d241e42c7a34c37e"; };

  };

  class SetLightResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetLightResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return SETLIGHT; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetLight {
    public:
    typedef SetLightRequest Request;
    typedef SetLightResponse Response;
  };

}
#endif
