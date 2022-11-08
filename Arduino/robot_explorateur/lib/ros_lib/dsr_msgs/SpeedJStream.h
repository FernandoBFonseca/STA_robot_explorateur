#ifndef _ROS_dsr_msgs_SpeedJStream_h
#define _ROS_dsr_msgs_SpeedJStream_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class SpeedJStream : public ros::Msg
  {
    public:
      float vel[6];
      float acc[6];
      typedef float _time_type;
      _time_type time;

    SpeedJStream():
      vel(),
      acc(),
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->vel[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->acc[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vel[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/SpeedJStream"; };
    virtual const char * getMD5() override { return "e4ccbedcf19c8719d28eda24c562383b"; };

  };

}
#endif
