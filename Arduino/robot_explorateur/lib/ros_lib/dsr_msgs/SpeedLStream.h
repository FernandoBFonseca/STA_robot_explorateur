#ifndef _ROS_dsr_msgs_SpeedLStream_h
#define _ROS_dsr_msgs_SpeedLStream_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class SpeedLStream : public ros::Msg
  {
    public:
      float vel[6];
      float acc[2];
      typedef float _time_type;
      _time_type time;

    SpeedLStream():
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
      for( uint32_t i = 0; i < 2; i++){
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
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/SpeedLStream"; };
    virtual const char * getMD5() override { return "e46b9dfc7c2bf557dfdf28c9ac4104b9"; };

  };

}
#endif
