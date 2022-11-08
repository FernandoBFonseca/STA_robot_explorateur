#ifndef _ROS_dsr_msgs_TorqueRTStream_h
#define _ROS_dsr_msgs_TorqueRTStream_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class TorqueRTStream : public ros::Msg
  {
    public:
      float tor[6];
      typedef float _time_type;
      _time_type time;

    TorqueRTStream():
      tor(),
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tor[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tor[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/TorqueRTStream"; };
    virtual const char * getMD5() override { return "f87591a62093bf8e8f3ad8edb630f87d"; };

  };

}
#endif
