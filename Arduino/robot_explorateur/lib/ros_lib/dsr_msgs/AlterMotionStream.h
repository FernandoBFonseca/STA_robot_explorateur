#ifndef _ROS_dsr_msgs_AlterMotionStream_h
#define _ROS_dsr_msgs_AlterMotionStream_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class AlterMotionStream : public ros::Msg
  {
    public:
      float pos[6];

    AlterMotionStream():
      pos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pos[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/AlterMotionStream"; };
    virtual const char * getMD5() override { return "537431324117c3d1847d70e057990155"; };

  };

}
#endif
