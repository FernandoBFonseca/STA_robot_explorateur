#ifndef _ROS_dsr_msgs_RobotStop_h
#define _ROS_dsr_msgs_RobotStop_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class RobotStop : public ros::Msg
  {
    public:
      typedef int32_t _stop_mode_type;
      _stop_mode_type stop_mode;

    RobotStop():
      stop_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_stop_mode;
      u_stop_mode.real = this->stop_mode;
      *(outbuffer + offset + 0) = (u_stop_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stop_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stop_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stop_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_stop_mode;
      u_stop_mode.base = 0;
      u_stop_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stop_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stop_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stop_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stop_mode = u_stop_mode.real;
      offset += sizeof(this->stop_mode);
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/RobotStop"; };
    virtual const char * getMD5() override { return "82712390efeed0d0668a551e004b332c"; };

  };

}
#endif
