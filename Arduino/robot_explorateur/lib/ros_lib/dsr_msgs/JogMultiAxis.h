#ifndef _ROS_dsr_msgs_JogMultiAxis_h
#define _ROS_dsr_msgs_JogMultiAxis_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class JogMultiAxis : public ros::Msg
  {
    public:
      float jog_axis[6];
      typedef int8_t _move_reference_type;
      _move_reference_type move_reference;
      typedef float _speed_type;
      _speed_type speed;

    JogMultiAxis():
      jog_axis(),
      move_reference(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->jog_axis[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_move_reference;
      u_move_reference.real = this->move_reference;
      *(outbuffer + offset + 0) = (u_move_reference.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->move_reference);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->jog_axis[i]));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_move_reference;
      u_move_reference.base = 0;
      u_move_reference.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->move_reference = u_move_reference.real;
      offset += sizeof(this->move_reference);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/JogMultiAxis"; };
    virtual const char * getMD5() override { return "e36e615ff2ffad91791b721818ab6be9"; };

  };

}
#endif
