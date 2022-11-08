#ifndef _ROS_dsr_msgs_RobotError_h
#define _ROS_dsr_msgs_RobotError_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class RobotError : public ros::Msg
  {
    public:
      typedef int32_t _level_type;
      _level_type level;
      typedef int32_t _group_type;
      _group_type group;
      typedef int32_t _code_type;
      _code_type code;
      typedef const char* _msg1_type;
      _msg1_type msg1;
      typedef const char* _msg2_type;
      _msg2_type msg2;
      typedef const char* _msg3_type;
      _msg3_type msg3;

    RobotError():
      level(0),
      group(0),
      code(0),
      msg1(""),
      msg2(""),
      msg3("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_level;
      u_level.real = this->level;
      *(outbuffer + offset + 0) = (u_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->level);
      union {
        int32_t real;
        uint32_t base;
      } u_group;
      u_group.real = this->group;
      *(outbuffer + offset + 0) = (u_group.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_group.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_group.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_group.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->group);
      union {
        int32_t real;
        uint32_t base;
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->code);
      uint32_t length_msg1 = strlen(this->msg1);
      varToArr(outbuffer + offset, length_msg1);
      offset += 4;
      memcpy(outbuffer + offset, this->msg1, length_msg1);
      offset += length_msg1;
      uint32_t length_msg2 = strlen(this->msg2);
      varToArr(outbuffer + offset, length_msg2);
      offset += 4;
      memcpy(outbuffer + offset, this->msg2, length_msg2);
      offset += length_msg2;
      uint32_t length_msg3 = strlen(this->msg3);
      varToArr(outbuffer + offset, length_msg3);
      offset += 4;
      memcpy(outbuffer + offset, this->msg3, length_msg3);
      offset += length_msg3;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_level;
      u_level.base = 0;
      u_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->level = u_level.real;
      offset += sizeof(this->level);
      union {
        int32_t real;
        uint32_t base;
      } u_group;
      u_group.base = 0;
      u_group.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_group.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_group.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_group.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->group = u_group.real;
      offset += sizeof(this->group);
      union {
        int32_t real;
        uint32_t base;
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->code = u_code.real;
      offset += sizeof(this->code);
      uint32_t length_msg1;
      arrToVar(length_msg1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg1-1]=0;
      this->msg1 = (char *)(inbuffer + offset-1);
      offset += length_msg1;
      uint32_t length_msg2;
      arrToVar(length_msg2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg2-1]=0;
      this->msg2 = (char *)(inbuffer + offset-1);
      offset += length_msg2;
      uint32_t length_msg3;
      arrToVar(length_msg3, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg3; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg3-1]=0;
      this->msg3 = (char *)(inbuffer + offset-1);
      offset += length_msg3;
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/RobotError"; };
    virtual const char * getMD5() override { return "2d9680ace6867f719d7e19bbc321e6da"; };

  };

}
#endif
