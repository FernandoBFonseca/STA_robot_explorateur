#ifndef _ROS_dsr_msgs_LogAlarm_h
#define _ROS_dsr_msgs_LogAlarm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class LogAlarm : public ros::Msg
  {
    public:
      typedef int32_t _level_type;
      _level_type level;
      typedef int32_t _group_type;
      _group_type group;
      typedef int32_t _index_type;
      _index_type index;
      char* param[3];

    LogAlarm():
      level(0),
      group(0),
      index(0),
      param()
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
      } u_index;
      u_index.real = this->index;
      *(outbuffer + offset + 0) = (u_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index);
      for( uint32_t i = 0; i < 3; i++){
      uint32_t length_parami = strlen(this->param[i]);
      varToArr(outbuffer + offset, length_parami);
      offset += 4;
      memcpy(outbuffer + offset, this->param[i], length_parami);
      offset += length_parami;
      }
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
      } u_index;
      u_index.base = 0;
      u_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->index = u_index.real;
      offset += sizeof(this->index);
      for( uint32_t i = 0; i < 3; i++){
      uint32_t length_parami;
      arrToVar(length_parami, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parami; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parami-1]=0;
      this->param[i] = (char *)(inbuffer + offset-1);
      offset += length_parami;
      }
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/LogAlarm"; };
    virtual const char * getMD5() override { return "c127c7c1149264259595eb8c3ff9972c"; };

  };

}
#endif
