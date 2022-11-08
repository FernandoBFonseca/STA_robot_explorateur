#ifndef _ROS_dsr_msgs_ModbusState_h
#define _ROS_dsr_msgs_ModbusState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dsr_msgs
{

  class ModbusState : public ros::Msg
  {
    public:
      typedef const char* _modbus_symbol_type;
      _modbus_symbol_type modbus_symbol;
      typedef int32_t _modbus_value_type;
      _modbus_value_type modbus_value;

    ModbusState():
      modbus_symbol(""),
      modbus_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_modbus_symbol = strlen(this->modbus_symbol);
      varToArr(outbuffer + offset, length_modbus_symbol);
      offset += 4;
      memcpy(outbuffer + offset, this->modbus_symbol, length_modbus_symbol);
      offset += length_modbus_symbol;
      union {
        int32_t real;
        uint32_t base;
      } u_modbus_value;
      u_modbus_value.real = this->modbus_value;
      *(outbuffer + offset + 0) = (u_modbus_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_modbus_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_modbus_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_modbus_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->modbus_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_modbus_symbol;
      arrToVar(length_modbus_symbol, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_modbus_symbol; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_modbus_symbol-1]=0;
      this->modbus_symbol = (char *)(inbuffer + offset-1);
      offset += length_modbus_symbol;
      union {
        int32_t real;
        uint32_t base;
      } u_modbus_value;
      u_modbus_value.base = 0;
      u_modbus_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_modbus_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_modbus_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_modbus_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->modbus_value = u_modbus_value.real;
      offset += sizeof(this->modbus_value);
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/ModbusState"; };
    virtual const char * getMD5() override { return "c941aa5cf7a235f21e3789f2d6e5305f"; };

  };

}
#endif
