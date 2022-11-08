#ifndef _ROS_dsr_msgs_RobotStateRT_h
#define _ROS_dsr_msgs_RobotStateRT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"

namespace dsr_msgs
{

  class RobotStateRT : public ros::Msg
  {
    public:
      typedef float _time_stamp_type;
      _time_stamp_type time_stamp;
      float actual_joint_position[6];
      float actual_joint_position_abs[6];
      float actual_joint_velocity[6];
      float actual_joint_velocity_abs[6];
      float actual_tcp_position[6];
      float actual_tcp_velocity[6];
      float actual_flange_position[6];
      float actual_flange_velocity[6];
      float actual_motor_torque[6];
      float actual_joint_torque[6];
      float raw_joint_torque[6];
      float raw_force_torque[6];
      float external_joint_torque[6];
      float external_tcp_force[6];
      float target_joint_position[6];
      float target_joint_velocity[6];
      float target_joint_acceleration[6];
      float target_motor_torque[6];
      float target_tcp_position[6];
      float target_tcp_velocity[6];
      uint32_t jacobian_matrix_length;
      typedef std_msgs::Float64MultiArray _jacobian_matrix_type;
      _jacobian_matrix_type st_jacobian_matrix;
      _jacobian_matrix_type * jacobian_matrix;
      float gravity_torque[6];
      uint32_t coriolis_matrix_length;
      typedef std_msgs::Float64MultiArray _coriolis_matrix_type;
      _coriolis_matrix_type st_coriolis_matrix;
      _coriolis_matrix_type * coriolis_matrix;
      uint32_t mass_matrix_length;
      typedef std_msgs::Float64MultiArray _mass_matrix_type;
      _mass_matrix_type st_mass_matrix;
      _mass_matrix_type * mass_matrix;
      typedef uint16_t _solution_space_type;
      _solution_space_type solution_space;
      typedef float _singularity_type;
      _singularity_type singularity;
      typedef float _operation_speed_rate_type;
      _operation_speed_rate_type operation_speed_rate;
      float joint_temperature[6];
      typedef uint16_t _controller_digital_input_type;
      _controller_digital_input_type controller_digital_input;
      typedef uint16_t _controller_digital_output_type;
      _controller_digital_output_type controller_digital_output;
      uint8_t controller_analog_input_type[2];
      float controller_analog_input[2];
      uint8_t controller_analog_output_type[2];
      float controller_analog_output[2];
      typedef uint8_t _flange_digital_input_type;
      _flange_digital_input_type flange_digital_input;
      typedef uint8_t _flange_digital_output_type;
      _flange_digital_output_type flange_digital_output;
      float flange_analog_input[4];
      uint8_t external_encoder_strobe_count[2];
      uint16_t external_encoder_count[2];
      float goal_joint_position[6];
      float goal_tcp_position[6];
      typedef uint8_t _robot_mode_type;
      _robot_mode_type robot_mode;
      typedef uint8_t _robot_state_type;
      _robot_state_type robot_state;
      typedef uint16_t _control_mode_type;
      _control_mode_type control_mode;
      uint8_t reserved[256];

    RobotStateRT():
      time_stamp(0),
      actual_joint_position(),
      actual_joint_position_abs(),
      actual_joint_velocity(),
      actual_joint_velocity_abs(),
      actual_tcp_position(),
      actual_tcp_velocity(),
      actual_flange_position(),
      actual_flange_velocity(),
      actual_motor_torque(),
      actual_joint_torque(),
      raw_joint_torque(),
      raw_force_torque(),
      external_joint_torque(),
      external_tcp_force(),
      target_joint_position(),
      target_joint_velocity(),
      target_joint_acceleration(),
      target_motor_torque(),
      target_tcp_position(),
      target_tcp_velocity(),
      jacobian_matrix_length(0), st_jacobian_matrix(), jacobian_matrix(nullptr),
      gravity_torque(),
      coriolis_matrix_length(0), st_coriolis_matrix(), coriolis_matrix(nullptr),
      mass_matrix_length(0), st_mass_matrix(), mass_matrix(nullptr),
      solution_space(0),
      singularity(0),
      operation_speed_rate(0),
      joint_temperature(),
      controller_digital_input(0),
      controller_digital_output(0),
      controller_analog_input_type(),
      controller_analog_input(),
      controller_analog_output_type(),
      controller_analog_output(),
      flange_digital_input(0),
      flange_digital_output(0),
      flange_analog_input(),
      external_encoder_strobe_count(),
      external_encoder_count(),
      goal_joint_position(),
      goal_tcp_position(),
      robot_mode(0),
      robot_state(0),
      control_mode(0),
      reserved()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->time_stamp);
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_joint_position[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_joint_position_abs[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_joint_velocity[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_joint_velocity_abs[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_tcp_position[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_tcp_velocity[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_flange_position[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_flange_velocity[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_motor_torque[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_joint_torque[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->raw_joint_torque[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->raw_force_torque[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->external_joint_torque[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->external_tcp_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_joint_position[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_joint_velocity[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_joint_acceleration[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_motor_torque[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_tcp_position[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_tcp_velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->jacobian_matrix_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->jacobian_matrix_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->jacobian_matrix_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->jacobian_matrix_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->jacobian_matrix_length);
      for( uint32_t i = 0; i < jacobian_matrix_length; i++){
      offset += this->jacobian_matrix[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->gravity_torque[i]);
      }
      *(outbuffer + offset + 0) = (this->coriolis_matrix_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->coriolis_matrix_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->coriolis_matrix_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->coriolis_matrix_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->coriolis_matrix_length);
      for( uint32_t i = 0; i < coriolis_matrix_length; i++){
      offset += this->coriolis_matrix[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->mass_matrix_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mass_matrix_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mass_matrix_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mass_matrix_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mass_matrix_length);
      for( uint32_t i = 0; i < mass_matrix_length; i++){
      offset += this->mass_matrix[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->solution_space >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->solution_space >> (8 * 1)) & 0xFF;
      offset += sizeof(this->solution_space);
      offset += serializeAvrFloat64(outbuffer + offset, this->singularity);
      offset += serializeAvrFloat64(outbuffer + offset, this->operation_speed_rate);
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_temperature[i]);
      }
      *(outbuffer + offset + 0) = (this->controller_digital_input >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controller_digital_input >> (8 * 1)) & 0xFF;
      offset += sizeof(this->controller_digital_input);
      *(outbuffer + offset + 0) = (this->controller_digital_output >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controller_digital_output >> (8 * 1)) & 0xFF;
      offset += sizeof(this->controller_digital_output);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->controller_analog_input_type[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_analog_input_type[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->controller_analog_input[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->controller_analog_output_type[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_analog_output_type[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->controller_analog_output[i]);
      }
      *(outbuffer + offset + 0) = (this->flange_digital_input >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flange_digital_input);
      *(outbuffer + offset + 0) = (this->flange_digital_output >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flange_digital_output);
      for( uint32_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->flange_analog_input[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->external_encoder_strobe_count[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->external_encoder_strobe_count[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->external_encoder_count[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->external_encoder_count[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->external_encoder_count[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_joint_position[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_tcp_position[i]);
      }
      *(outbuffer + offset + 0) = (this->robot_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robot_mode);
      *(outbuffer + offset + 0) = (this->robot_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robot_state);
      *(outbuffer + offset + 0) = (this->control_mode >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->control_mode >> (8 * 1)) & 0xFF;
      offset += sizeof(this->control_mode);
      for( uint32_t i = 0; i < 256; i++){
      *(outbuffer + offset + 0) = (this->reserved[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_stamp));
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_joint_position[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_joint_position_abs[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_joint_velocity[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_joint_velocity_abs[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_tcp_position[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_tcp_velocity[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_flange_position[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_flange_velocity[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_motor_torque[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_joint_torque[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->raw_joint_torque[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->raw_force_torque[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->external_joint_torque[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->external_tcp_force[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_joint_position[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_joint_velocity[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_joint_acceleration[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_motor_torque[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_tcp_position[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_tcp_velocity[i]));
      }
      uint32_t jacobian_matrix_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      jacobian_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      jacobian_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      jacobian_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->jacobian_matrix_length);
      if(jacobian_matrix_lengthT > jacobian_matrix_length)
        this->jacobian_matrix = (std_msgs::Float64MultiArray*)realloc(this->jacobian_matrix, jacobian_matrix_lengthT * sizeof(std_msgs::Float64MultiArray));
      jacobian_matrix_length = jacobian_matrix_lengthT;
      for( uint32_t i = 0; i < jacobian_matrix_length; i++){
      offset += this->st_jacobian_matrix.deserialize(inbuffer + offset);
        memcpy( &(this->jacobian_matrix[i]), &(this->st_jacobian_matrix), sizeof(std_msgs::Float64MultiArray));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gravity_torque[i]));
      }
      uint32_t coriolis_matrix_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      coriolis_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      coriolis_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      coriolis_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->coriolis_matrix_length);
      if(coriolis_matrix_lengthT > coriolis_matrix_length)
        this->coriolis_matrix = (std_msgs::Float64MultiArray*)realloc(this->coriolis_matrix, coriolis_matrix_lengthT * sizeof(std_msgs::Float64MultiArray));
      coriolis_matrix_length = coriolis_matrix_lengthT;
      for( uint32_t i = 0; i < coriolis_matrix_length; i++){
      offset += this->st_coriolis_matrix.deserialize(inbuffer + offset);
        memcpy( &(this->coriolis_matrix[i]), &(this->st_coriolis_matrix), sizeof(std_msgs::Float64MultiArray));
      }
      uint32_t mass_matrix_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      mass_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      mass_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      mass_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->mass_matrix_length);
      if(mass_matrix_lengthT > mass_matrix_length)
        this->mass_matrix = (std_msgs::Float64MultiArray*)realloc(this->mass_matrix, mass_matrix_lengthT * sizeof(std_msgs::Float64MultiArray));
      mass_matrix_length = mass_matrix_lengthT;
      for( uint32_t i = 0; i < mass_matrix_length; i++){
      offset += this->st_mass_matrix.deserialize(inbuffer + offset);
        memcpy( &(this->mass_matrix[i]), &(this->st_mass_matrix), sizeof(std_msgs::Float64MultiArray));
      }
      this->solution_space =  ((uint16_t) (*(inbuffer + offset)));
      this->solution_space |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->solution_space);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->singularity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->operation_speed_rate));
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_temperature[i]));
      }
      this->controller_digital_input =  ((uint16_t) (*(inbuffer + offset)));
      this->controller_digital_input |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->controller_digital_input);
      this->controller_digital_output =  ((uint16_t) (*(inbuffer + offset)));
      this->controller_digital_output |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->controller_digital_output);
      for( uint32_t i = 0; i < 2; i++){
      this->controller_analog_input_type[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_analog_input_type[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->controller_analog_input[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      this->controller_analog_output_type[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_analog_output_type[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->controller_analog_output[i]));
      }
      this->flange_digital_input =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flange_digital_input);
      this->flange_digital_output =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flange_digital_output);
      for( uint32_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->flange_analog_input[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      this->external_encoder_strobe_count[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->external_encoder_strobe_count[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      this->external_encoder_count[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->external_encoder_count[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->external_encoder_count[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_joint_position[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_tcp_position[i]));
      }
      this->robot_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robot_mode);
      this->robot_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robot_state);
      this->control_mode =  ((uint16_t) (*(inbuffer + offset)));
      this->control_mode |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->control_mode);
      for( uint32_t i = 0; i < 256; i++){
      this->reserved[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/RobotStateRT"; };
    virtual const char * getMD5() override { return "3fc4bdbba50c12ddf9126a58ad825fc4"; };

  };

}
#endif
