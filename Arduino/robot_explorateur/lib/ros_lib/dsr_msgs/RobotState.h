#ifndef _ROS_dsr_msgs_RobotState_h
#define _ROS_dsr_msgs_RobotState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"
#include "dsr_msgs/ModbusState.h"

namespace dsr_msgs
{

  class RobotState : public ros::Msg
  {
    public:
      typedef int32_t _robot_state_type;
      _robot_state_type robot_state;
      typedef const char* _robot_state_str_type;
      _robot_state_str_type robot_state_str;
      typedef int8_t _actual_mode_type;
      _actual_mode_type actual_mode;
      typedef int8_t _actual_space_type;
      _actual_space_type actual_space;
      float current_posj[6];
      float current_velj[6];
      float joint_abs[6];
      float joint_err[6];
      float target_posj[6];
      float target_velj[6];
      float current_posx[6];
      float current_tool_posx[6];
      float current_velx[6];
      float task_err[6];
      float target_velx[6];
      float target_posx[6];
      float dynamic_tor[6];
      float actual_jts[6];
      float actual_ejt[6];
      float actual_ett[6];
      int8_t actual_bk[6];
      float actual_mc[6];
      float actual_mt[6];
      typedef int8_t _solution_space_type;
      _solution_space_type solution_space;
      typedef float _sync_time_type;
      _sync_time_type sync_time;
      int8_t actual_bt[5];
      uint32_t rotation_matrix_length;
      typedef std_msgs::Float64MultiArray _rotation_matrix_type;
      _rotation_matrix_type st_rotation_matrix;
      _rotation_matrix_type * rotation_matrix;
      int8_t ctrlbox_digital_input[16];
      int8_t ctrlbox_digital_output[16];
      int8_t flange_digital_input[6];
      int8_t flange_digital_output[6];
      uint32_t modbus_state_length;
      typedef dsr_msgs::ModbusState _modbus_state_type;
      _modbus_state_type st_modbus_state;
      _modbus_state_type * modbus_state;
      typedef int32_t _access_control_type;
      _access_control_type access_control;
      typedef bool _homming_completed_type;
      _homming_completed_type homming_completed;
      typedef bool _tp_initialized_type;
      _tp_initialized_type tp_initialized;
      typedef int8_t _mastering_need_type;
      _mastering_need_type mastering_need;
      typedef bool _drl_stopped_type;
      _drl_stopped_type drl_stopped;
      typedef bool _disconnected_type;
      _disconnected_type disconnected;
      float fActualW2B[6];
      uint32_t fCurrentPosW_length;
      typedef std_msgs::Float64MultiArray _fCurrentPosW_type;
      _fCurrentPosW_type st_fCurrentPosW;
      _fCurrentPosW_type * fCurrentPosW;
      float fCurrentVelW[6];
      float fWorldETT[6];
      float fTargetPosW[6];
      float fTargetVelW[6];
      uint32_t fRotationMatrixWorld_length;
      typedef std_msgs::Float64MultiArray _fRotationMatrixWorld_type;
      _fRotationMatrixWorld_type st_fRotationMatrixWorld;
      _fRotationMatrixWorld_type * fRotationMatrixWorld;
      typedef int8_t _iActualUCN_type;
      _iActualUCN_type iActualUCN;
      typedef int8_t _iParent_type;
      _iParent_type iParent;
      uint32_t fCurrentPosU_length;
      typedef std_msgs::Float64MultiArray _fCurrentPosU_type;
      _fCurrentPosU_type st_fCurrentPosU;
      _fCurrentPosU_type * fCurrentPosU;
      float fCurrentVelU[6];
      float fUserETT[6];
      float fTargetPosU[6];
      float fTargetVelU[6];
      uint32_t fRotationMatrixUser_length;
      typedef std_msgs::Float64MultiArray _fRotationMatrixUser_type;
      _fRotationMatrixUser_type st_fRotationMatrixUser;
      _fRotationMatrixUser_type * fRotationMatrixUser;
      float fActualAI[6];
      bool bActualSW[3];
      bool bActualSI[2];
      int8_t iActualAT[2];
      float fTargetAO[2];
      int8_t iTargetAT[2];
      bool bActualES[2];
      int8_t iActualED[2];
      bool bActualER[2];

    RobotState():
      robot_state(0),
      robot_state_str(""),
      actual_mode(0),
      actual_space(0),
      current_posj(),
      current_velj(),
      joint_abs(),
      joint_err(),
      target_posj(),
      target_velj(),
      current_posx(),
      current_tool_posx(),
      current_velx(),
      task_err(),
      target_velx(),
      target_posx(),
      dynamic_tor(),
      actual_jts(),
      actual_ejt(),
      actual_ett(),
      actual_bk(),
      actual_mc(),
      actual_mt(),
      solution_space(0),
      sync_time(0),
      actual_bt(),
      rotation_matrix_length(0), st_rotation_matrix(), rotation_matrix(nullptr),
      ctrlbox_digital_input(),
      ctrlbox_digital_output(),
      flange_digital_input(),
      flange_digital_output(),
      modbus_state_length(0), st_modbus_state(), modbus_state(nullptr),
      access_control(0),
      homming_completed(0),
      tp_initialized(0),
      mastering_need(0),
      drl_stopped(0),
      disconnected(0),
      fActualW2B(),
      fCurrentPosW_length(0), st_fCurrentPosW(), fCurrentPosW(nullptr),
      fCurrentVelW(),
      fWorldETT(),
      fTargetPosW(),
      fTargetVelW(),
      fRotationMatrixWorld_length(0), st_fRotationMatrixWorld(), fRotationMatrixWorld(nullptr),
      iActualUCN(0),
      iParent(0),
      fCurrentPosU_length(0), st_fCurrentPosU(), fCurrentPosU(nullptr),
      fCurrentVelU(),
      fUserETT(),
      fTargetPosU(),
      fTargetVelU(),
      fRotationMatrixUser_length(0), st_fRotationMatrixUser(), fRotationMatrixUser(nullptr),
      fActualAI(),
      bActualSW(),
      bActualSI(),
      iActualAT(),
      fTargetAO(),
      iTargetAT(),
      bActualES(),
      iActualED(),
      bActualER()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_robot_state;
      u_robot_state.real = this->robot_state;
      *(outbuffer + offset + 0) = (u_robot_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_state);
      uint32_t length_robot_state_str = strlen(this->robot_state_str);
      varToArr(outbuffer + offset, length_robot_state_str);
      offset += 4;
      memcpy(outbuffer + offset, this->robot_state_str, length_robot_state_str);
      offset += length_robot_state_str;
      union {
        int8_t real;
        uint8_t base;
      } u_actual_mode;
      u_actual_mode.real = this->actual_mode;
      *(outbuffer + offset + 0) = (u_actual_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actual_mode);
      union {
        int8_t real;
        uint8_t base;
      } u_actual_space;
      u_actual_space.real = this->actual_space;
      *(outbuffer + offset + 0) = (u_actual_space.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actual_space);
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->current_posj[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->current_velj[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_abs[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_err[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_posj[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_velj[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->current_posx[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->current_tool_posx[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->current_velx[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->task_err[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_velx[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->target_posx[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dynamic_tor[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_jts[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_ejt[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_ett[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_actual_bki;
      u_actual_bki.real = this->actual_bk[i];
      *(outbuffer + offset + 0) = (u_actual_bki.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actual_bk[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_mc[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->actual_mt[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_solution_space;
      u_solution_space.real = this->solution_space;
      *(outbuffer + offset + 0) = (u_solution_space.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solution_space);
      offset += serializeAvrFloat64(outbuffer + offset, this->sync_time);
      for( uint32_t i = 0; i < 5; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_actual_bti;
      u_actual_bti.real = this->actual_bt[i];
      *(outbuffer + offset + 0) = (u_actual_bti.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actual_bt[i]);
      }
      *(outbuffer + offset + 0) = (this->rotation_matrix_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rotation_matrix_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rotation_matrix_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rotation_matrix_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotation_matrix_length);
      for( uint32_t i = 0; i < rotation_matrix_length; i++){
      offset += this->rotation_matrix[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 16; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_ctrlbox_digital_inputi;
      u_ctrlbox_digital_inputi.real = this->ctrlbox_digital_input[i];
      *(outbuffer + offset + 0) = (u_ctrlbox_digital_inputi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ctrlbox_digital_input[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_ctrlbox_digital_outputi;
      u_ctrlbox_digital_outputi.real = this->ctrlbox_digital_output[i];
      *(outbuffer + offset + 0) = (u_ctrlbox_digital_outputi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ctrlbox_digital_output[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_flange_digital_inputi;
      u_flange_digital_inputi.real = this->flange_digital_input[i];
      *(outbuffer + offset + 0) = (u_flange_digital_inputi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flange_digital_input[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_flange_digital_outputi;
      u_flange_digital_outputi.real = this->flange_digital_output[i];
      *(outbuffer + offset + 0) = (u_flange_digital_outputi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flange_digital_output[i]);
      }
      *(outbuffer + offset + 0) = (this->modbus_state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->modbus_state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->modbus_state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->modbus_state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->modbus_state_length);
      for( uint32_t i = 0; i < modbus_state_length; i++){
      offset += this->modbus_state[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_access_control;
      u_access_control.real = this->access_control;
      *(outbuffer + offset + 0) = (u_access_control.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_access_control.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_access_control.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_access_control.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->access_control);
      union {
        bool real;
        uint8_t base;
      } u_homming_completed;
      u_homming_completed.real = this->homming_completed;
      *(outbuffer + offset + 0) = (u_homming_completed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->homming_completed);
      union {
        bool real;
        uint8_t base;
      } u_tp_initialized;
      u_tp_initialized.real = this->tp_initialized;
      *(outbuffer + offset + 0) = (u_tp_initialized.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tp_initialized);
      union {
        int8_t real;
        uint8_t base;
      } u_mastering_need;
      u_mastering_need.real = this->mastering_need;
      *(outbuffer + offset + 0) = (u_mastering_need.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mastering_need);
      union {
        bool real;
        uint8_t base;
      } u_drl_stopped;
      u_drl_stopped.real = this->drl_stopped;
      *(outbuffer + offset + 0) = (u_drl_stopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->drl_stopped);
      union {
        bool real;
        uint8_t base;
      } u_disconnected;
      u_disconnected.real = this->disconnected;
      *(outbuffer + offset + 0) = (u_disconnected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->disconnected);
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fActualW2B[i]);
      }
      *(outbuffer + offset + 0) = (this->fCurrentPosW_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fCurrentPosW_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fCurrentPosW_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fCurrentPosW_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fCurrentPosW_length);
      for( uint32_t i = 0; i < fCurrentPosW_length; i++){
      offset += this->fCurrentPosW[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fCurrentVelW[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fWorldETT[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fTargetPosW[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fTargetVelW[i]);
      }
      *(outbuffer + offset + 0) = (this->fRotationMatrixWorld_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fRotationMatrixWorld_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fRotationMatrixWorld_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fRotationMatrixWorld_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fRotationMatrixWorld_length);
      for( uint32_t i = 0; i < fRotationMatrixWorld_length; i++){
      offset += this->fRotationMatrixWorld[i].serialize(outbuffer + offset);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_iActualUCN;
      u_iActualUCN.real = this->iActualUCN;
      *(outbuffer + offset + 0) = (u_iActualUCN.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->iActualUCN);
      union {
        int8_t real;
        uint8_t base;
      } u_iParent;
      u_iParent.real = this->iParent;
      *(outbuffer + offset + 0) = (u_iParent.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->iParent);
      *(outbuffer + offset + 0) = (this->fCurrentPosU_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fCurrentPosU_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fCurrentPosU_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fCurrentPosU_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fCurrentPosU_length);
      for( uint32_t i = 0; i < fCurrentPosU_length; i++){
      offset += this->fCurrentPosU[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fCurrentVelU[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fUserETT[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fTargetPosU[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fTargetVelU[i]);
      }
      *(outbuffer + offset + 0) = (this->fRotationMatrixUser_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fRotationMatrixUser_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fRotationMatrixUser_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fRotationMatrixUser_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fRotationMatrixUser_length);
      for( uint32_t i = 0; i < fRotationMatrixUser_length; i++){
      offset += this->fRotationMatrixUser[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fActualAI[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualSWi;
      u_bActualSWi.real = this->bActualSW[i];
      *(outbuffer + offset + 0) = (u_bActualSWi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bActualSW[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualSIi;
      u_bActualSIi.real = this->bActualSI[i];
      *(outbuffer + offset + 0) = (u_bActualSIi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bActualSI[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_iActualATi;
      u_iActualATi.real = this->iActualAT[i];
      *(outbuffer + offset + 0) = (u_iActualATi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->iActualAT[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fTargetAO[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_iTargetATi;
      u_iTargetATi.real = this->iTargetAT[i];
      *(outbuffer + offset + 0) = (u_iTargetATi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->iTargetAT[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualESi;
      u_bActualESi.real = this->bActualES[i];
      *(outbuffer + offset + 0) = (u_bActualESi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bActualES[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_iActualEDi;
      u_iActualEDi.real = this->iActualED[i];
      *(outbuffer + offset + 0) = (u_iActualEDi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->iActualED[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualERi;
      u_bActualERi.real = this->bActualER[i];
      *(outbuffer + offset + 0) = (u_bActualERi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bActualER[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_robot_state;
      u_robot_state.base = 0;
      u_robot_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->robot_state = u_robot_state.real;
      offset += sizeof(this->robot_state);
      uint32_t length_robot_state_str;
      arrToVar(length_robot_state_str, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_robot_state_str; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_robot_state_str-1]=0;
      this->robot_state_str = (char *)(inbuffer + offset-1);
      offset += length_robot_state_str;
      union {
        int8_t real;
        uint8_t base;
      } u_actual_mode;
      u_actual_mode.base = 0;
      u_actual_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actual_mode = u_actual_mode.real;
      offset += sizeof(this->actual_mode);
      union {
        int8_t real;
        uint8_t base;
      } u_actual_space;
      u_actual_space.base = 0;
      u_actual_space.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actual_space = u_actual_space.real;
      offset += sizeof(this->actual_space);
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_posj[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_velj[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_abs[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_err[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_posj[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_velj[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_posx[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_tool_posx[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_velx[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->task_err[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_velx[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_posx[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dynamic_tor[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_jts[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_ejt[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_ett[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_actual_bki;
      u_actual_bki.base = 0;
      u_actual_bki.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actual_bk[i] = u_actual_bki.real;
      offset += sizeof(this->actual_bk[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_mc[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->actual_mt[i]));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_solution_space;
      u_solution_space.base = 0;
      u_solution_space.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solution_space = u_solution_space.real;
      offset += sizeof(this->solution_space);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sync_time));
      for( uint32_t i = 0; i < 5; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_actual_bti;
      u_actual_bti.base = 0;
      u_actual_bti.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actual_bt[i] = u_actual_bti.real;
      offset += sizeof(this->actual_bt[i]);
      }
      uint32_t rotation_matrix_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rotation_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rotation_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rotation_matrix_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rotation_matrix_length);
      if(rotation_matrix_lengthT > rotation_matrix_length)
        this->rotation_matrix = (std_msgs::Float64MultiArray*)realloc(this->rotation_matrix, rotation_matrix_lengthT * sizeof(std_msgs::Float64MultiArray));
      rotation_matrix_length = rotation_matrix_lengthT;
      for( uint32_t i = 0; i < rotation_matrix_length; i++){
      offset += this->st_rotation_matrix.deserialize(inbuffer + offset);
        memcpy( &(this->rotation_matrix[i]), &(this->st_rotation_matrix), sizeof(std_msgs::Float64MultiArray));
      }
      for( uint32_t i = 0; i < 16; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_ctrlbox_digital_inputi;
      u_ctrlbox_digital_inputi.base = 0;
      u_ctrlbox_digital_inputi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ctrlbox_digital_input[i] = u_ctrlbox_digital_inputi.real;
      offset += sizeof(this->ctrlbox_digital_input[i]);
      }
      for( uint32_t i = 0; i < 16; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_ctrlbox_digital_outputi;
      u_ctrlbox_digital_outputi.base = 0;
      u_ctrlbox_digital_outputi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ctrlbox_digital_output[i] = u_ctrlbox_digital_outputi.real;
      offset += sizeof(this->ctrlbox_digital_output[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_flange_digital_inputi;
      u_flange_digital_inputi.base = 0;
      u_flange_digital_inputi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->flange_digital_input[i] = u_flange_digital_inputi.real;
      offset += sizeof(this->flange_digital_input[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_flange_digital_outputi;
      u_flange_digital_outputi.base = 0;
      u_flange_digital_outputi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->flange_digital_output[i] = u_flange_digital_outputi.real;
      offset += sizeof(this->flange_digital_output[i]);
      }
      uint32_t modbus_state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      modbus_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      modbus_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      modbus_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->modbus_state_length);
      if(modbus_state_lengthT > modbus_state_length)
        this->modbus_state = (dsr_msgs::ModbusState*)realloc(this->modbus_state, modbus_state_lengthT * sizeof(dsr_msgs::ModbusState));
      modbus_state_length = modbus_state_lengthT;
      for( uint32_t i = 0; i < modbus_state_length; i++){
      offset += this->st_modbus_state.deserialize(inbuffer + offset);
        memcpy( &(this->modbus_state[i]), &(this->st_modbus_state), sizeof(dsr_msgs::ModbusState));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_access_control;
      u_access_control.base = 0;
      u_access_control.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_access_control.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_access_control.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_access_control.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->access_control = u_access_control.real;
      offset += sizeof(this->access_control);
      union {
        bool real;
        uint8_t base;
      } u_homming_completed;
      u_homming_completed.base = 0;
      u_homming_completed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->homming_completed = u_homming_completed.real;
      offset += sizeof(this->homming_completed);
      union {
        bool real;
        uint8_t base;
      } u_tp_initialized;
      u_tp_initialized.base = 0;
      u_tp_initialized.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tp_initialized = u_tp_initialized.real;
      offset += sizeof(this->tp_initialized);
      union {
        int8_t real;
        uint8_t base;
      } u_mastering_need;
      u_mastering_need.base = 0;
      u_mastering_need.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mastering_need = u_mastering_need.real;
      offset += sizeof(this->mastering_need);
      union {
        bool real;
        uint8_t base;
      } u_drl_stopped;
      u_drl_stopped.base = 0;
      u_drl_stopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->drl_stopped = u_drl_stopped.real;
      offset += sizeof(this->drl_stopped);
      union {
        bool real;
        uint8_t base;
      } u_disconnected;
      u_disconnected.base = 0;
      u_disconnected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->disconnected = u_disconnected.real;
      offset += sizeof(this->disconnected);
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fActualW2B[i]));
      }
      uint32_t fCurrentPosW_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fCurrentPosW_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fCurrentPosW_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fCurrentPosW_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fCurrentPosW_length);
      if(fCurrentPosW_lengthT > fCurrentPosW_length)
        this->fCurrentPosW = (std_msgs::Float64MultiArray*)realloc(this->fCurrentPosW, fCurrentPosW_lengthT * sizeof(std_msgs::Float64MultiArray));
      fCurrentPosW_length = fCurrentPosW_lengthT;
      for( uint32_t i = 0; i < fCurrentPosW_length; i++){
      offset += this->st_fCurrentPosW.deserialize(inbuffer + offset);
        memcpy( &(this->fCurrentPosW[i]), &(this->st_fCurrentPosW), sizeof(std_msgs::Float64MultiArray));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fCurrentVelW[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fWorldETT[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fTargetPosW[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fTargetVelW[i]));
      }
      uint32_t fRotationMatrixWorld_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fRotationMatrixWorld_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fRotationMatrixWorld_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fRotationMatrixWorld_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fRotationMatrixWorld_length);
      if(fRotationMatrixWorld_lengthT > fRotationMatrixWorld_length)
        this->fRotationMatrixWorld = (std_msgs::Float64MultiArray*)realloc(this->fRotationMatrixWorld, fRotationMatrixWorld_lengthT * sizeof(std_msgs::Float64MultiArray));
      fRotationMatrixWorld_length = fRotationMatrixWorld_lengthT;
      for( uint32_t i = 0; i < fRotationMatrixWorld_length; i++){
      offset += this->st_fRotationMatrixWorld.deserialize(inbuffer + offset);
        memcpy( &(this->fRotationMatrixWorld[i]), &(this->st_fRotationMatrixWorld), sizeof(std_msgs::Float64MultiArray));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_iActualUCN;
      u_iActualUCN.base = 0;
      u_iActualUCN.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->iActualUCN = u_iActualUCN.real;
      offset += sizeof(this->iActualUCN);
      union {
        int8_t real;
        uint8_t base;
      } u_iParent;
      u_iParent.base = 0;
      u_iParent.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->iParent = u_iParent.real;
      offset += sizeof(this->iParent);
      uint32_t fCurrentPosU_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fCurrentPosU_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fCurrentPosU_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fCurrentPosU_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fCurrentPosU_length);
      if(fCurrentPosU_lengthT > fCurrentPosU_length)
        this->fCurrentPosU = (std_msgs::Float64MultiArray*)realloc(this->fCurrentPosU, fCurrentPosU_lengthT * sizeof(std_msgs::Float64MultiArray));
      fCurrentPosU_length = fCurrentPosU_lengthT;
      for( uint32_t i = 0; i < fCurrentPosU_length; i++){
      offset += this->st_fCurrentPosU.deserialize(inbuffer + offset);
        memcpy( &(this->fCurrentPosU[i]), &(this->st_fCurrentPosU), sizeof(std_msgs::Float64MultiArray));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fCurrentVelU[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fUserETT[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fTargetPosU[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fTargetVelU[i]));
      }
      uint32_t fRotationMatrixUser_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fRotationMatrixUser_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fRotationMatrixUser_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fRotationMatrixUser_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fRotationMatrixUser_length);
      if(fRotationMatrixUser_lengthT > fRotationMatrixUser_length)
        this->fRotationMatrixUser = (std_msgs::Float64MultiArray*)realloc(this->fRotationMatrixUser, fRotationMatrixUser_lengthT * sizeof(std_msgs::Float64MultiArray));
      fRotationMatrixUser_length = fRotationMatrixUser_lengthT;
      for( uint32_t i = 0; i < fRotationMatrixUser_length; i++){
      offset += this->st_fRotationMatrixUser.deserialize(inbuffer + offset);
        memcpy( &(this->fRotationMatrixUser[i]), &(this->st_fRotationMatrixUser), sizeof(std_msgs::Float64MultiArray));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fActualAI[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualSWi;
      u_bActualSWi.base = 0;
      u_bActualSWi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bActualSW[i] = u_bActualSWi.real;
      offset += sizeof(this->bActualSW[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualSIi;
      u_bActualSIi.base = 0;
      u_bActualSIi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bActualSI[i] = u_bActualSIi.real;
      offset += sizeof(this->bActualSI[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_iActualATi;
      u_iActualATi.base = 0;
      u_iActualATi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->iActualAT[i] = u_iActualATi.real;
      offset += sizeof(this->iActualAT[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fTargetAO[i]));
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_iTargetATi;
      u_iTargetATi.base = 0;
      u_iTargetATi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->iTargetAT[i] = u_iTargetATi.real;
      offset += sizeof(this->iTargetAT[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualESi;
      u_bActualESi.base = 0;
      u_bActualESi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bActualES[i] = u_bActualESi.real;
      offset += sizeof(this->bActualES[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_iActualEDi;
      u_iActualEDi.base = 0;
      u_iActualEDi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->iActualED[i] = u_iActualEDi.real;
      offset += sizeof(this->iActualED[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        bool real;
        uint8_t base;
      } u_bActualERi;
      u_bActualERi.base = 0;
      u_bActualERi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bActualER[i] = u_bActualERi.real;
      offset += sizeof(this->bActualER[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "dsr_msgs/RobotState"; };
    virtual const char * getMD5() override { return "0473bf35fc3b2d88cf36052d2ba4677e"; };

  };

}
#endif
