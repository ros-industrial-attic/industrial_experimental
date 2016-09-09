/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FLATHEADERS
#include "simple_message/dynamic_joint_pt.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "dynamic_joint_pt.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using industrial::byte_array::ByteArray;

namespace industrial
{
namespace dynamic_joint_pt
{

DynamicJointPtGrp::DynamicJointPtGrp(void)
{
  this->init(0);
}
DynamicJointPtGrp::~DynamicJointPtGrp(void)
{

}

bool DynamicJointPtGrp::init(shared_int num_joints)
{
  if (num_joints < 0)
    return false;

  this->group_id_ = 0;
  this->num_joints_ = num_joints;
  this->valid_fields_.reset();
  this->pos_.clear();
  this->vel_.clear();
  this->accel_.clear();
  this->effort_.clear();
  this->time_ = 0.0;

  return true;
}

bool DynamicJointPtGrp::operator==(const DynamicJointPtGrp &rhs) const
{
  return this->group_id_ == rhs.group_id_ &&
         this->num_joints_   == rhs.num_joints_ &&
         this->valid_fields_ == rhs.valid_fields_ &&
         ( !isValid(FieldTypes::POSITION) || (this->pos_  == rhs.pos_) ) &&
         ( !isValid(FieldTypes::VELOCITY) || (this->vel_  == rhs.vel_) ) &&
         ( !isValid(FieldTypes::ACCELERATION) || (this->accel_ == rhs.accel_) ) &&
         ( !isValid(FieldTypes::EFFORT)   || (this->effort_ == rhs.effort_) ) &&
         ( !isValid(FieldTypes::TIME)     || (this->time_ == rhs.time_) );
}

bool DynamicJointPtGrp::loadArray(const std::vector<shared_real> &data, 
                                  ByteArray *buffer)
{
  for (size_t i=0; i<data.size(); ++i)
  {
    if (!buffer->load(data[i]))
      return false;
  }
  return true;
}

bool DynamicJointPtGrp::unloadArray(std::vector<shared_real> &data,
                                    unsigned int len,
                                    ByteArray *buffer)
{
  data.resize(len);
  
  for (size_t i=0; i<len; ++i)
  {
    if (!buffer->unloadFront(data[i]))
      return false;
  }
  return true;
}

bool DynamicJointPtGrp::load(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Joint Point Group load");

  if (!buffer->load(this->group_id_))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group group_id");
    return false;
  }
  
  if (!buffer->load(this->num_joints_))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group num_joints");
    return false;
  }

  shared_int valid_fields = this->valid_fields_.to_ulong();
  if (!buffer->load(valid_fields))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group valid_fields");
    return false;
  }

  if (isValid(FieldTypes::POSITION) && !loadArray(this->pos_, buffer))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group positions");
    return false;
  }

  if (isValid(FieldTypes::VELOCITY) && !loadArray(this->vel_, buffer))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group velocities");
    return false;
  }

  if (isValid(FieldTypes::ACCELERATION) && !loadArray(this->accel_, buffer))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group accelerations");
    return false;
  }

  if (isValid(FieldTypes::EFFORT) && !loadArray(this->effort_, buffer))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group efforts");
    return false;
  }

  if (isValid(FieldTypes::TIME) && !buffer->load(this->time_))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point Group time");
    return false;
  }

  LOG_COMM("Dynamic Joint Point Group successfully loaded");
  return true;
}

bool DynamicJointPtGrp::unloadFront(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Joint Point Group unloadFront");

  if (!buffer->unloadFront(this->group_id_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group group_id");
    return false;
  }

  if (!buffer->unloadFront(this->num_joints_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group num_joints");
    return false;
  }

  shared_int valid_fields;
  if (!buffer->unloadFront(valid_fields))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group valid_fields");
    return false;
  }
  this->valid_fields_ = valid_fields;

  if ( isValid(FieldTypes::POSITION)
    && !unloadArray(this->pos_, this->num_joints_, buffer))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group positions");
    return false;
  }

  if (isValid(FieldTypes::VELOCITY)
   && !unloadArray(this->vel_, this->num_joints_, buffer))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group velocities");
    return false;
  }

  if ( isValid(FieldTypes::ACCELERATION)
    && !unloadArray(this->accel_, this->num_joints_, buffer))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group accelerations");
    return false;
  }

  if ( isValid(FieldTypes::EFFORT)
    && !unloadArray(this->effort_, this->num_joints_, buffer))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group efforts");
    return false;
  }

  if ( isValid(FieldTypes::TIME)
    && !buffer->unloadFront(this->time_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group time");
    return false;
  }

  LOG_COMM("Dynamic Joint Point Group successfully unloaded (from FRONT)");
  return true;
}

bool DynamicJointPtGrp::unload(ByteArray *buffer)
{
  LOG_ERROR("DynamicJointPtGrp.unload() unsupported.  Use unloadFront() instead.");
  return false;
}


DynamicJointPt::DynamicJointPt(void)
{
  this->init();
}
DynamicJointPt::~DynamicJointPt(void)
{

}

void DynamicJointPt::init(void)
{
  this->sequence_ = 0;
  this->groups_.clear();
}

bool DynamicJointPt::operator==(const DynamicJointPt &rhs) const
{
  bool result = this->sequence_       == rhs.sequence_ &&
                this->getNumGroups()  == rhs.getNumGroups();

  // this simplistic comparison requires groups to be in the same order
  // TODO: improve comparison to not require matching order
  for (size_t i=0; i<this->groups_.size(); ++i)
    result &= this->groups_[i] == rhs.getGroup(i);

  return result;
}

bool DynamicJointPt::load(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Joint Point load");

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point sequence");
    return false;
  }

  if (!buffer->load(this->getNumGroups()))
  {
    LOG_ERROR("Failed to load Dynamic Joint Point num_groups");
    return false;
  }

  for (size_t i=0; i<this->groups_.size(); ++i)
  {
    if (!this->groups_[i].load(buffer))
    {
      LOG_ERROR("Failed to load Dynamic Joint Point group %d",
                this->groups_[i].getGroupID());
      return false;
    }
  }

  LOG_COMM("Dynamic Joint Point successfully loaded");
  return true;
}

bool DynamicJointPt::unloadFront(ByteArray *buffer)
{
  int num_groups;

  LOG_COMM("Executing Dynamic Joint Point unloadFront");

  if (!buffer->unloadFront(this->sequence_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point sequence");
    return false;
  }

  if (!buffer->unloadFront(num_groups))
  {
    LOG_ERROR("Failed to unload Dynamic Joint Point Group num_groups");
    return false;
  }

  for (int i=0; i<num_groups; ++i)
  {
    DynamicJointPtGrp group;

    if (!group.unloadFront(buffer))
    {
      LOG_ERROR("Failed to unload Dynamic Joint Point group (index %d)", i);
      return false;
    }
    int id = group.getGroupID();

    if (this->hasGroupID(id))
    {
      LOG_ERROR("Duplicate data received for group id %d", id);
      return false;
    }

    this->addGroup(group);
  }

  LOG_COMM("Dynamic Joint Point successfully unloaded (from FRONT)");
  return true;
}

bool DynamicJointPt::unload(ByteArray *buffer)
{
  LOG_ERROR("DynamicJointPt.unload() unsupported.  Use unloadFront() instead.");
  return false;
}

}
}

