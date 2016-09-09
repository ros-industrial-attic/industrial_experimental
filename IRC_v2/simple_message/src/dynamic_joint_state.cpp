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
#include "simple_message/dynamic_joint_state.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "dynamic_joint_state.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using industrial::byte_array::ByteArray;

namespace industrial
{
namespace dynamic_joint_state
{

DynamicJointStateGrp::DynamicJointStateGrp(void)
{
  this->init(0);
}
DynamicJointStateGrp::~DynamicJointStateGrp(void)
{

}

bool DynamicJointStateGrp::init(shared_int num_joints, shared_int group_id)
{
  if (num_joints < 0)
    return false;

  this->group_id_ = group_id;
  this->num_joints_ = num_joints;
  this->valid_fields_.reset();
  this->state_.clear();

  return true;
}

bool DynamicJointStateGrp::operator==(const DynamicJointStateGrp &rhs) const
{
  return this->num_joints_   == rhs.num_joints_ &&
         this->valid_fields_ == rhs.valid_fields_ &&
         this->state_ == rhs.state_;
}

bool DynamicJointStateGrp::setDynData(ValidFieldType validFieldCode,
                                      const std::vector<shared_real> &newVals)
{
  if (this->num_joints_ != newVals.size())
    return false;

  this->state_[validFieldCode] = newVals;
  valid_fields_.set(validFieldCode, true);

  return true;
}

bool DynamicJointStateGrp::getDynData(ValidFieldType validFieldCode,
                                      std::vector<shared_real> &result) const
{
  bool found = isValid(validFieldCode);

  if (found)
    result = this->state_.at(validFieldCode);

  return found;
}

void DynamicJointStateGrp::clearDynData(ValidFieldType validFieldCode)
{
  this->state_.erase(validFieldCode);
  valid_fields_.set(validFieldCode, false);
}

bool DynamicJointStateGrp::loadArray(const std::vector<shared_real> &data, 
                                     ByteArray *buffer)
{
  std::vector<shared_real>::const_iterator it;
  for (it=data.begin(); it!=data.end(); ++it)
  {
    if (!buffer->load(*it))
      return false;
  }
  return true;
}

bool DynamicJointStateGrp::unloadArray(std::vector<shared_real> &data,
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

bool DynamicJointStateGrp::load(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Joint State Group load");

  if (!buffer->load(this->group_id_))
  {
    LOG_ERROR("Failed to load Dynamic Joint State Group group_id");
    return false;
  }

  if (!buffer->load(this->num_joints_))
  {
    LOG_ERROR("Failed to load Dynamic Joint State Group num_joints");
    return false;
  }

  shared_int valid_fields = this->valid_fields_.to_ulong();
  if (!buffer->load(valid_fields))
  {
    LOG_ERROR("Failed to load Dynamic Joint State Group valid_fields");
    return false;
  }

  // load all dynamic data arrays, in correct order
  for (int i=0; i<FieldTypes::end; ++i)
  {
    ValidFieldType field = static_cast<ValidFieldType>(i);

    if (isValid(field) && !loadArray(this->state_[field], buffer))
    {
      LOG_ERROR("Failed to load Dynamic Joint State Group type %d", field);
      return false;
    }
  }

  LOG_COMM("Dynamic Joint State Group successfully loaded");
  return true;
}

bool DynamicJointStateGrp::unloadFront(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Joint State Group unloadFront");

  if (!buffer->unloadFront(this->group_id_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint State Group group_id");
    return false;
  }

  if (!buffer->unloadFront(this->num_joints_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint State Group num_joints");
    return false;
  }

  shared_int valid_fields;
  if (!buffer->unloadFront(valid_fields))
  {
    LOG_ERROR("Failed to unload Dynamic Joint State Group valid_fields");
    return false;
  }
  this->valid_fields_ = valid_fields;

  // unload all dynamic data arrays, in correct order
  for (int i=0; i<FieldTypes::end; ++i)
  {
    ValidFieldType field = static_cast<ValidFieldType>(i);

    if (isValid(field)
     && !unloadArray(this->state_[field], this->num_joints_, buffer))
    {
      LOG_ERROR("Failed to unload Dynamic Joint State Group type %d", field);
      return false;
    }
  }

  LOG_COMM("Dynamic Joint State Group successfully unloaded (from FRONT)");
  return true;
}

bool DynamicJointStateGrp::unload(ByteArray *buffer)
{
  LOG_ERROR("DynamicJointStateGrp.unload() unsupported.  Use unloadFront() instead.");
  return false;
}


DynamicJointState::DynamicJointState(void)
{
  this->init();
}
DynamicJointState::~DynamicJointState(void)
{

}

void DynamicJointState::init(void)
{
  this->sequence_ = 0;
  this->groups_.clear();
}

bool DynamicJointState::operator==(const DynamicJointState &rhs) const
{
  bool result = this->sequence_ == rhs.sequence_ &&
                this->groups_   == rhs.groups_;

  return result;
}

bool DynamicJointState::load(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Joint State load");

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load Dynamic Joint State sequence");
    return false;
  }

  if (!buffer->load(this->getNumGroups()))
  {
    LOG_ERROR("Failed to load Dynamic Joint State num_groups");
    return false;
  }

  GroupList::iterator grpIt;
  for (grpIt=this->groups_.begin(); grpIt!=this->groups_.end(); ++grpIt)
  {
    if (!grpIt->load(buffer))
    {
      LOG_ERROR("Failed to load Dynamic Joint State group %d", grpIt->getGroupID());
      return false;
    }
  }

  LOG_COMM("Dynamic Joint State successfully loaded");
  return true;
}

bool DynamicJointState::unloadFront(ByteArray *buffer)
{
  int num_groups;

  LOG_COMM("Executing Dynamic Joint State unloadFront");

  if (!buffer->unloadFront(this->sequence_))
  {
    LOG_ERROR("Failed to unload Dynamic Joint State sequence");
    return false;
  }

  if (!buffer->unloadFront(num_groups))
  {
    LOG_ERROR("Failed to unload Dynamic Joint State num_groups");
    return false;
  }

  for (int i=0; i<num_groups; ++i)
  {
    DynamicJointStateGrp group;

    if (!group.unloadFront(buffer))
    {
      LOG_ERROR("Failed to parse Dynamic Joint State Group (index %d)", i);
      return false;
    }

    if (!this->addGroup(group))
    {
      LOG_ERROR("Duplicate data received for group id %d", group.getGroupID());
      return false;
    }
  }

  LOG_COMM("Dynamic Joint State successfully unloaded (from FRONT)");
  return true;
}

bool DynamicJointState::unload(ByteArray *buffer)
{
  LOG_ERROR("DynamicJointState.unload() unsupported.  Use unloadFront() instead.");
  return false;
}

}
}

