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
#include "simple_message/dynamic_group_status.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "dynamic_group_state.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using industrial::byte_array::ByteArray;
using industrial::robot_status::RobotStatus;

namespace industrial
{
namespace dynamic_group_status
{

bool DynamicGroupStatusGrp::operator==(const DynamicGroupStatusGrp &rhs) const
{
  return ((this->group_id_ == rhs.group_id_)
           && RobotStatus::operator==(rhs));
}

bool DynamicGroupStatusGrp::load(industrial::byte_array::ByteArray *buffer)
{
    if (!buffer->load(this->group_id_))
    {
      LOG_ERROR("Failed to load Dynamic Group Status group ID");
      return false;
    }

    if (!RobotStatus::load(buffer))
    {
      LOG_ERROR("Failed to load Dynamic Group Status status data");
      return false;
    }

    return true;
}

bool DynamicGroupStatusGrp::unloadFront(industrial::byte_array::ByteArray *buffer)
{
    if (!buffer->unloadFront(this->group_id_))
    {
      LOG_ERROR("Failed to unload Dynamic Group Status group ID");
      return false;
    }

    if (!RobotStatus::unloadFront(buffer))
    {
      LOG_ERROR("Failed to unload Dynamic Group Status status data");
      return false;
    }

    return true;
}

bool DynamicGroupStatusGrp::unload(ByteArray *buffer)
{
    if (!RobotStatus::unload(buffer))
    {
      LOG_ERROR("Failed to unload Dynamic Group Status status data");
      return false;
    }

    if (!buffer->unload(this->group_id_))
    {
      LOG_ERROR("Failed to unload Dynamic Group Status group ID");
      return false;
    }

    return true;
}

DynamicGroupStatus::DynamicGroupStatus(void)
{
  this->init();
}
DynamicGroupStatus::~DynamicGroupStatus(void)
{

}

void DynamicGroupStatus::init(void)
{
  this->groups_.clear();
}

bool DynamicGroupStatus::operator==(const DynamicGroupStatus &rhs) const
{
  return (this->groups_ == rhs.groups_);
}

bool DynamicGroupStatus::load(ByteArray *buffer)
{
  LOG_COMM("Executing Dynamic Group Status load");

  if (!buffer->load(this->getNumGroups()))
  {
    LOG_ERROR("Failed to load Dynamic Group Status num_groups");
    return false;
  }

  for (int i=0; i<this->groups_.size(); ++i)
  {
    if (!this->groups_[i].load(buffer))
    {
      LOG_ERROR("Failed to load Dynamic Group Status group %d", this->groups_[i].getGroupID());
      return false;
    }
  }

  LOG_COMM("Dynamic Group Status successfully loaded");
  return true;
}

bool DynamicGroupStatus::unloadFront(ByteArray *buffer)
{
  int num_groups;

  LOG_COMM("Executing Dynamic Group Status unloadFront");

  if (!buffer->unloadFront(num_groups))
  {
    LOG_ERROR("Failed to unload Dynamic Group Status num_groups");
    return false;
  }

  for (int i=0; i<num_groups; ++i)
  {
    DynamicGroupStatusGrp group;

    if (!group.unloadFront(buffer))
    {
      LOG_ERROR("Failed to unload Dynamic Group Status group (index %d)", i);
      return false;
    }

    if (this->hasGroupID(group.getGroupID()))
    {
      LOG_ERROR("Duplicate data received for group id %d", group.getGroupID());
      return false;
    }

    this->addGroup(group);
  }

  LOG_COMM("Dynamic Group Status successfully unloaded (from FRONT)");
  return true;
}

bool DynamicGroupStatus::unload(ByteArray *buffer)
{
  LOG_ERROR("DynamicGroupStatus.unload() unsupported.  Use unloadFront() instead.");
  return false;
}

}
}

