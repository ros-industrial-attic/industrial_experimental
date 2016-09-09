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
#include "simple_message/messages/dynamic_joint_pt_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "dynamic_joint_pt_message.h"
#include "joint_data.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::dynamic_joint_pt;

namespace industrial
{
namespace dynamic_joint_pt_message
{

DynamicJointPtMessage::DynamicJointPtMessage(void)
{
  this->init();
}

DynamicJointPtMessage::~DynamicJointPtMessage(void)
{

}

bool DynamicJointPtMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (this->point_.unloadFront(&data))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload dynamic joint pt data");
  }
  return rtn;
}

void DynamicJointPtMessage::init(industrial::dynamic_joint_pt::DynamicJointPt & point)
{
  this->init();
  this->point_ = point;
}

void DynamicJointPtMessage::init()
{
  this->setMessageType(StandardMsgTypes::DYNAMIC_JOINT_PT);
  this->point_.init();
}


bool DynamicJointPtMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing dynamic joint pt. message load");
  if (buffer->load(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load dynamic joint pt data");
  }
  return rtn;
}

bool DynamicJointPtMessage::unload(ByteArray *buffer)
{
  LOG_ERROR("DynamicJointPtMessage.unload() unsupported.  Use unloadFront() instead.");
  return false;
}

bool DynamicJointPtMessage::unloadFront(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing dynamic joint pt message unloadFront");

  if (this->point_.unloadFront(buffer))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload dynamic joint pt data");
  }
  return rtn;
}

}
}

