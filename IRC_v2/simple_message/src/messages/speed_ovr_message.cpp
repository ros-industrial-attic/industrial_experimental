/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Southwest Research Institute, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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
#include "simple_message/messages/speed_ovr_message.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;
using industrial::byte_array::ByteArray;
using industrial::simple_message::SimpleMessage;

namespace industrial
{
namespace speed_ovr_message
{

void SpeedOvrMessage::init()
{
  this->setMessageType(industrial::simple_message::StandardMsgTypes::SPEED_OVR);
  this->speed_ovr_ = 0;
}

void SpeedOvrMessage::init(industrial::shared_types::shared_int speed_ovr)
{
  this->init();
  this->speed_ovr_ = speed_ovr;
}

bool SpeedOvrMessage::init(SimpleMessage & msg)
{
  ByteArray data = msg.getData();
  this->init();

  return data.unload(this->speed_ovr_);
}

bool SpeedOvrMessage::load(industrial::byte_array::ByteArray *buffer)
{
  if (buffer->load(this->speed_ovr_))
    return true;

  LOG_ERROR("Failed to load SpeedOvr data");
  return false;
}

bool SpeedOvrMessage::unload(industrial::byte_array::ByteArray *buffer)
{
  if (buffer->unload(this->speed_ovr_))
    return true;

  LOG_ERROR("Failed to unload SpeedOvr data");
  return false;
}

}
}

