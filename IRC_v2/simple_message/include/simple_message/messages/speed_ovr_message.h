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

#ifndef SPEED_OVR_MESSAGE_H
#define SPEED_OVR_MESSAGE_H

#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/typed_message.h"

namespace industrial
{
namespace speed_ovr_message
{

/**
 * \brief Class encapsulated speed_ovr message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class SpeedOvrMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  SpeedOvrMessage(void) { this->init(); }

  /**
   * \brief Destructor
   *
   */
  ~SpeedOvrMessage(void) {};

  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a speed value
   *
   * \param speed_ovr speed override value (0-100)%
   *
   */
  void init(industrial::shared_types::shared_int speed_ovr);

  /**
   * \brief Initializes a new empty message
   *
   */
  void init();

  /**
   * \brief Sets speed override
   *
   * \param speed_ovr speed override value (0-100)%
   */
  void setSpeedOvr(industrial::shared_types::shared_int speed_ovr)
  {
    this->speed_ovr_ = speed_ovr;
  }

  /**
   * \brief returns the speed override value
   *
   * \return speed override value (0-100)%
   */
  industrial::shared_types::shared_int getSpeedOvr()
  {
    return this->speed_ovr_;
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief speed override value
   */
  industrial::shared_types::shared_int speed_ovr_;
};

}
}

#endif /* SPEED_OVR_MESSAGE_H */
