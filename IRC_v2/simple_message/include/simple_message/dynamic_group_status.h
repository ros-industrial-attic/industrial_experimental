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

#ifndef DYNAMIC_GROUP_STATUS_H
#define DYNAMIC_GROUP_STATUS_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/robot_status.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "robot_status.h"
#endif
#include <map>

namespace industrial
{
namespace dynamic_group_status
{

/**
 * \brief Class encapsulating dynamic group status group-data.
 *
 * This class is similar to the simple_message robot_status class, but this
 * class adds support for a group_id field.  This is
 * intended to represent data for a single motion group inside a higher-level 
 * dynamic joint state data object.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest).
 *
 *   member:             type                                      size
 *   group_id         (industrial::shared_types::shared_int)    4  bytes
 *   status           (industrial::robot_status)                28 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class DynamicGroupStatusGrp : public industrial::robot_status::RobotStatus
{
public:

  /**
   * \brief Sets group ID
   *
   * \param id new group ID
   * \return FALSE if invalid
   */
  bool setGroupID(industrial::shared_types::shared_int id)
  {
    this->group_id_ = id;
    return true;
  }

  /**
   * \brief Returns the group ID
   *
   * \return group ID
   */
  industrial::shared_types::shared_int getGroupID() const
  {
    return this->group_id_;
  }

  /**
   * \brief unload data from the FRONT of a byte-buffer
   *
   * \param buffer buffer containing source data
   * \return FALSE if errors occurred
   */
  bool unloadFront(industrial::byte_array::ByteArray *buffer);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(const DynamicGroupStatusGrp &rhs) const;

  /**
   * \brief != operator implementation
   *
   * \return true if not-equal
   */
  bool operator!=(const DynamicGroupStatusGrp &rhs) const
  {
    return !operator==(rhs);
  }


  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(group_id_) + industrial::robot_status::RobotStatus::byteLength();
  }

private:

  /**
   * \brief group ID
   */
  industrial::shared_types::shared_int group_id_;
};

/**
 * \brief Class encapsulating dynamic robot-status data.  This data is meant to
 * mirror the industrial_msgs/RobotStatus message.
 *
 * This class is similar to the simple_message robot_status class, but this
 * class provides support for multiple robot motion groups.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes.
 * Unlike most simple-message types, this message is NOT a fixed-length message,
 * depending on the contents (numGroups).
 *
 *   member:     type:                                      size:
 *   num_groups  (industrial::shared_types::shared_int)     4  bytes
 *   group[]     (DynamicGroupStatusGrp)                    32 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class DynamicGroupStatus : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor (empty data)
   */
  DynamicGroupStatus(void);
  /**
   * \brief Destructor
   */
  ~DynamicGroupStatus(void);

  /**
   * \brief Initializes a empty dynamic group status
   *
   */
  void init();

  /**
   * \brief Returns number of groups
   *
   * \return number of groups
   */
  industrial::shared_types::shared_int getNumGroups() const
  {
    return this->groups_.size();
  }

  /**
   * \brief Checks to see if data has been set for specified group
   *
   * \param id group id
   * \return TRUE if exists; FALSE if not defined
   */
  bool hasGroupID(industrial::shared_types::shared_int id) const
  {
    for (size_t i=0; i<this->groups_.size(); ++i)
      if (this->groups_[i].getGroupID() == id)
        return true;

    return false;
  }  

  /**
   * \brief Adds new group status data
   *
   * \param status group status data
   * \return TRUE if valid data; FALSE if duplicate group
   */
  bool addGroup(DynamicGroupStatusGrp &status)
  {
    // check to see if group exists already
    if (hasGroupID(status.getGroupID()))
      return false;

    this->groups_.push_back(status);
    return true;
  }

  /**
   * \brief Gets group state data
   *
   * Returns a reference to the group data, for editing.
   *   - if doesn't exist, will throw an exception
   *
   * \param idx index in group-list
   * \return group data
   */
  DynamicGroupStatusGrp& getGroup(int idx)
  {
      return this->groups_.at(idx);
  }

  /**
   * \brief Gets group state data
   *
   * Returns a const-reference to the group data, for read only.
   *   - if doesn't exist, will throw an exception
   *
   * \param idx index in group-list
   * \return group data
   */
  const DynamicGroupStatusGrp& getGroup(int idx) const
  {
    return this->groups_.at(idx);
  }

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(const DynamicGroupStatus &rhs) const;

  /**
   * \brief != operator implementation
   *
   * \return true if not-equal
   */
  bool operator!=(const DynamicGroupStatus &rhs) const
  {
    return !operator==(rhs);
  }

  /**
   * \brief unload data from the FRONT of a byte-buffer
   *
   * \param buffer buffer containing source data
   * \return FALSE if errors occurred
   */
  bool unloadFront(industrial::byte_array::ByteArray *buffer);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    unsigned int groupSize = 0;

    for (int i=0; i<this->groups_.size(); ++i)
      groupSize += this->groups_[i].byteLength();  // grpData

    return sizeof(industrial::shared_types::shared_int) // numGrp
         + groupSize;
  }

private:

  typedef std::vector<DynamicGroupStatusGrp> GroupList;

  /**
   * \brief list of motion groups
   */
  GroupList groups_;


};

}
}

#endif /* DYNAMIC_GROUP_STATUS_H */
