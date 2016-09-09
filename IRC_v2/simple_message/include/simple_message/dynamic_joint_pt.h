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

#ifndef DYNAMIC_JOINT_PT_H
#define DYNAMIC_JOINT_PT_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif
#include <bitset>
#include <map>
#include <vector>
#include <iostream>

namespace industrial
{
namespace dynamic_joint_pt
{

namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  START_TRAJECTORY_DOWNLOAD = -1,
  START_TRAJECTORY_STREAMING = -2,
  END_TRAJECTORY = -3,
  STOP_TRAJECTORY = -4
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

namespace FieldTypes
{
enum ValidFieldType
{
  POSITION     = 0,
  VELOCITY     = 1,
  ACCELERATION = 2,
  EFFORT       = 3,
  TIME         = 4,
};
}
typedef FieldTypes::ValidFieldType ValidFieldType;

/**

 * \brief Class encapsulating dynamic joint point group-data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt_full class, but this
 * class provides support for variable-length (numJoints) data arrays.  This is
 * intended to represent data for a single motion group inside a higher-level 
 * dynamic joint point data object.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes.
 * Unlike most simple-message types, this message is NOT a fixed-length message,
 * depending on the contents (numJoints).
 *
 *
 *   member:             type                                      size
 *   id              (industrial::shared_types::shared_int)    4  bytes
 *   num_joints      (industrial::shared_types::shared_int)    4  bytes
 *   valid_fields    (industrial::shared_types::shared_int)    4  bytes
 *   positions       (industrial::shared_types::shared_real)   4 * num_joints
 *   velocities      (industrial::shared_types::shared_real)   4 * num_joints
 *   accelerations   (industrial::shared_types::shared_real)   4 * num_joints
 *   effort          (industrial::shared_types::shared_real)   4 * num_joints
 *   time            (industrial::shared_types::shared_real)   4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class DynamicJointPtGrp : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor (empty data)
   */
  DynamicJointPtGrp(void);
  /**
   * \brief Constructor - known # of joints
   * \param num_joints number of joints
   */
  DynamicJointPtGrp(industrial::shared_types::shared_int num_joints);
  /**
   * \brief Destructor
   */
  ~DynamicJointPtGrp(void);

  /**
   * \brief Initializes an empty dynamic joint point group-data
   * \param num_joints number of joints
   */
  bool init(industrial::shared_types::shared_int num_joints);

  /**
   * \brief Returns number of joints
   *
   * \return number of joints
   */
  industrial::shared_types::shared_int getNumJoints() const
  {
    return this->num_joints_;
  }

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
   * \brief Sets trajectory point timestamp
   *
   * \param time new time value
   * \return FALSE if invalid time value (< 0)
   */
  bool setTime(industrial::shared_types::shared_real time)
  {
    if (time < 0)
      return false;

    this->time_ = time;
    valid_fields_.set(FieldTypes::TIME, true);
    return true;
  }

  /**
   * \brief Returns trajectory point timestamp
   *
   * \param time returned time value (0 if not valid)
   * \return TRUE if valid data
   */
  bool getTime(industrial::shared_types::shared_real &time) const
  {
    time = this->time_;
    return isValid(FieldTypes::TIME);
  }

  /**
   * \brief Clears the time data (reset to empty/invalid)
   */
  void clearTime()
  {
    this->time_ = 0.0;
    valid_fields_.set(FieldTypes::TIME, false);
  }

  /**
   * \brief Sets joint position data
   *
   * \param positions new joint position data
   * \return FALSE if invalid (length doesn't match num_joints)
   */
  bool setPositions(const std::vector<industrial::shared_types::shared_real> &positions)
  {
    if (this->num_joints_ != positions.size())
      return false;

    this->pos_ = positions;
    valid_fields_.set(FieldTypes::POSITION, true);
    return true;
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param positions returned joint position (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getPositions(std::vector<industrial::shared_types::shared_real> &positions) const
  {
    positions = this->pos_;
    return isValid(FieldTypes::POSITION);
  }

  /**
   * \brief Clears the position data (reset to empty/invalid)
   */
  void clearPositions()
  {
    this->pos_.clear();
    valid_fields_.set(FieldTypes::POSITION, false);
  }

  /**
   * \brief Sets joint velocity data
   *
   * \param velocities new joint velocity data
   * \return FALSE if invalid (length doesn't match num_joints)
   */
  bool setVelocities(const std::vector<industrial::shared_types::shared_real> &velocities)
  {
    if (this->num_joints_ != velocities.size())
      return false;

    this->vel_ = velocities;
    valid_fields_.set(FieldTypes::VELOCITY, true);
    return true;
  }

  /**
   * \brief Returns a copy of the velocity data
   *
   * \param velocities returned joint velocity (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getVelocities(std::vector<industrial::shared_types::shared_real> &velocities) const
  {
    velocities = this->vel_;
    return isValid(FieldTypes::VELOCITY);
  }

  /**
   * \brief Clears the velocity data (reset to empty/invalid)
   */
  void clearVelocities()
  {
    this->vel_.clear();
    valid_fields_.set(FieldTypes::VELOCITY, false);
  }


  /**
   * \brief Sets joint acceleration data
   *
   * \param accelerations new joint acceleration data
   * \return FALSE if invalid (length doesn't match num_joints)
   */
  bool setAccelerations(const std::vector<industrial::shared_types::shared_real> &accelerations)
  {
    if (this->num_joints_ != accelerations.size())
      return false;

    this->accel_ = accelerations;
    valid_fields_.set(FieldTypes::ACCELERATION, true);
    return true;
  }

  /**
   * \brief Returns a copy of the acceleration data
   *
   * \param accelerations returned joint accelerations (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getAccelerations(std::vector<industrial::shared_types::shared_real> &accelerations) const
  {
    accelerations = this->accel_;
    return isValid(FieldTypes::ACCELERATION);
  }

  /**
   * \brief Clears the acceleration data (reset to empty/invalid)
   */
  void clearAccelerations()
  {
    this->accel_.clear();
    valid_fields_.set(FieldTypes::ACCELERATION, false);
  }

  /**
   * \brief Sets joint effort data
   *
   * \param effort new joint effort data
   * \return FALSE if invalid (length doesn't match num_joints)
   */
  bool setEfforts(const std::vector<industrial::shared_types::shared_real> &efforts)
  {
    if (this->num_joints_ != efforts.size())
      return false;

    this->effort_ = efforts;
    valid_fields_.set(FieldTypes::EFFORT, true);
    return true;
  }

  /**
   * \brief Returns a copy of the effort data
   *
   * \param efforts returned joint efforts (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getEfforts(std::vector<industrial::shared_types::shared_real> &efforts) const
  {
    efforts = this->effort_;
    return isValid(FieldTypes::EFFORT);
  }

  /**
   * \brief Clears the effort data (reset to empty/invalid)
   */
  void clearEfforts()
  {
    this->effort_.clear();
    valid_fields_.set(FieldTypes::EFFORT, false);
  }

  /**
   * \brief check the validity state for a given field
   * @param field field to check
   * @return true if specified field contains valid data
   */
  bool isValid(ValidFieldType field) const
  {
    return valid_fields_.test(field);
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
  bool operator==(const DynamicJointPtGrp &rhs) const;

  /**
   * \brief != operator implementation
   *
   * \return true if not-equal
   */
  bool operator!=(const DynamicJointPtGrp &rhs) const
  {
    return !operator==(rhs);
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    const unsigned int szInt   = sizeof(industrial::shared_types::shared_int);
    const unsigned int szReal  = sizeof(industrial::shared_types::shared_real);
    bool hasTime         = isValid(FieldTypes::TIME);
    unsigned int szArray = this->num_joints_ * szReal;
    int numValidArrays   = this->valid_fields_.count() - hasTime;

    std::cout << "**BL: numValidArrays " << numValidArrays;
    std::cout << ", numJoints "<< this->num_joints_ << ", szReal " << szReal << std::endl;
    return 3 * szInt                 // group_id, num_joints, valid_fields
         + (hasTime ? szReal : 0)    // time
         + numValidArrays * szArray; // var-length arrays
  }

private:

  /**
   * \brief load array data into byte-buffer
   *
   * \param data array data to load
   * \param buffer buffer to receive data
   * \return FALSE if errors occurred
   */
   bool loadArray(const std::vector<industrial::shared_types::shared_real> &data, 
                  industrial::byte_array::ByteArray *buffer);

  /**
   * \brief unload array data from byte-buffer
   *   - starting at front of buffer
   *
   * \param data array to receive data
   * \param len  number of data points
   * \param buffer buffer containing source data
   * \return FALSE if errors occurred
   */
   bool unloadArray(std::vector<industrial::shared_types::shared_real> &data,
                    unsigned int len,
                    industrial::byte_array::ByteArray *buffer);

  /**
   * \brief bit-mask of (optional) fields that have valid data
   * \see enum FieldTypes
   */
  std::bitset<8> valid_fields_;

  /**
   * \brief group ID
   */
  industrial::shared_types::shared_int group_id_;

  /**
   * \brief number of joints
   */
  industrial::shared_types::shared_int num_joints_;

  /**
   * \brief trajectory point timestamp
   *        Typically, time_from_start of this trajectory (in seconds)
   */
  industrial::shared_types::shared_real time_;

  /**
   * \brief trajectory point positional data
   */
  std::vector<industrial::shared_types::shared_real> pos_;
  /**
   * \brief trajectory point velocity data
   */
  std::vector<industrial::shared_types::shared_real> vel_;
  /**
   * \brief trajectory point acceleration data
   */
  std::vector<industrial::shared_types::shared_real> accel_;
  /**
   * \brief trajectory point effort data
   */
  std::vector<industrial::shared_types::shared_real> effort_;

};

/**
 * \brief Class encapsulating dynamic joint point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt_full class, but this
 * class provides support for multiple robot motion groups (coord motion)
 * and variable-length (numJoints) data arrays.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes.
 * Unlike most simple-message types, this message is NOT a fixed-length message,
 * depending on the contents (numGroups, numJoints).
 *
 *   member:     type:                                                size:
 *   sequence    (industrial::shared_types::shared_int)               4  bytes
 *   num_groups  (industrial::shared_types::shared_int)               4  bytes
 *   group[]
 *     data      (industrial::dynamic_joint_pt::DynamicJointPtGrp)    varies
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class DynamicJointPt : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor (empty data)
   */
  DynamicJointPt(void);
  /**
   * \brief Destructor
   */
  ~DynamicJointPt(void);

  /**
   * \brief Initializes a empty dynamic joint point
   *
   */
  void init();

  /**
   * \brief Sets trajectory sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns trajectory sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence() const
  {
    return this->sequence_;
  }

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
   * \brief Adds new group trajectory data
   *
   * \param group group trajectory data
   * \return TRUE if valid data; FALSE if duplicate group
   */
  bool addGroup(const DynamicJointPtGrp &group)
  {
    // check to see if group exists already
    if (hasGroupID(group.getGroupID()))
      return false;

    this->groups_.push_back(group);
  }

  /**
   * \brief Gets group trajectory data
   *
   * Returns a reference to the group data, for editing.
   *   - if doesn't exist, will throw an exception
   *
   * \param id group id
   * \return group data
   */
  DynamicJointPtGrp& getGroup(industrial::shared_types::shared_int id)
  {
    return this->groups_.at(id);
  }

  /**
   * \brief Gets group trajectory data
   *
   * Returns a const-reference to the group data, for read only.
   *   - if doesn't exist, will throw an exception
   *
   * \param id group id
   * \return group data
   */
  const DynamicJointPtGrp& getGroup(industrial::shared_types::shared_int id) const
  {
    return this->groups_.at(id);
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
  bool operator==(const DynamicJointPt &rhs) const;

  /**
   * \brief != operator implementation
   *
   * \return true if not-equal
   */
  bool operator!=(const DynamicJointPt &rhs) const
  {
    return !operator==(rhs);
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    unsigned int groupSize = 0;

    for (size_t i=0; i<this->groups_.size(); ++i)
      groupSize += this->groups_[i].byteLength();

    return 2*sizeof(industrial::shared_types::shared_int) // seq, numGrp
         + groupSize;
  }

private:

  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

  typedef std::vector<DynamicJointPtGrp> GroupList;

  /**
   * \brief list of motion groups
   */
  GroupList groups_;


};

}
}

#endif /* DYNAMIC_JOINT_PT_H */
