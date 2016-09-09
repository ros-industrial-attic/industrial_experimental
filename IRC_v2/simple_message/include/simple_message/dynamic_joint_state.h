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

#ifndef DYNAMIC_JOINT_STATE_H
#define DYNAMIC_JOINT_STATE_H

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

namespace industrial
{
namespace dynamic_joint_state
{

namespace FieldTypes
{
enum ValidFieldType
{
  POSITION             = 0,
  VELOCITY             = 1,
  ACCELERATION         = 2,
  EFFORT               = 3,
  POSITION_DESIRED     = 4,
  POSITION_ERROR       = 5,
  VELOCITY_DESIRED     = 6,
  VELOCITY_ERROR       = 7,
  ACCELERATION_DESIRED = 8,
  ACCELERATION_ERROR   = 9,
  EFFORT_DESIRED       = 10,
  EFFORT_ERROR         = 11,
  end
};
}
typedef FieldTypes::ValidFieldType ValidFieldType;


/**

 * \brief Class encapsulating dynamic joint state group-data.  The state data
 * captures the current robot joint-state, and is used to feed both
 * JointState and FollowJointTrajectoryFeedback messages in ROS.
 *
 * This class is similar to the simple_message joint_feedback class, but this
 * class provides support for variable-length (numJoints) data arrays.  This is
 * intended to represent data for a single motion group inside a higher-level 
 * dynamic joint state data object.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes.
 * Unlike most simple-message types, this message is NOT a fixed-length message,
 * depending on the contents (numJoints).
 *
 *   member:             type                                      size
 *   group_id         (industrial::shared_types::shared_int)    4  bytes
 *   num_joints       (industrial::shared_types::shared_int)    4  bytes
 *   valid_fields     (industrial::shared_types::shared_int)    4  bytes
 *   positions        (industrial::shared_types::shared_real)   4 * num_joints
 *   velocities       (industrial::shared_types::shared_real)   4 * num_joints
 *   accelerations    (industrial::shared_types::shared_real)   4 * num_joints
 *   effort           (industrial::shared_types::shared_real)   4 * num_joints
 *   position_desired (industrial::shared_types::shared_real)   4 * num_joints
 *   position_error   (industrial::shared_types::shared_real)   4 * num_joints
 *   velocity_desired (industrial::shared_types::shared_real)   4 * num_joints
 *   velocity_error   (industrial::shared_types::shared_real)   4 * num_joints
 *   accel_desired    (industrial::shared_types::shared_real)   4 * num_joints
 *   accel_error      (industrial::shared_types::shared_real)   4 * num_joints
 *   effort_desired   (industrial::shared_types::shared_real)   4 * num_joints
 *   effort_error     (industrial::shared_types::shared_real)   4 * num_joints
  *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class DynamicJointStateGrp : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor (empty data)
   */
  DynamicJointStateGrp(void);

  /**
   * \brief Constructor - known # of joints
   * \param num_joints number of joints
   * \param group_id group ID
   */
  DynamicJointStateGrp(industrial::shared_types::shared_int num_joints,
                       industrial::shared_types::shared_int group_id = 0)
  {
    init(num_joints, group_id);
  }

  /**
   * \brief Destructor
   */
  ~DynamicJointStateGrp(void);

  /**
   * \brief Initializes an empty dynamic joint state group-data
   * \param num_joints number of joints
   * \param group_id group ID
   */
  bool init(industrial::shared_types::shared_int num_joints,
            industrial::shared_types::shared_int group_id = 0);

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
   * \brief Sets actual-position data
   *
   * \param actual new actual-position data
   * \return FALSE if invalid
   */
  bool setActualPositions(const std::vector<industrial::shared_types::shared_real> &actual)
  {
    return setDynData(FieldTypes::POSITION, actual);
  }

  /**
   * \brief Returns a copy of the actual-position data
   *
   * \param actual returned position data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getActualPositions(std::vector<industrial::shared_types::shared_real> &actual) const
  {
    return getDynData(FieldTypes::POSITION, actual);
  }

  /**
   * \brief Clears the actual-position data (reset to empty/invalid)
   */
  void clearActualPositions()
  {
    clearDynData(FieldTypes::POSITION);
  }

  /**
   * \brief Sets desired-position data
   *
   * \param desired new desired-position data
   * \return FALSE if invalid
   */
  bool setDesiredPositions(const std::vector<industrial::shared_types::shared_real> &desired)
  {
    return setDynData(FieldTypes::POSITION_DESIRED, desired);
  }

  /**
   * \brief Returns a copy of the desired-position data
   *
   * \param desired returned position data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getDesiredPositions(std::vector<industrial::shared_types::shared_real> &desired) const
  {
    return getDynData(FieldTypes::POSITION_DESIRED, desired);
  }

  /**
   * \brief Clears the desired-position data (reset to empty/invalid)
   */
  void clearDesiredPositions()
  {
    clearDynData(FieldTypes::POSITION_DESIRED);
  }

  /**
   * \brief Sets position-error data
   *
   * \param errors new position-error data
   * \return FALSE if invalid
   */
  bool setPositionErrors(const std::vector<industrial::shared_types::shared_real> &errors)
  {
    return setDynData(FieldTypes::POSITION_ERROR, errors);
  }

  /**
   * \brief Returns a copy of the position-error data
   *
   * \param errors returned position data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getPositionErrors(std::vector<industrial::shared_types::shared_real> &errors) const
  {
    return getDynData(FieldTypes::POSITION_ERROR, errors);
  }

  /**
   * \brief Clears the position-error data (reset to empty/invalid)
   */
  void clearPositionErrors()
  {
    clearDynData(FieldTypes::POSITION_ERROR);
  }

  /**
   * \brief Sets actual-velocity data
   *
   * \param actual new actual-velocity data
   * \return FALSE if invalid
   */
  bool setActualVelocities(const std::vector<industrial::shared_types::shared_real> &actual)
  {
    return setDynData(FieldTypes::VELOCITY, actual);
  }

  /**
   * \brief Returns a copy of the actual-velocity data
   *
   * \param actual returned velocity data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getActualVelocities(std::vector<industrial::shared_types::shared_real> &actual) const
  {
    return getDynData(FieldTypes::VELOCITY, actual);
  }

  /**
   * \brief Clears the actual-velocity data (reset to empty/invalid)
   */
  void clearActualVelocities()
  {
    clearDynData(FieldTypes::VELOCITY);
  }

  /**
   * \brief Sets desired-velocity data
   *
   * \param desired new desired-velocity data
   * \return FALSE if invalid
   */
  bool setDesiredVelocities(const std::vector<industrial::shared_types::shared_real> &desired)
  {
    return setDynData(FieldTypes::VELOCITY_DESIRED, desired);
  }

  /**
   * \brief Returns a copy of the desired-velocity data
   *
   * \param desired returned velocity data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getDesiredVelocities(std::vector<industrial::shared_types::shared_real> &desired) const
  {
    return getDynData(FieldTypes::VELOCITY_DESIRED, desired);
  }

  /**
   * \brief Clears the desired-velocity data (reset to empty/invalid)
   */
  void clearDesiredVelocities()
  {
    clearDynData(FieldTypes::VELOCITY_DESIRED);
  }

  /**
   * \brief Sets velocity-error data
   *
   * \param errors new velocity-error data
   * \return FALSE if invalid
   */
  bool setVelocityErrors(const std::vector<industrial::shared_types::shared_real> &errors)
  {
    return setDynData(FieldTypes::VELOCITY_ERROR, errors);
  }

  /**
   * \brief Returns a copy of the velocity-error data
   *
   * \param errors returned error data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getVelocityErrors(std::vector<industrial::shared_types::shared_real> &errors) const
  {
    return getDynData(FieldTypes::VELOCITY_ERROR, errors);
  }

  /**
   * \brief Clears the velocity-error data (reset to empty/invalid)
   */
  void clearVelocityErrors()
  {
    clearDynData(FieldTypes::VELOCITY_ERROR);
  }

  /**
   * \brief Sets actual-acceleration data
   *
   * \param actual new actual-acceleration data
   * \return FALSE if invalid
   */
  bool setActualAccelerations(const std::vector<industrial::shared_types::shared_real> &actual)
  {
    return setDynData(FieldTypes::ACCELERATION, actual);
  }

  /**
   * \brief Returns a copy of the actual-acceleration data
   *
   * \param actual returned acceleration data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getActualAccelerations(std::vector<industrial::shared_types::shared_real> &actual) const
  {
    return getDynData(FieldTypes::ACCELERATION, actual);
  }

  /**
   * \brief Clears the actual-acceleration data (reset to empty/invalid)
   */
  void clearActualAccelerations()
  {
    clearDynData(FieldTypes::ACCELERATION);
  }

  /**
   * \brief Sets desired-acceleration data
   *
   * \param desired new desired-acceleration data
   * \return FALSE if invalid
   */
  bool setDesiredAccelerations(const std::vector<industrial::shared_types::shared_real> &desired)
  {
    return setDynData(FieldTypes::ACCELERATION_DESIRED, desired);
  }

  /**
   * \brief Returns a copy of the desired-acceleration data
   *
   * \param desired returned acceleration data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getDesiredAccelerations(std::vector<industrial::shared_types::shared_real> &desired) const
  {
    return getDynData(FieldTypes::ACCELERATION_DESIRED, desired);
  }

  /**
   * \brief Clears the desired-acceleration data (reset to empty/invalid)
   */
  void clearDesiredAccelerations()
  {
    clearDynData(FieldTypes::ACCELERATION_DESIRED);
  }

  /**
   * \brief Sets acceleration-error data
   *
   * \param errors new acceleration-error data
   * \return FALSE if invalid
   */
  bool setAccelerationErrors(const std::vector<industrial::shared_types::shared_real> &errors)
  {
    return setDynData(FieldTypes::ACCELERATION_ERROR, errors);
  }

  /**
   * \brief Returns a copy of the acceleration-error data
   *
   * \param errors returned acceleration data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getAccelerationErrors(std::vector<industrial::shared_types::shared_real> &errors) const
  {
    return getDynData(FieldTypes::ACCELERATION_ERROR, errors);
  }

  /**
   * \brief Clears the acceleration-error data (reset to empty/invalid)
   */
  void clearAccelerationErrors()
  {
    clearDynData(FieldTypes::ACCELERATION_ERROR);
  }

  /**
   * \brief Sets actual-effort data
   *
   * \param actual new actual-effort data
   * \return FALSE if invalid
   */
  bool setActualEfforts(const std::vector<industrial::shared_types::shared_real> &actual)
  {
    return setDynData(FieldTypes::EFFORT, actual);
  }

  /**
   * \brief Returns a copy of the actual-effort data
   *
   * \param actual returned effort data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getActualEfforts(std::vector<industrial::shared_types::shared_real> &actual) const
  {
    return getDynData(FieldTypes::EFFORT, actual);
  }

  /**
   * \brief Clears the actual-effort data (reset to empty/invalid)
   */
  void clearActualEfforts()
  {
    clearDynData(FieldTypes::EFFORT);
  }

  /**
   * \brief Sets desired-effort data
   *
   * \param desired new desired-effort data
   * \return FALSE if invalid
   */
  bool setDesiredEfforts(const std::vector<industrial::shared_types::shared_real> &desired)
  {
    return setDynData(FieldTypes::EFFORT_DESIRED, desired);
  }

  /**
   * \brief Returns a copy of the desired-effort data
   *
   * \param desired returned effort data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getDesiredEfforts(std::vector<industrial::shared_types::shared_real> &desired) const
  {
    return getDynData(FieldTypes::EFFORT_DESIRED, desired);
  }

  /**
   * \brief Clears the desired-effort data (reset to empty/invalid)
   */
  void clearDesiredEfforts()
  {
    clearDynData(FieldTypes::EFFORT_DESIRED);
  }

  /**
   * \brief Sets effort-error data
   *
   * \param errors new effort-error data
   * \return FALSE if invalid
   */
  bool setEffortErrors(const std::vector<industrial::shared_types::shared_real> &errors)
  {
    return setDynData(FieldTypes::EFFORT_ERROR, errors);
  }

  /**
   * \brief Returns a copy of the effort-error data
   *
   * \param errors returned effort data (empty if validFields not set)
   * \return TRUE if valid data
   */
  bool getEffortErrors(std::vector<industrial::shared_types::shared_real> &errors) const
  {
    return getDynData(FieldTypes::EFFORT_ERROR, errors);
  }

  /**
   * \brief Clears the effort-error data (reset to empty/invalid)
   */
  void clearEffortErrors()
  {
    clearDynData(FieldTypes::EFFORT_ERROR);
  }

  /**
   * \brief check the validity state for a given field
   * \param field field to check
   * \return true if specified field contains valid data
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
  bool operator==(const DynamicJointStateGrp &rhs) const;

  /**
   * \brief != operator implementation
   *
   * \return true if not-equal
   */
  bool operator!=(const DynamicJointStateGrp &rhs) const
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
    unsigned int szArray = this->num_joints_ * szReal;
    int numValidArrays   = this->valid_fields_.count();

    return 3 * szInt                 // group_id, num_joints, valid_fields
         + numValidArrays * szArray; // var-length arrays
  }

private:


  /**
   * \brief Sets dynamic joint-array data
   *
   * \param[in] validFieldCode valid-field type code
   * \param[in] newVals new joint data
   * \return FALSE if invalid (length doesn't match num_joints)
   */
  bool setDynData(ValidFieldType validFieldCode,
                  const std::vector<industrial::shared_types::shared_real> &newVals);

  /**
   * \brief Returns dynamic joint-array data
   *
   * \param[in] validFieldCode valid-field type code
   * \param[out] result output result data
   * \return FALSE if no valid data
   */
  bool getDynData(ValidFieldType validFieldCode,
                 std::vector<industrial::shared_types::shared_real> &result) const;

  /**
   * \brief Clears dynamic joint-array data
   *
   * \param[in] validFieldCode valid-field type code
   */
  void clearDynData(ValidFieldType validFieldCode);

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
  std::bitset<16> valid_fields_;

  /**
   * \brief group ID
   */
  industrial::shared_types::shared_int group_id_;

  /**
   * \brief number of joints
   */
  industrial::shared_types::shared_int num_joints_;

  /**
   * \brief joint feedback state data
   */
  std::map<ValidFieldType, std::vector<industrial::shared_types::shared_real> > state_;

};

/**
 * \brief Class encapsulating dynamic joint state data.  The state data
 * captures the current robot joint-state, and is used to feed both
 * JointState and FollowJointTrajectoryFeedback messages in ROS.
 *
 * This class is similar to the simple_message joint_feedback class, but this
 * class provides support for multiple robot motion groups (multi-arm)
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
 *   group[]     (industrial::dynamic_joint_pt::DynamicJointStateGrp) varies
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class DynamicJointState : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor (empty data)
   */
  DynamicJointState(void);
  /**
   * \brief Destructor
   */
  ~DynamicJointState(void);

  /**
   * \brief Initializes a empty dynamic joint state
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
   * \brief Adds new group state data
   *
   * \param group group state data
   * \return TRUE if valid data; FALSE if duplicate group
   */
  bool addGroup(DynamicJointStateGrp &group)
  {
    // check to see if group exists already
    if (hasGroupID(group.getGroupID()))
      return false;

    this->groups_.push_back(group);
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
  DynamicJointStateGrp& getGroup(int idx)
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
  const DynamicJointStateGrp& getGroup(int idx) const
  {
    return this->groups_.at(idx);
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
  bool operator==(const DynamicJointState &rhs) const;

  /**
   * \brief != operator implementation
   *
   * \return true if not-equal
   */
  bool operator!=(const DynamicJointState &rhs) const
  {
    return !operator==(rhs);
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    unsigned int groupSize = 0;
    GroupList::iterator it;
    for (it=this->groups_.begin(); it!=this->groups_.end(); ++it)
      groupSize += it->byteLength();  // grpData

    return 2*sizeof(industrial::shared_types::shared_int) // seq, numGrp
         + groupSize;
  }

private:

  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

  typedef std::vector<DynamicJointStateGrp> GroupList;

  /**
   * \brief list of motion groups
   */
  GroupList groups_;


};

}
}

#endif /* DYNAMIC_JOINT_STATE_H */
