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

#include <algorithm>

#include "industrial_robot_client2/joint_relay_handler.h"
#include "simple_message/log_wrapper.h"

using industrial::dynamic_joint_state::DynamicJointState;
using industrial::shared_types::shared_int;
using industrial::shared_types::shared_real;
using namespace industrial::simple_message;
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::JointGroupMap;

namespace industrial_robot_client2
{
namespace joint_relay_handler
{

bool JointRelayHandler::init(SmplMsgConnection* connection,
                             const std::vector<JointGroupMap>& joint_map)
{
  // save joint map for later use
  this->joint_map_ = joint_map;

  // clear existing publisher definitions
  this->pub_control_state_.clear();
  this->pub_sensor_state_.clear();
  
  // create new publishers for each specified namespace
  std::vector<JointGroupMap>::const_iterator map;
  for (map=joint_map.begin(); map!=joint_map.end(); ++map)
  {
    const std::string& ns = map->ns;
    ros::NodeHandle nh(ns);  // create a nodeHandle for specified namespace

    // create new publishers, if none have already been defined for this namespace
    if (this->pub_control_state_.count(ns) == 0)
    {
      this->pub_control_state_[ns] = nh.advertise<FeedbackMsg>("feedback_states", 1);
      this->pub_sensor_state_[ns] = nh.advertise<StateMsg>("joint_states", 1);
    }
  }

  return init((int)StandardMsgTypes::DYNAMIC_JOINT_STATE, connection);
}

bool JointRelayHandler::internalCB(SimpleMessage& in)
{
  SimpleMsg joint_msg;

  // create specific typed-message from generic input message
  if (!joint_msg.init(in))
  {
    LOG_ERROR("Failed to initialize dynamic joint state message");
    return false;
  }

  return internalCB(joint_msg);
}

bool JointRelayHandler::internalCB(SimpleMsg& in)
{
  std::map<Namespace, FeedbackMsg> control_msgs;
  std::map<Namespace, StateMsg> sensor_msgs;

  // check incoming SimpleMessage for validity
  bool rtn = check_message(in);

  // Convert incoming SimpleMessage to outgoing ROS messages
  if (rtn)
    rtn = create_messages(in, control_msgs, sensor_msgs);

  // Publish outgoing ROS messages
  std::map<Namespace, FeedbackMsg>::const_iterator mapIt;
  for (mapIt=control_msgs.begin(); rtn && mapIt!=control_msgs.end(); ++mapIt)
  {
    const Namespace& ns = mapIt->first;
    this->pub_control_state_[ns].publish(control_msgs[ns]);
    this->pub_sensor_state_[ns].publish(sensor_msgs[ns]);
  }

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == in.getMessageType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

bool JointRelayHandler::check_message(const SimpleMsg& msg)
{
  bool rtn = check_duplicate_groups(msg)
             && check_joint_map_match(msg);

  if (!rtn)
    LOG_ERROR("Failed to parse incoming DynamicJointState message");
  return rtn;
}

bool JointRelayHandler::check_duplicate_groups(const SimpleMsg& msg)
{
  std::set<shared_int> ids;
  const DynamicJointState& state = msg.state_;

  // loop over each group in message
  for (size_t i=0; i<state.getNumGroups(); ++i)
  {
    shared_int id = state.getGroup(i).getGroupID();

    // check for duplicate group IDs
    if (ids.count(id) > 0)
    {
      LOG_ERROR("Duplicate IDs detected in DynamicJointState message");
      return false;
    }
    else
      ids.insert(id);
  }

  return true;
}

bool JointRelayHandler::check_joint_map_match(const SimpleMsg& msg)
{
  const DynamicJointState& state = msg.state_;

  // loop over each group in message
  for (size_t i=0; i<state.getNumGroups(); ++i)
  {
    bool found=false;
    shared_int id = state.getGroup(i).getGroupID();

    // check for any matching joint_map entry
    for (size_t j=0; j<this->joint_map_.size(); ++j)
    {
      if (id == this->joint_map_[j].group_id)
      {
        found = true;

        // check for matching # of joints
        if (state.getGroup(i).getNumJoints() != this->joint_map_[j].joints.size())
        {
           ROS_ERROR("Number of joints doesn't match joint_map");
           return false;
        }
      }
    }

    // log error if no matching group defined in joint_map
    if (!found)
    {
      LOG_ERROR("No matching joint_map found for group #%d", id);
      return false;
    }

  }

  return true;
}


bool JointRelayHandler::create_messages(const SimpleMsg& msg_in,
                                  std::map<Namespace, FeedbackMsg>& control_msgs,
                                  std::map<Namespace, StateMsg>& sensor_msgs)
{
  control_msgs.clear();
  sensor_msgs.clear();

  // create control_msgs ROS messages from incoming simple_message
  if (!create_messages(msg_in, control_msgs))
    return false;

  // create sensor_msgs ROS messages from control_msgs ROS messages
  std::map<Namespace, FeedbackMsg>::const_iterator kvpIt;
  for (kvpIt=control_msgs.begin(); kvpIt!=control_msgs.end(); ++kvpIt)
  {
    const Namespace&  ns          = kvpIt->first;
    const FeedbackMsg& control_msg = kvpIt->second;

    StateMsg& sensor_msg = sensor_msgs[ns];
    sensor_msg.header    = control_msg.header;
    sensor_msg.name      = control_msg.joint_names;
    sensor_msg.position  = control_msg.actual.positions;
    sensor_msg.velocity  = control_msg.actual.velocities;
    sensor_msg.effort    = control_msg.actual.effort;
  }

  return true;
}

bool JointRelayHandler::create_messages(const SimpleMsg& msg_in,
                               std::map<Namespace, FeedbackMsg>& control_msgs)
{
  // copy raw message data into ROS msg structure
  //   - use one msg per joint_map entry
  //   - this is "better" than one msg per group_id,
  //       because it allows for different joint_name mappings for the same group
  //   - the distinction only matters if duplicate mapping entries for one group,
  //       which is rare.
  std::vector<FeedbackMsg> working_msgs;
  if (!copy_data(msg_in, working_msgs))
  {
    LOG_ERROR("Failed to copy data to ROS message structure");
    return false;
  }

  // process each message separately
  for (int i=0; i<working_msgs.size(); ++i)
  {
    // apply transform to joint positions, if required
    if (!transform(this->joint_map_[i].group_id, working_msgs[i]))
    {
      LOG_ERROR("Failed to transform joint positions");
      return false;
    }

    // select specific joints for publishing
    if (!select(this->joint_map_[i].group_id, working_msgs[i]))
    {
      LOG_ERROR("Failed to select joints for publishing");
      return false;
    }
  }

  // aggregate per-group messages into per-namespace messages
  if (!aggregate(working_msgs, control_msgs))
  {
    LOG_ERROR("Failed to aggregate multi-group messages for publishing");
    return false;
  }

  return true;
}

bool JointRelayHandler::copy_data(const SimpleMsg& in,
                                  std::vector<FeedbackMsg>& out)
{
  // reset output array with empty msgs, one for each group-map rule
  out = std::vector<FeedbackMsg>(this->joint_map_.size());

  // ensure all messages use the same header/timestamp
  std_msgs::Header header;
  header.stamp = ros::Time::now();

  // process each group in the incoming message
  for (size_t msg_idx=0; msg_idx<in.state_.getNumGroups(); ++msg_idx)
  {
    const SimpleMsgGrpData& in_grp = in.state_.getGroup(msg_idx);

    // check joint_map for matching group IDs
    for (size_t map_idx=0; map_idx<this->joint_map_.size(); ++map_idx)
    {
      const JointGroupMap& map = this->joint_map_[map_idx];

      // if group IDs match, copy data
      if (in_grp.getGroupID() == map.group_id)
      {
        copy_data(map.joints, in_grp, out[map_idx]);
        out[map_idx].header = header;
      }
    }
  }

  return true;
}

bool JointRelayHandler::copy_data(const std::vector<std::string>& joint_names,
                                  const SimpleMsgGrpData& in, FeedbackMsg& out)
{
  out = FeedbackMsg();  // clear/reset message contents

  out.joint_names = joint_names;

  // NOTE: the temporary "vals" array is required to convert between
  // the different array types (float vs. double) used in SimpleMsg/ROS messages
  std::vector<shared_real> vals;

  if (in.getDesiredPositions(vals))
    out.desired.positions = std::vector<double>(vals.begin(), vals.end());
  if (in.getDesiredVelocities(vals))
    out.desired.velocities = std::vector<double>(vals.begin(), vals.end());
  if (in.getDesiredAccelerations(vals))
    out.desired.accelerations = std::vector<double>(vals.begin(), vals.end());
  if (in.getDesiredEfforts(vals))
    out.desired.effort = std::vector<double>(vals.begin(), vals.end());

  if (in.getActualPositions(vals))
    out.actual.positions = std::vector<double>(vals.begin(), vals.end());
  if (in.getActualVelocities(vals))
    out.actual.velocities = std::vector<double>(vals.begin(), vals.end());
  if (in.getActualAccelerations(vals))
    out.actual.accelerations = std::vector<double>(vals.begin(), vals.end());
  if (in.getActualEfforts(vals))
    out.actual.effort = std::vector<double>(vals.begin(), vals.end());

  if (in.getPositionErrors(vals))
    out.error.positions = std::vector<double>(vals.begin(), vals.end());
  if (in.getVelocityErrors(vals))
    out.error.velocities = std::vector<double>(vals.begin(), vals.end());
  if (in.getAccelerationErrors(vals))
    out.error.accelerations = std::vector<double>(vals.begin(), vals.end());
  if (in.getEffortErrors(vals))
    out.error.effort = std::vector<double>(vals.begin(), vals.end());
}

template<typename T>
void eraseIdx(std::vector<T>&v, int idx)
{
  if (!v.empty())
    v.erase(v.begin()+idx);
}

void eraseIdx(trajectory_msgs::JointTrajectoryPoint &pt, int idx)
{
  eraseIdx(pt.positions, idx);
  eraseIdx(pt.velocities, idx);
  eraseIdx(pt.accelerations, idx);
  eraseIdx(pt.effort, idx);
}

bool JointRelayHandler::select(int group_id, FeedbackMsg &msg)
{
  std::list<int> to_remove;

  // find "blank" joint names
  for (int i=0; i<msg.joint_names.size(); ++i)
    if (msg.joint_names[i].empty())
      to_remove.push_back(i);

  // remove selected joints in each message component
  std::list<int>::const_iterator it;
  for (it=to_remove.begin(); it!=to_remove.end(); ++it)
  {
    int idx=*it;

    eraseIdx(msg.joint_names, idx);
    eraseIdx(msg.actual, idx);
    eraseIdx(msg.desired, idx);
    eraseIdx(msg.error, idx);
  }

  return true;
}

template<typename T>
void append(std::vector<T>&v1, const std::vector<T>&v2)
{
  v1.insert( v1.end(), v2.begin(), v2.end() );
}

void append(trajectory_msgs::JointTrajectoryPoint& pt1,
            const trajectory_msgs::JointTrajectoryPoint& pt2)
{
  append(pt1.positions,     pt2.positions);
  append(pt1.velocities,    pt2.velocities);
  append(pt1.accelerations, pt2.accelerations);
  append(pt1.effort,        pt2.effort);
}

bool JointRelayHandler::aggregate(const std::vector<FeedbackMsg>& msgs_in,
                                  std::map<Namespace, FeedbackMsg>& msgs_out)
{
  ROS_ASSERT(msgs_in.size() == this->joint_map_.size());

  // loop over input messages
  for (int i=0; i<msgs_in.size(); ++i)
  {
    const FeedbackMsg& msg_in = msgs_in[i];
    FeedbackMsg& msg_out = msgs_out[this->joint_map_[i].ns];
    
    // keep the latest timestamp
    msg_out.header.stamp = std::max(msg_in.header.stamp, msg_out.header.stamp);

    // append these joints into output message
    append(msg_out.joint_names, msg_in.joint_names);
    append(msg_out.actual,      msg_in.actual);
    append(msg_out.desired,     msg_in.desired);
    append(msg_out.error,       msg_in.error);
  }

  return true;
}

}//namespace joint_relay_handler
}//namespace industrial_robot_client2




