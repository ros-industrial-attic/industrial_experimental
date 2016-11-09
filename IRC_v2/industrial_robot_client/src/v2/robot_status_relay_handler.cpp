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

#include "industrial_robot_client2/robot_status_relay_handler.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;
using industrial::smpl_msg_connection::SmplMsgConnection;
using namespace industrial::simple_message;
using industrial::dynamic_group_status::DynamicGroupStatus;
using industrial::dynamic_group_status_message::DynamicGroupStatusMessage;
using namespace industrial::robot_status;
using industrial_utils::param::JointGroupMap;

namespace industrial_robot_client2
{
namespace robot_status_relay_handler
{

bool RobotStatusRelayHandler::init(SmplMsgConnection* connection,
                                   const std::vector<JointGroupMap>& joint_map)
{
  // save joint map for later use
  this->joint_map_ = joint_map;

  // clear existing publisher definitions
  this->pub_robot_status_.clear();

  // create new publisher for each specified namespace
  std::vector<JointGroupMap>::const_iterator map;
  for (map=joint_map.begin(); map!=joint_map.end(); ++map)
  {
    const std::string& ns = map->ns;
    ros::NodeHandle nh(ns);  // create a nodeHandle for specified namespace

    // create new publisher, if not already been defined for this namespace
    if (this->pub_robot_status_.count(ns) == 0)
      this->pub_robot_status_[ns] = nh.advertise<StatusMsg>("robot_status", 1);
  }

  return init((int)StandardMsgTypes::DYNAMIC_GROUP_STATUS, connection);
}

bool RobotStatusRelayHandler::internalCB(SimpleMessage& in)
{
  SimpleMsg status_msg;

  // create specific typed-message from generic input message
  if (!status_msg.init(in))
  {
    LOG_ERROR("Failed to initialize status message");
    return false;
  }

  return internalCB(status_msg);
}

bool RobotStatusRelayHandler::internalCB(SimpleMsg& in)
{
  std::map<Namespace, StatusMsg> status_msgs;

  // check incoming SimpleMessage for validity
  bool rtn = check_message(in);

  // Convert incoming SimpleMessage to outgoing ROS messages
  if (rtn)
    rtn = create_messages(in, status_msgs);

  // Publish outgoing ROS messages
  std::map<Namespace, StatusMsg>::const_iterator mapIt;
  for (mapIt=status_msgs.begin(); rtn && mapIt!=status_msgs.end(); ++mapIt)
  {
    const Namespace& ns         = mapIt->first;
    const StatusMsg& status_msg = mapIt->second;
    this->pub_robot_status_[ns].publish(status_msg);
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


bool RobotStatusRelayHandler::check_message(const SimpleMsg& msg)
{
  bool rtn = check_duplicate_groups(msg)
             && check_joint_map_match(msg);

  if (!rtn)
    LOG_ERROR("Failed to parse incoming DynamicGroupState message");
  return rtn;
}


bool RobotStatusRelayHandler::check_duplicate_groups(const SimpleMsg& msg)
{
  std::set<shared_int> ids;
  const DynamicGroupStatus& status = msg.status_;

  // loop over each group in message
  for (size_t i=0; i<status.getNumGroups(); ++i)
  {
    shared_int id = status.getGroup(i).getGroupID();

    // check for duplicate group IDs
    if (ids.count(id) > 0)
    {
      LOG_ERROR("Duplicate IDs detected in DynamicGroupStatus message");
      return false;
    }
    else
      ids.insert(id);
  }

  return true;
}

bool RobotStatusRelayHandler::check_joint_map_match(const SimpleMsg& msg)
{
  const DynamicGroupStatus& status = msg.status_;

  // loop over each group in message
  for (size_t i=0; i<status.getNumGroups(); ++i)
  {
    bool found=false;
    shared_int id = status.getGroup(i).getGroupID();

    // check for any matching joint_map entry
    for (size_t j=0; !found && j<this->joint_map_.size(); ++j)
      found = (id == this->joint_map_[j].group_id);

    // log error if no matching group defined in joint_map
    if (!found)
    {
      LOG_ERROR("No matching joint_map found for group #%d", id);
      return false;
    }

  }

  return true;
}

bool RobotStatusRelayHandler::create_messages(const SimpleMsg& msg_in,
                                              std::map<Namespace, StatusMsg>& msg_out)
{
  // copy raw message data into ROS msg structure
  //   - use one msg per group ID
  std::map<int, StatusMsg> working_msgs;
  if (!copy_data(msg_in, working_msgs))
  {
    LOG_ERROR("Failed to copy data to ROS message structure");
    return false;
  }

  // aggregate per-group messages into per-namespace messages
  if (!aggregate(working_msgs, msg_out))
  {
    LOG_ERROR("Failed to aggregate multi-group messages for publishing");
    return false;
  }

  return true;
}

bool RobotStatusRelayHandler::copy_data(const SimpleMsg& in,
                                        std::map<int, StatusMsg>& out)
{
  // reset output msgs
  out.clear();

  // ensure all messages use the same header/timestamp
  std_msgs::Header header;
  header.stamp = ros::Time::now();

  // process each group in the incoming message
  for (size_t msg_idx=0; msg_idx<in.status_.getNumGroups(); ++msg_idx)
  {
    const SimpleMsgGrpData& in_grp = in.status_.getGroup(msg_idx);
    int group_id = in_grp.getGroupID();

    copy_data(in_grp, out[group_id]);
    out[group_id].header = header;
  }

  return true;
}

bool RobotStatusRelayHandler::copy_data(const SimpleMsgGrpData& in, StatusMsg& out)
{
  out.drives_powered.val  = TriStates::toROSMsgEnum(in.getDrivesPowered());
  out.e_stopped.val       = TriStates::toROSMsgEnum(in.getEStopped());
  out.error_code          = in.getErrorCode();
  out.in_error.val        = TriStates::toROSMsgEnum(in.getInError());
  out.in_motion.val       = TriStates::toROSMsgEnum(in.getInMotion());
  out.mode.val            = RobotModes::toROSMsgEnum(in.getMode());
  out.motion_possible.val = TriStates::toROSMsgEnum(in.getMotionPossible());

  return true;
}

bool RobotStatusRelayHandler::aggregate(const std::map<int, StatusMsg>&in,
                                        std::map<Namespace, StatusMsg>&out)
{
  // loop over input messages
  std::map<int, StatusMsg>::const_iterator msgIt;
  for (msgIt=in.begin(); msgIt!=in.end(); ++msgIt)
  {
    int group_id            = msgIt->first;
    const StatusMsg& in_msg = msgIt->second;

    // find matching joint-map rules (by group-id)
    for (int map_idx=0; map_idx<this->joint_map_.size(); ++map_idx)
    {
      const std::string& ns = this->joint_map_[map_idx].ns;

      // if group_id doesn't match, check next rule
      if (this->joint_map_[map_idx].group_id != group_id)
        continue;

      // aggregate with existing messages
      if (out.count(ns)==0)
        out[ns] = in_msg;
      else
        out[ns] = aggregate(in_msg, out[ns]);
    }
  }

  return true;
}

industrial_msgs::RobotMode allOrUnknown(const industrial_msgs::RobotMode& mode1,
                                        const industrial_msgs::RobotMode& mode2)
{
  industrial_msgs::RobotMode out;
  out.val = (mode1.val == mode2.val) ? mode1.val : industrial_msgs::RobotMode::UNKNOWN;
  return out;
}

industrial_msgs::TriState allOrUnknown(const industrial_msgs::TriState& state1,
                                       const industrial_msgs::TriState& state2)
{
  industrial_msgs::TriState out;
  out.val = (state1.val == state2.val) ? state1.val : industrial_msgs::TriState::UNKNOWN;
  return out;
}

RobotStatusRelayHandler::StatusMsg RobotStatusRelayHandler::aggregate(const StatusMsg& msg1, const StatusMsg& msg2)
{
  StatusMsg out;

  // keep the latest timestamp
  ros::Time maxTime  = std::max(msg1.header.stamp, msg2.header.stamp);

  out.header          = msg1.header;
  out.header.stamp    = maxTime;
  out.mode            = allOrUnknown(msg1.mode, msg2.mode);
  out.e_stopped       = allOrUnknown(msg1.e_stopped, msg2.e_stopped);
  out.drives_powered  = allOrUnknown(msg1.drives_powered, msg2.drives_powered);
  out.motion_possible = allOrUnknown(msg1.motion_possible, msg2.motion_possible);
  out.in_motion       = allOrUnknown(msg1.in_motion, msg2.in_motion);
  out.in_error        = allOrUnknown(msg1.in_error, msg2.in_error);
  out.error_code      = (msg1.error_code == 0) ? msg2.error_code : msg1.error_code;

  return out;
}

}
}

