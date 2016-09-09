/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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


#ifndef JOINT_RELAY_HANDLER_H
#define JOINT_RELAY_HANDLER_H

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "industrial_utils/param_utils.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/message_handler.h"
#include "simple_message/messages/dynamic_joint_state_message.h"

namespace industrial_robot_client2
{
namespace joint_relay_handler
{

/**
 * \brief Message handler that relays dynamic (multi-group) joint states
 * (converts simple message types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

  typedef std::string Namespace;
  typedef industrial::dynamic_joint_state_message::DynamicJointStateMessage SimpleMsg;
  typedef industrial::dynamic_joint_state::DynamicJointStateGrp SimpleMsgGrpData;
  typedef control_msgs::FollowJointTrajectoryFeedback FeedbackMsg;
  typedef sensor_msgs::JointState StateMsg;

public:

  /**
* \brief Constructor
*/
  JointRelayHandler() {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  * \param joint_map map of controller group/joint order to namespaces and
  * joint-names for msg-publishing.
  *   - Multiple mapping rules per group are allowed
  *   - Within a group, count and order should match data from robot connection.
  *   - Use blank-name to exclude a joint from publishing.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 virtual bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
                   const std::vector<industrial_utils::param::JointGroupMap>& joint_map);

protected:

  std::vector<industrial_utils::param::JointGroupMap> joint_map_;
  std::map<Namespace, ros::Publisher> pub_control_state_;
  std::map<Namespace, ros::Publisher> pub_sensor_state_;
  ros::NodeHandle node_;  // not used directly, but keeps Node alive

  /**
   * \brief Check incoming simple_message for validity
   *
   * \param[in] msg DynamicJointState message from robot
   *
   * \return true on success, false otherwise
   */
  virtual bool check_message(const SimpleMsg& msg);

  /**
   * \brief Check incoming simple_message for duplicate group-IDs
   *
   * \param[in] msg DynamicJointState message from robot
   *
   * \return true on success, false otherwise
   */
  virtual bool check_duplicate_groups(const SimpleMsg& msg);

  /**
   * \brief Check incoming simple_message against joint_map
   *    - Group ID is known/defined
   *    - # of joints matches
   *
   * \param[in] msg DynamicJointState message from robot
   *
   * \return true on success, false otherwise
   */
  virtual bool check_joint_map_match(const SimpleMsg& msg);

  /**
   * \brief Convert incoming simple_message into ROS message types, for publishing
   *
   * \param[in] msg_in Dynamic Joint State message from robot connection
   * \param[out] control_msgs FollowJointTrajectoryFeedback messages
   * \param[out] sensor_msgs JointState messages
   *
   * \return true on success, false otherwise
   */
  virtual bool create_messages(const SimpleMsg& msg_in,
                               std::map<Namespace, FeedbackMsg>& control_msgs,
                               std::map<Namespace, StateMsg>& sensor_msgs);

  /**
   * \brief Convert incoming simple_message into ROS messages.
   * - this version only creates the control-feedback messages, to minimize duplication
   *
   * \param[in] msg_in Dynamic Joint State message from robot connection
   * \param[out] control_msgs FollowJointTrajectoryFeedback messages
   *
   * \return true on success, false otherwise
   */
  virtual bool create_messages(const SimpleMsg& msg_in,
                               std::map<Namespace, FeedbackMsg>& control_msgs);

  /**
   * \brief Clone message data from SimpleMsg -> ROSMsg
   *  - this method only copies the raw data; no additional conversions are performed
   *
   * \param[in] in SimpleMsg data
   * \param[out] out joint data for ROS (unprocessed/filtered)
   *    - one entry created for each entry in active joint_map
   *
   * \return true on success, false otherwise
   */
  bool copy_data(const SimpleMsg& in, std::vector<FeedbackMsg>& out);

  /**
   * \brief Transform joint state before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] group_id group ID
   * \param[in,out] msg joint state data, before/after transformation
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(int group_id, FeedbackMsg& msg)
  {
    return true;  // by default, no transform is applied
  }

  /**
   * \brief Select specific joints for publishing
   *
   * \param[in] group_id group ID
   * \param[in,out] msg joint state data, before/after selection
   *
   * \return true on success, false otherwise
   */
  virtual bool select(int group_id, FeedbackMsg& msg);

  /**
   * \brief Aggregate groups mapped to same namespace into one message
   *
   * \param[in]  msgs_in  joint state data, listed by mapping rule
   * \param[out] msgs_out joint state data, grouped by namespace
   *
   * \return true on success, false otherwise
   */
  virtual bool aggregate(const std::vector<FeedbackMsg>& msgs_in,
                         std::map<Namespace, FeedbackMsg>& msgs_out);

  /**
   * \brief Callback executed upon receiving a dynamic joint state message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(SimpleMsg& in);

private:
  /**
   * \brief Callback executed upon receiving a message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(industrial::simple_message::SimpleMessage& in);

  /**
   * \brief Clone message data for a single group from SimpleMsg -> ROSMsg
   *  - this method only copies the raw data; no additional conversions are performed
   *
   * \param[in] joint_names joint names for this motion group
   * \param[in] in single-group state from Robot
   * \param[out] out single-group state for ROS (unprocessed/filtered)
   *
   * \return true on success, false otherwise
   */
  bool copy_data(const std::vector<std::string>& joint_names,
                 const SimpleMsgGrpData& in, FeedbackMsg& out);

};//class JointRelayHandler

}//joint_relay_handler
}//industrial_robot_client2


#endif /* JOINT_RELAY_HANDLER_H */
