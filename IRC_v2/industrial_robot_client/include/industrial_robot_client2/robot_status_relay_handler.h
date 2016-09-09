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


#ifndef ROBOT_STATUS_RELAY_HANDLER_H
#define ROBOT_STATUS_RELAY_HANDLER_H

#include "ros/ros.h"
#include "simple_message/message_handler.h"
#include "simple_message/messages/dynamic_group_status_message.h"
#include "industrial_msgs/RobotStatus.h"
#include "industrial_utils/param_utils.h"

namespace industrial_robot_client2
{
namespace robot_status_relay_handler
{

/**
 * \brief Message handler that relays robot status info (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class RobotStatusRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;
  typedef std::string Namespace;
  typedef industrial::dynamic_group_status_message::DynamicGroupStatusMessage SimpleMsg;
  typedef industrial::dynamic_group_status::DynamicGroupStatusGrp SimpleMsgGrpData;
  typedef industrial_msgs::RobotStatus StatusMsg;

public:

  /**
* \brief Constructor
*/
  RobotStatusRelayHandler() {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  * \param joint_map map of controller group/joint order to namespaces and
  * joint-names for msg-publishing.
  *   - Multiple mapping rules per group are allowed
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,
           const std::vector<industrial_utils::param::JointGroupMap>& joint_map);

protected:

  ros::NodeHandle node_;
  std::vector<industrial_utils::param::JointGroupMap> joint_map_;
  std::map<Namespace, ros::Publisher> pub_robot_status_;

  /**
   * \brief Check incoming simple_message for validity
   *
   * \param[in] msg DynamicGroupStatus message from robot
   *
   * \return true on success, false otherwise
   */
  virtual bool check_message(const SimpleMsg& msg);

  /**
   * \brief Check incoming simple_message for duplicate group-IDs
   *
   * \param[in] msg DynamicGroupStatus message from robot
   *
   * \return true on success, false otherwise
   */
  virtual bool check_duplicate_groups(const SimpleMsg& msg);

  /**
   * \brief Check incoming simple_message against joint_map
   *    - Group ID is known/defined
   *
   * \param[in] msg DynamicGroupStatus message from robot
   *
   * \return true on success, false otherwise
   */
  virtual bool check_joint_map_match(const SimpleMsg& msg);

  /**
   * \brief Convert incoming simple_message into ROS message types, for publishing
   *
   * \param[in] msg_in Dynamic Group Status message from robot connection
   * \param[out] msg_out RobotStatus messages for ROS
   *
   * \return true on success, false otherwise
   */
  virtual bool create_messages(const SimpleMsg& msg_in, std::map<Namespace, StatusMsg>& msg_out);

  /**
   * \brief Clone message data from SimpleMsg -> ROSMsg
   *  - this method only copies the raw data; no additional conversions are performed
   *
   * \param[in] in SimpleMsg data
   * \param[out] out status data for ROS (unprocessed/filtered)
   *    - one entry created for each group-id
   *
   * \return true on success, false otherwise
   */
  bool copy_data(const SimpleMsg& in, std::map<int, StatusMsg>& out);

  /**
   * \brief Clone message data from SimpleMsg -> ROSMsg
   *  - this method only copies the raw data; no additional conversions are performed
   *  - this method works on a single motion-group's data
   *
   * \param[in] in SimpleMsg data
   * \param[out] out status data for ROS (unprocessed/filtered)
   *
   * \return true on success, false otherwise
   */
  bool copy_data(const SimpleMsgGrpData& in, StatusMsg& out);

  /**
   * \brief Aggregate groups mapped to same namespace into one message
   *
   * \param[in]  in  robot status msgs, for each group-id
   * \param[out] out robot status msgs, grouped by namespace
   *
   * \return true on success, false otherwise
   */
  virtual bool aggregate(const std::map<int, StatusMsg>&in, std::map<Namespace, StatusMsg>&out);

  /**
   * \brief Aggregate two RobotStatus messages into a single message
   *   - override this method to change multi-group aggregation logic
   *
   * \param[in]  msg1 robot status msg, for one group
   * \param[out] msg2 robot status msg, for another group
   *
   * \return combined message
   */
  RobotStatusRelayHandler::StatusMsg aggregate(const StatusMsg& msg1, const StatusMsg& msg2);

  /**
   * \brief Callback executed upon receiving a robot status message
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
};

}
}


#endif /* ROBOT_STATUS_RELAY_HANDLER_H */
