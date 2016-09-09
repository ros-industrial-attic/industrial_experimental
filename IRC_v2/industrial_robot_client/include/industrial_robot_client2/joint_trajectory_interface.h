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

#ifndef JOINT_TRAJECTORY_INTERFACE_H
#define JOINT_TRAJECTORY_INTERFACE_H

#include <map>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "industrial_utils/param_utils.h"
#include "industrial_msgs/CmdJointTrajectory.h"
#include "industrial_msgs/StopMotion.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/messages/dynamic_joint_pt_message.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace industrial_robot_client2
{
namespace joint_trajectory_interface
{
  using industrial::smpl_msg_connection::SmplMsgConnection;
  using industrial::tcp_client::TcpClient;
  using industrial::dynamic_joint_pt::DynamicJointPtGrp;
  using industrial::dynamic_joint_pt_message::DynamicJointPtMessage;
  namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
  using industrial_utils::param::JointGroupMap;

/**
 * \brief Message handler that relays joint trajectories to the robot controller
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryInterface
{

public:
  typedef std::string Namespace;  // ROS namespace
  
 /**
  * \brief Default constructor.
  */
    JointTrajectoryInterface() : default_joint_pos_(0.0) {};

    /**
     * \brief Initialize robot connection using default method.
     *
     * \param default_ip default IP address to use for robot connection [OPTIONAL]
     *                    - this value will be used if ROS param "robot_ip_address" cannot be read
     * \param default_port default port to use for robot connection [OPTIONAL]
     *                    - this value will be used if ROS param "~port" cannot be read
     *
     * \return true on success, false otherwise
     */
    virtual bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION);


    /**
     * \brief Initialize robot connection using specified method.
     *
     * \param connection new robot-connection instance (ALREADY INITIALIZED).
     *
     * \return true on success, false otherwise
     */
    virtual bool init(SmplMsgConnection* connection);

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_map map of controller group/joint order to namespaces and joint-names for msg-publishing.
   *   - Multiple mapping rules per group are allowed
   *   - Within a group, count and order should match data from robot connection.
   *   - Use blank-name to to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<JointGroupMap>& joint_map);

  virtual ~JointTrajectoryInterface();

  /**
   * \brief Begin processing messages and publishing topics.
   */
  virtual void run() { ros::spin(); }

protected:

  /**
   * \brief Send a stop command to the robot
   */
  virtual bool trajectoryStop();

  /**
   * \brief Convert ROS trajectory message into stream of DynamicJointPtMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] ns  Namespace associated with this trajectory
   * \param[in] traj ROS JointTrajectory message
   * \param[out] msgs list of DynamicJointPtMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool trajectory_to_msgs(const std::string &ns, const trajectory_msgs::JointTrajectory &traj,
                                  std::vector<DynamicJointPtMessage> &msgs);

  /**
   * \brief Check for "active" joint_map rules corresponding to the given trajectory
   *
   * \param[in] ns  Namespace associated with this trajectory
   * \param[in] traj ROS JointTrajectory message
   * \param[out] active_map list of JointGroupMap rules that match this trajectory
   *
   * \return true on success, false otherwise
   */
  virtual bool get_active_joint_maps(const std::string &ns, const trajectory_msgs::JointTrajectory &traj,
                                    std::vector<JointGroupMap> &active_map);

  /**
   * \brief Select specific joints for sending to the robot
   *
   * \param[in] traj_names joint names from ROS command
   * \param[in] traj_pt single trajectory point from ROS command
   * \param[in] map map of joint-name ordering for the currently-selected group
   * \param[out] group trajectory-point data for the current group, ordered to match the joint_map
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<std::string> &traj_names,
                      const trajectory_msgs::JointTrajectoryPoint &traj_pt,
                      const JointGroupMap &map,
                      DynamicJointPtGrp &group);

  /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in,out] pt trajectory-point, ordered to match the joint_map
   * \param[in] map map of joint-name ordering for the currently-selected group
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(DynamicJointPtGrp &pt, const JointGroupMap &map)
  {
    // by default, no transform is applied
    return true;
  }
  
  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param messages List of SimpleMessage JointTrajPtMessages to send to robot.
   *
   * \return true on success, false otherwise
   */
  virtual bool send_to_robot(const std::vector<DynamicJointPtMessage>& messages)=0;

  /**
   * \brief Callback function to process a new joint trajectory
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param ns  Namespace associated with callback
   * \param msg JointTrajectory message from ROS trajectory-planner
   */
  virtual bool jointTrajectoryCB(const std::string &ns,
                                 const trajectory_msgs::JointTrajectory &msg);

  /**
   * \brief Validate that trajectory command meets minimum requirements
   *
   * \param traj incoming trajectory
   * \return true if trajectory is valid, false otherwise
   */
  virtual bool is_valid(const trajectory_msgs::JointTrajectory &traj);

  /**
   * \brief Pack multiple motion groups into a single message
   *
   * \param seq sequence number for this trajectory point
   * \param groups trajectory data for each motion group
   * \return trajectory point message to send to robot
   */
  virtual DynamicJointPtMessage create_message(int seq, std::vector<DynamicJointPtGrp> groups);

  /**
   * \brief ROS-interface handles for a single namespace
   */
  struct RosHandles
  {
    ros::Subscriber    sub_joint_trajectory;  // handle for joint-trajectory topic subscription
    ros::ServiceServer srv_joint_trajectory;  // handle for joint-trajectory service
    ros::ServiceServer srv_stop_motion;       // handle for stop_motion service
  };
  
  TcpClient default_tcp_connection_;
  ros::NodeHandle node_;
  SmplMsgConnection* connection_;
  std::map<Namespace, RosHandles> handles_;  // handles for ROS sub/svc interfaces
  std::vector<JointGroupMap> joint_map_;  // map of ROS namespaces/joints to controller motion groups
  double default_joint_pos_;  // default position to use for "dummy joints", if none specified
  sensor_msgs::JointState cur_joint_pos_;  // cache of last received joint state

private:

  /**
   * \brief Callback function registered to ROS CmdJointTrajectory service
   *   Duplicates message-topic functionality, but in service form.
   *
   * \param ns  Namespace associated with service call
   * \param req CmdJointTrajectory request from service call
   * \param res CmdJointTrajectory response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded
   */
  bool svcCB_CmdJointTrajectory(const std::string &ns,
                                industrial_msgs::CmdJointTrajectory::Request &req,
                                industrial_msgs::CmdJointTrajectory::Response &res);

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param ns  Namespace associated with subscription callback
   * \param msg JointTrajectory message from ROS trajectory-planner
   */
  void subCB_JointTrajectory(const std::string &ns,
                             const trajectory_msgs::JointTrajectoryConstPtr &msg)
  {
    jointTrajectoryCB(ns, *msg);
  }

  /**
   * \brief Callback function registered to ROS stopMotion service
   *   Sends stop-motion command to robot.
   *
   * \param ns  Namespace associated with service call
   * \param req StopMotion request from service call
   * \param res StopMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool stopMotionCB(const std::string &ns,
                            industrial_msgs::StopMotion::Request &req,
                            industrial_msgs::StopMotion::Response &res);

};

} //joint_trajectory_interface
} //industrial_robot_client2

#endif /* JOINT_TRAJECTORY_INTERFACE_H */
