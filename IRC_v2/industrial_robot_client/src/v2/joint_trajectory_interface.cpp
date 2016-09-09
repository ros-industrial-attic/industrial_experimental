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

#include "industrial_robot_client2/joint_trajectory_interface.h"

using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
namespace SpecialSeqValues = industrial::dynamic_joint_pt::SpecialSeqValues;
using industrial::dynamic_joint_pt::DynamicJointPtGrp;
using industrial::dynamic_joint_pt_message::DynamicJointPtMessage;
using industrial::shared_types::shared_real;
using industrial_msgs::CmdJointTrajectory;
using industrial_msgs::StopMotion;
using trajectory_msgs::JointTrajectory;

namespace industrial_robot_client2
{
namespace joint_trajectory_interface
{

bool JointTrajectoryInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Joint Trajectory Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection)
{
  std::vector<JointGroupMap> joint_map;
  if (!industrial_utils::param::getJointMap("", joint_map))
  {
    ROS_ERROR("Failed to initialize joint_map.  Aborting");
    return false;
  }

  return init(connection, joint_map);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection, const std::vector<JointGroupMap> &joint_map)
{
  this->connection_ = connection;
  this->joint_map_ = joint_map;
  connection_->makeConnect();

  // clear existing ROS-interface definitions
  this->handles_.clear();
  
  // create new subscribers/services for each specified namespace
  std::vector<JointGroupMap>::const_iterator mapIt;
  for (mapIt=joint_map.begin(); mapIt!=joint_map.end(); ++mapIt)
  {
    const std::string& ns = mapIt->ns;

    // create new subscriber/service handles, if none have already been defined for this namespace
    if (this->handles_.count(ns) == 0)
    {
      ros::NodeHandle nh(ns);  // create a nodeHandle for specified namespace
      RosHandles& hdl = this->handles_[ns];

      // the following boost::bind calls are used to add the matching namespace to the callback args
      //   - this allows using the same callback method for each namespace
      hdl.srv_stop_motion      = nh.advertiseService<StopMotion::Request, StopMotion::Response>(
                                         "stop_motion",
                                         boost::bind(&JointTrajectoryInterface::stopMotionCB, this, ns, _1, _2));
      hdl.srv_joint_trajectory = nh.advertiseService<CmdJointTrajectory::Request, CmdJointTrajectory::Response>(
                                         "joint_path_command",
                                         boost::bind(&JointTrajectoryInterface::svcCB_CmdJointTrajectory, this, ns, _1, _2));
      hdl.sub_joint_trajectory = nh.subscribe<JointTrajectory>(
                                         "joint_path_command", 0,
                                         boost::bind(&JointTrajectoryInterface::subCB_JointTrajectory, this, ns, _1));
    }
  }

  return true;
}

JointTrajectoryInterface::~JointTrajectoryInterface()
{  
  trajectoryStop();
}

bool JointTrajectoryInterface::svcCB_CmdJointTrajectory(const std::string &ns,
                                                        CmdJointTrajectory::Request &req,
                                                        CmdJointTrajectory::Response &res)
{
  bool success = this->jointTrajectoryCB(ns, req.trajectory);

  if (success)
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  else
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}
  
bool JointTrajectoryInterface::jointTrajectoryCB(const std::string &ns, const JointTrajectory &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // check for STOP command
  if (msg.points.empty())
  {
    ROS_INFO("Empty trajectory received, canceling current trajectory");
    trajectoryStop();
    return true;
  }

  // convert trajectory into robot-format
  std::vector<DynamicJointPtMessage> robot_msgs;
  if (!trajectory_to_msgs(ns, msg, robot_msgs))
    return false;

  // send command messages to robot
  send_to_robot(robot_msgs);
}

bool JointTrajectoryInterface::trajectory_to_msgs(const std::string &ns, const JointTrajectory &traj,
                                                  std::vector<DynamicJointPtMessage> &msgs)
{
  std::vector<JointGroupMap> active_map;
  msgs.clear();

  // check for valid trajectory
  if (!is_valid(traj))
    return false;
  
  // calculate "active" subset of joint_map for this trajectory
  if (!get_active_joint_maps(ns, traj, active_map))
    return false;

  // process each point in the trajectory
  for (size_t traj_idx=0; traj_idx<traj.points.size(); ++traj_idx)
  {
    std::vector<DynamicJointPtGrp> groups;

    // process each "active" group-map rule
    for (size_t map_idx=0; map_idx<active_map.size(); ++map_idx)
    {
      const JointGroupMap &map = active_map[map_idx];
      DynamicJointPtGrp group;

      // select / reorder joints for sending to robot
      if (!select(traj.joint_names, traj.points[traj_idx], map, group))
        return false;
      
      // skip this group if no joints specified in trajectory
      if (group.getNumJoints() == 0)
        continue;

      // transform point data (e.g. for joint-coupling)
      if (!transform(group, map))
        return false; 
      
      // add this group to active group-list for this trajectory point
      groups.push_back(group);
    }

    msgs.push_back(create_message(traj_idx, groups));
  }

  return true;
}

bool JointTrajectoryInterface::get_active_joint_maps(const std::string &ns,
                                                     const trajectory_msgs::JointTrajectory &traj,
                                                     std::vector<JointGroupMap> &active_map)
{
  active_map.clear();

  // convert trajectory joint-names to set, for faster lookup
  std::set<std::string> traj_joints(traj.joint_names.begin(), traj.joint_names.end());
  
  // check each group-map rule defined in joint_map
  for (size_t map_idx=0; map_idx<joint_map_.size(); ++map_idx)
  {
    const JointGroupMap &map = joint_map_[map_idx];

    // skip this group if namespace doesn't match
    if (map.ns != ns)
      continue;
    
    // check for any matching joint-name
    bool found=false;
    for (size_t jnt_idx=0; !found && jnt_idx<map.joints.size(); jnt_idx++)
      found = (traj_joints.count(map.joints[jnt_idx]) > 0);
    
    if (found)
      active_map.push_back(map);  // save this JointGroupMap
  }
  
  return !active_map.empty();
}

bool JointTrajectoryInterface::select(const std::vector<std::string> &traj_names,
                                      const trajectory_msgs::JointTrajectoryPoint &traj_pt,
                                      const JointGroupMap &map,
                                      DynamicJointPtGrp &group)
{
  // pre-allocate vectors to correct size, with default values (used when joint_map name is empty-string)
  size_t numJnts = map.joints.size();
  std::vector<shared_real> positions(numJnts, default_joint_pos_), velocities(numJnts, 0.0);
  std::vector<shared_real> accelerations(numJnts, 0.0), effort(numJnts, 0.0);

  // loop over each joint in the group joint-map
  for (size_t grp_idx=0; grp_idx < map.joints.size(); ++grp_idx)
  {
    // find matching trajectory joint-name
    size_t traj_idx = std::find(traj_names.begin(), traj_names.end(), map.joints[grp_idx]) - traj_names.begin();
    bool is_found = (traj_idx < traj_names.size()) && !map.joints[grp_idx].empty();

    // error if required joint not found in trajectory
    if (!is_found)
    {
      ROS_ERROR_STREAM("Expected joint (" << map.joints[grp_idx] << ") not found in JointTrajectory.  Aborting command.");
      return false;
    }
    
    // copy joint-values to local arrays, in controller joint-order
    if (!traj_pt.positions.empty())     positions[grp_idx]     = traj_pt.positions[traj_idx];
    if (!traj_pt.velocities.empty())    velocities[grp_idx]    = traj_pt.velocities[traj_idx];
    if (!traj_pt.accelerations.empty()) accelerations[grp_idx] = traj_pt.accelerations[traj_idx];
    if (!traj_pt.effort.empty())        effort[grp_idx]        = traj_pt.effort[traj_idx];
  }
  
  // create output simple_message data structure
  group.init(map.joints.size());
  group.setGroupID(map.group_id);
  group.setTime(traj_pt.time_from_start.toSec());
  if (!traj_pt.positions.empty())     group.setPositions(positions);
  if (!traj_pt.velocities.empty())    group.setVelocities(velocities);
  if (!traj_pt.accelerations.empty()) group.setAccelerations(accelerations);
  if (!traj_pt.effort.empty())        group.setEfforts(effort);
  
  return true;
}

DynamicJointPtMessage JointTrajectoryInterface::create_message(int seq, std::vector<DynamicJointPtGrp> groups)
{
  industrial::dynamic_joint_pt_message::DynamicJointPtMessage msg;
  industrial::dynamic_joint_pt::DynamicJointPt &pt = msg.point_;
  
  pt.setSequence(seq);
  for (size_t i=0; i<groups.size(); ++i)
    pt.addGroup(groups[i]);

  return msg;
}

bool JointTrajectoryInterface::trajectoryStop()
{
  DynamicJointPtMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory handler: entering stopping state");
  jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending stop command");
  return this->connection_->sendAndReceiveMsg(msg, reply);
}

bool JointTrajectoryInterface::stopMotionCB(const std::string &ns,
                                            industrial_msgs::StopMotion::Request &req,
                                            industrial_msgs::StopMotion::Response &res)
{
  bool result = trajectoryStop();

  if (result)
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  else
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

bool JointTrajectoryInterface::is_valid(const trajectory_msgs::JointTrajectory &traj)
{
  for (size_t i=0; i<traj.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[i];

    // require at least one valid data-field
    if (pt.positions.empty() && pt.velocities.empty() && pt.accelerations.empty() && pt.effort.empty())
    {
      ROS_ERROR("Validation failed: no data specified for trajectory pt %d", i);
      return false;
    }

    // check for correct # of joints
    if (  (!pt.positions.empty() && traj.joint_names.size() != pt.positions.size() )
       || (!pt.velocities.empty() && traj.joint_names.size() != pt.velocities.size() )
       || (!pt.accelerations.empty() && traj.joint_names.size() != pt.accelerations.size() )
       || (!pt.effort.empty() && traj.joint_names.size() != pt.effort.size() ) )
    {
      ROS_ERROR("Validation failed: incorrect # of joints for trajectory pt %d", i);
      return false;
    }

    // check for valid timestamp
    if ((i > 0) && (pt.time_from_start.toSec() == 0))
    {
      ROS_ERROR("Validation failed: Missing valid timestamp data for trajectory pt %d", i);
      return false;
    }
  }

  return true;
}

} //joint_trajectory_interface
} //industrial_robot_client2

