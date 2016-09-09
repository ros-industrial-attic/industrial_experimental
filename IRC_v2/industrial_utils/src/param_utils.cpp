/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#include <sstream>

#include "industrial_utils/param_utils.h"
#include "industrial_utils/utils.h"
#include "ros/ros.h"
#include "urdf/model.h"

namespace industrial_utils
{
namespace param
{
bool getListParam(const std::string param_name, std::vector<std::string> & list_param)
{
  XmlRpc::XmlRpcValue rpc_list;

  list_param.clear(); //clear out return value

  if (!ros::param::get(param_name, rpc_list))
  {
    ROS_ERROR_STREAM("Failed to get parameter: " << param_name);
    return false;
  }

  if (!getListParam(rpc_list, list_param))
  {
    ROS_ERROR_STREAM("Failed to parse parameter: " << param_name);
    return false;
  }

  return true;
}

bool getListParam(const std::string param_name, std::vector<JointGroupMap> & list_param)
{
  XmlRpc::XmlRpcValue rpc_list;

  list_param.clear(); //clear out return value

  if (!ros::param::get(param_name, rpc_list))
  {
    ROS_ERROR_STREAM("Failed to get parameter: " << param_name);
    return false;
  }

  if (rpc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Parameter: " << param_name << " not of list type");
    return false;
  }

  for (int i = 0; i < rpc_list.size(); ++i)
  {
    JointGroupMap map;
    if (!map.parse(rpc_list[i]))
    {
      ROS_ERROR_STREAM("Failed to parse parameter: " << param_name
                       << "[" << i << "]");
      return false;
    }

    list_param.push_back(map);
  }

  return true;
}

bool getListParam(XmlRpc::XmlRpcValue rpc_list,
                  std::vector<std::string> & list_param)
{
  list_param.clear();

  if (rpc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("parameter not of list type");
    return false;
  }

  for (int i = 0; i < rpc_list.size(); ++i)
  {
    if (rpc_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("List item " << i << " not of string type");
      return false;
    }

    ROS_DEBUG_STREAM("Adding " << rpc_list[i] << " to list parameter");
    list_param.push_back(static_cast<std::string>(rpc_list[i]));
  }

  return true;
}

std::string vec2str(const std::vector<std::string> &vec)
{
  std::string s, delim = ", ";
  std::stringstream ss;
  std::copy(vec.begin(), vec.end(), std::ostream_iterator<std::string>(ss, delim.c_str()));
  s = ss.str();
  return "[" + s.erase(s.length()-2) + "]";
}

bool getJointNames(const std::string joint_list_param_, const std::string urdf_param_,
		           std::vector<std::string> & joint_names)
{
  std::string joint_list_param(joint_list_param_);
  std::string urdf_param(urdf_param_);

  joint_names.clear();

  // use default parameter-names, if none specified
  if (joint_list_param.empty()) joint_list_param = "controller_joint_names";
  if (urdf_param.empty()) urdf_param = "robot_description";

  // 1) Try to read explicit list of joint names
  if (ros::param::has(joint_list_param) && getListParam(joint_list_param, joint_names))
  {
    ROS_INFO_STREAM("Found user-specified joint names in '" << joint_list_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    ROS_WARN_STREAM("Unable to find user-specified joint names in '" << joint_list_param << "'");

  // 2) Try to find joint names from URDF model
  urdf::Model model;
  if ( ros::param::has(urdf_param)
       && model.initParam(urdf_param)
       && findChainJointNames(model.getRoot(), true, joint_names) )
  {
    ROS_INFO_STREAM("Using joint names from URDF: '" << urdf_param << "': " << vec2str(joint_names));
    return true;
  }
  else
    ROS_WARN_STREAM("Unable to find URDF joint names in '" << urdf_param << "'");

  // 3) Use default joint-names
  const int NUM_JOINTS = 6;  //Most robots have 6 joints
  for (int i=0; i<NUM_JOINTS; ++i)
  {
    std::stringstream tmp;
    tmp << "joint_" << i+1;
    joint_names.push_back(tmp.str());
  }

  ROS_INFO_STREAM("Using standard 6-DOF joint names: " << vec2str(joint_names));
  return true;
}

bool getJointVelocityLimits(const std::string urdf_param_name, std::map<std::string, double> &velocity_limits)
{
  urdf::Model model;
  std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator iter;

  if (!ros::param::has(urdf_param_name) || !model.initParam(urdf_param_name))
    return false;
    
  velocity_limits.clear();
  for (iter=model.joints_.begin(); iter!=model.joints_.end(); ++iter)
  {
    std::string joint_name(iter->first);
    boost::shared_ptr<urdf::JointLimits> limits = iter->second->limits;
    if ( limits && (limits->velocity > 0) )
      velocity_limits.insert(std::pair<std::string,double>(joint_name,limits->velocity));
  }
  
  return true;
}

bool getJointMap(const std::string joint_map_param_,
                 std::vector<JointGroupMap> & joint_map)
{
  std::string joint_map_param(joint_map_param_);

  joint_map.clear();

  // use default parameter-name, if none specified
  if (joint_map_param.empty()) joint_map_param = "controller_joint_map";

  // 1) Try to read explicit joint map
  if (ros::param::has(joint_map_param) && getListParam(joint_map_param, joint_map))
  {
    ROS_INFO_STREAM("Found user-specified joint map in '" << joint_map_param << "':");
    for (int i=0; i<joint_map.size(); ++i)
      ROS_INFO_STREAM("  " << joint_map[i].toString());
    return true;
  }

  ROS_WARN_STREAM("Unable to find user-specified joint names in '"
                  << joint_map_param
                  << "'.  Assuming single-group (id:1) in this namespace.");

  std::vector<std::string> joint_names;
  if (!getJointNames("", "", joint_names))
  {
    ROS_ERROR_STREAM("Failed to get any joint names.  ROS->robot mapping will fail.");
    return false;
  }

  joint_map.push_back( JointGroupMap(1, "", joint_names) );
  return true;
}

bool JointGroupMap::parse(XmlRpc::XmlRpcValue value)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("JointGroupMap not struct type");
    return false;
  }

  if (!value.hasMember("group")
   || (value["group"].getType() != XmlRpc::XmlRpcValue::TypeInt) )
  {
    ROS_ERROR("JointGroupMap 'group' field missing or invalid type");
    return false;
  }
  this->group_id = static_cast<int>(value["group"]);

  if (!value.hasMember("ns")
   || (value["ns"].getType() != XmlRpc::XmlRpcValue::TypeString) )
  {
    ROS_ERROR("JointGroupMap 'ns' field missing or invalid type");
    return false;
  }
  this->ns = static_cast<std::string>(value["ns"]);

  if (!value.hasMember("joints") || !getListParam(value["joints"], this->joints) )
  {
    ROS_ERROR("JointGroupMap 'joints' field missing or invalid type");
    return false;
  }

  return true;
}

std::string JointGroupMap::toString()
{
  std::stringstream ss;
  ss << "Grp " << this->group_id << " ('" << this->ns << "'): " << vec2str(this->joints);

  return ss.str();
}

} //industrial_utils::param
} //industrial_utils
