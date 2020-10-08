/*
 * Filename: demo_utils.h
 *
 * Copyright 2020 Tecnalia
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MANIPULABILITY_METRICS_EXAMPLES_DEMO_UTILS_H
#define MANIPULABILITY_METRICS_EXAMPLES_DEMO_UTILS_H

#include <urdf/model.h>

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>

#include <kdl/jntarray.hpp>

namespace utils
{
template <typename Msg, typename Fun>
auto makeCallback(Fun&& fun)
{
  return static_cast<boost::function<void(const Msg&)>>(fun);
}

inline auto getModel(const ros::NodeHandle& nh)
{
  auto model = urdf::Model{};
  if (!model.initParamWithNodeHandle("robot_description", nh))
  {
    throw std::runtime_error("Failed to retrieve and initialize URDF model");
  }
  return model;
}

template <typename ValueType>
ValueType getParam(const ros::NodeHandle& nh, const std::string& name)
{
  auto value = ValueType{};
  if (!nh.getParam(name, value))
  {
    throw std::runtime_error("Failed to retrieve param: " + name);
  }
  return value;
}

inline KDL::Frame getTransform(const std::string& parent, const std::string& child)
{
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener{ buffer };

  geometry_msgs::TransformStamped transform_stamped =
      buffer.lookupTransform(parent, child, ros::Time(0), ros::Duration(1.0));

  auto transform_frame = KDL::Frame{};
  tf::transformMsgToKDL(transform_stamped.transform, transform_frame);

  return transform_frame;
}

inline std::vector<std::size_t> jointPermutation(const std::vector<std::string>& find_joints,
                                                 const std::vector<std::string>& from_joints)
{
  auto permutation = std::vector<std::size_t>{};
  permutation.reserve(find_joints.size());
  std::transform(find_joints.cbegin(), find_joints.cend(), std::back_inserter(permutation),
                 [&from_joints](const auto& joint) {
                   auto it = std::find(from_joints.cbegin(), from_joints.cend(), joint);
                   if (it == from_joints.cend())
                   {
                     throw std::runtime_error("Failed to find joint in joint states");
                   }
                   return std::distance(from_joints.cbegin(), it);
                 });
  return permutation;
}

inline std::vector<std::size_t> permutationFromJointState(const std::vector<std::string>& joints)
{
  const auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", ros::Duration(1.0));
  if (!msg)
  {
    throw std::runtime_error("Failed to receive joint states");
  }
  return jointPermutation(joints, msg->name);
}

inline KDL::JntArray extractJoints(const std::vector<double>& positions, const std::vector<std::size_t>& permutation)
{
  auto jnt_array = KDL::JntArray{ static_cast<unsigned int>(permutation.size()) };
  for (std::size_t i = 0; i < permutation.size(); ++i)
  {
    jnt_array(i) = positions[permutation[i]];
  }
  return jnt_array;
}
}  // namespace utils

#endif
