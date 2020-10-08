/*
 * Filename: dual_panda_manipulability_demo.cpp
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

#include <manipulability_metrics/kinematics/dual_chain.h>
#include <manipulability_metrics/metrics/damm.h>
#include <manipulability_metrics/metrics/todamm.h>
#include <manipulability_metrics/util/ellipsoid.h>
#include <manipulability_metrics/util/markers.h>

#include "demo_utils.h"

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_panda_manipulability_demo");

  auto nh = ros::NodeHandle{};
  auto p_nh = ros::NodeHandle{ "~" };

  const auto model = utils::getModel(p_nh);
  const auto root_link = utils::getParam<std::string>(p_nh, "root_link");
  const auto left_tip_link = utils::getParam<std::string>(p_nh, "left_tip_link");
  const auto right_tip_link = utils::getParam<std::string>(p_nh, "right_tip_link");
  const auto left_chain_joints = utils::getParam<std::vector<std::string>>(p_nh, "left_chain_joints");
  const auto right_chain_joints = utils::getParam<std::vector<std::string>>(p_nh, "right_chain_joints");
  const auto tcp_frame = std::string{ "tcp_frame" };

  const auto dual_chain = manipulability_metrics::DualChain{ model, root_link, left_tip_link, right_tip_link };

  const auto fixed_tcp_frame = utils::getTransform(root_link, tcp_frame);

  const auto desired_manipulability =
      manipulability_metrics::Ellipsoid{ { { (Eigen::Matrix<double, 6, 1>{} << 1, 0, 0, 0, 0, 0).finished(), 0.2 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 1, 0, 0, 0, 0).finished(), 0.8 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 1, 0, 0, 0).finished(), 0.8 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 1, 0, 0).finished(), 0.9 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 1, 0).finished(), 1.7 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 0, 1).finished(), 1.7 } } };

  const auto marker_pub = nh.advertise<visualization_msgs::MarkerArray>("manipulability_markers", 1);

  const auto left_joint_permutation = utils::permutationFromJointState(left_chain_joints);
  const auto right_joint_permutation = utils::permutationFromJointState(right_chain_joints);

  const auto joint_state_cb = utils::makeCallback<sensor_msgs::JointState>([&](const auto& msg) {
    const auto left_jnt_array = utils::extractJoints(msg.position, left_joint_permutation);
    const auto right_jnt_array = utils::extractJoints(msg.position, right_joint_permutation);

    const auto dual_tcp = manipulability_metrics::DualTcp{ dual_chain, manipulability_metrics::TcpBase::Base,
                                                           fixed_tcp_frame, left_jnt_array, right_jnt_array };

    const auto damm = manipulability_metrics::damm(dual_chain, dual_tcp, left_jnt_array, right_jnt_array);
    const auto todamm =
        manipulability_metrics::todamm(dual_chain, dual_tcp, desired_manipulability, left_jnt_array, right_jnt_array);

    ROS_INFO_STREAM("Got an updated joint state. Computed metrics:\nDAMM: " << damm << "\nTODAMM: " << todamm);

    const auto left_chain_manipulability_markers =
        manipulability_metrics::ellipsoidMarkers(dual_chain.leftChain(), dual_tcp.leftTcp(), left_jnt_array, "left_");
    const auto right_chain_manipulability_markers = manipulability_metrics::ellipsoidMarkers(
        dual_chain.rightChain(), dual_tcp.rightTcp(), right_jnt_array, "right_");

    marker_pub.publish(left_chain_manipulability_markers);
    marker_pub.publish(right_chain_manipulability_markers);

    const auto desired_manipulability_markers =
        manipulability_metrics::desiredEllipsoidMarkers(desired_manipulability, root_link, fixed_tcp_frame);
    marker_pub.publish(desired_manipulability_markers);
  });

  const auto joint_state_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, joint_state_cb);

  ros::spin();

  return 0;
}
