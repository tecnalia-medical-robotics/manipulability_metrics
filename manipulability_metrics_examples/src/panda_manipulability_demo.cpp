/*
 * Filename: panda_manipulability_demo.cpp
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

#include <manipulability_metrics/kinematics/chain.h>
#include <manipulability_metrics/metrics/inverse_condition_number.h>
#include <manipulability_metrics/metrics/manipulability_measure.h>
#include <manipulability_metrics/metrics/minimum_singular_value.h>
#include <manipulability_metrics/metrics/tomm.h>
#include <manipulability_metrics/util/ellipsoid.h>
#include <manipulability_metrics/util/markers.h>

#include "demo_utils.h"

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_manipulability_demo");

  auto nh = ros::NodeHandle{};
  auto p_nh = ros::NodeHandle{ "~" };

  const auto model = utils::getModel(p_nh);
  const auto root_link = utils::getParam<std::string>(p_nh, "root_link");
  const auto tip_link = utils::getParam<std::string>(p_nh, "tip_link");
  const auto chain_joints = utils::getParam<std::vector<std::string>>(p_nh, "chain_joints");

  const auto chain = manipulability_metrics::Chain{ model, root_link, tip_link };

  const auto desired_manipulability =
      manipulability_metrics::Ellipsoid{ { { (Eigen::Matrix<double, 6, 1>{} << 1, 0, 0, 0, 0, 0).finished(), 0.2 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 1, 0, 0, 0, 0).finished(), 0.8 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 1, 0, 0, 0).finished(), 0.8 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 1, 0, 0).finished(), 0.9 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 1, 0).finished(), 1.7 },
                                           { (Eigen::Matrix<double, 6, 1>{} << 0, 0, 0, 0, 0, 1).finished(), 1.7 } } };

  const auto marker_pub = nh.advertise<visualization_msgs::MarkerArray>("manipulability_markers", 1);

  const auto joint_permutation = utils::permutationFromJointState(chain_joints);

  const auto joint_state_cb = utils::makeCallback<sensor_msgs::JointState>([&](const auto& msg) {
    const auto jnt_array = utils::extractJoints(msg.position, joint_permutation);

    const auto icn = manipulability_metrics::inverseConditionNumber(chain, jnt_array);
    const auto mm = manipulability_metrics::manipulabilityMeasure(chain, jnt_array);
    const auto msv = manipulability_metrics::minimumSingularValue(chain, jnt_array);
    const auto tomm = manipulability_metrics::tomm(chain, desired_manipulability, jnt_array);

    ROS_INFO_STREAM("Got an updated joint state. Computed metrics:\nInverse Condition Number: "
                    << icn << "\nManipulability Measure: " << mm << "\nMinimum Singular Value: " << msv
                    << "\nTOMM: " << tomm);

    const auto manipulability_markers = manipulability_metrics::ellipsoidMarkers(chain, jnt_array);
    marker_pub.publish(manipulability_markers);

    auto transform = KDL::Frame{};
    chain.transform(jnt_array, transform);

    const auto desired_manipulability_markers =
        manipulability_metrics::desiredEllipsoidMarkers(desired_manipulability, chain.root, transform);
    marker_pub.publish(desired_manipulability_markers);
  });

  const auto joint_state_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, joint_state_cb);

  ros::spin();

  return 0;
}
