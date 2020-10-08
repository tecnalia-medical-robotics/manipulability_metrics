/*
 * Filename: markers.cpp
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

#include <manipulability_metrics/util/markers.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>

namespace manipulability_metrics
{
visualization_msgs::Marker ellipsoidMarker(const Eigen::Matrix<double, 3, Eigen::Dynamic>& half_jacobian,
                                           const Eigen::Isometry3d& transform, const std::string& root,
                                           const std::string& ns, const std_msgs::ColorRGBA& color)
{
  auto svd = Eigen::JacobiSVD<Eigen::Matrix<double, 3, Eigen::Dynamic>>{ half_jacobian, Eigen::ComputeFullU };
  auto base = svd.matrixU();
  auto len = static_cast<Eigen::Vector3d>(svd.singularValues().topRows<3>());
  if (base.determinant() < 0)
  {
    base.col(1).swap(base.col(2));
    len.row(1).swap(len.row(2));
  }

  auto ellipsoid_marker = visualization_msgs::Marker{};
  ellipsoid_marker.type = visualization_msgs::Marker::SPHERE;
  ellipsoid_marker.header.frame_id = root;
  ellipsoid_marker.ns = ns;
  ellipsoid_marker.color = color;
  tf::pointEigenToMsg(transform.translation(), ellipsoid_marker.pose.position);
  tf::quaternionEigenToMsg(static_cast<Eigen::Quaterniond>(base), ellipsoid_marker.pose.orientation);
  tf::vectorEigenToMsg(len, ellipsoid_marker.scale);

  return ellipsoid_marker;
}

visualization_msgs::MarkerArray ellipsoidMarkers(const Chain& chain, const std::vector<double>& joint_positions,
                                                 const std::string& ns_prefix)
{
  return ellipsoidMarkers(chain, KDL::Frame{}, joint_positions, ns_prefix);
}

visualization_msgs::MarkerArray ellipsoidMarkers(const Chain& chain, const KDL::JntArray& joint_positions,
                                                 const std::string& ns_prefix)
{
  return ellipsoidMarkers(chain, KDL::Frame{}, joint_positions, ns_prefix);
}

visualization_msgs::MarkerArray ellipsoidMarkers(const Chain& chain, const KDL::Frame& tcp,
                                                 const std::vector<double>& joint_positions,
                                                 const std::string& ns_prefix)
{
  auto jnt_array = KDL::JntArray{ static_cast<unsigned int>(joint_positions.size()) };
  jnt_array.data =
      Eigen::Map<const Eigen::VectorXd>{ joint_positions.data(), static_cast<Eigen::Index>(joint_positions.size()) };
  return ellipsoidMarkers(chain, tcp, jnt_array, ns_prefix);
}

visualization_msgs::MarkerArray ellipsoidMarkers(const Chain& chain, const KDL::Frame& tcp,
                                                 const KDL::JntArray& joint_positions, const std::string& ns_prefix)
{
  auto frame = KDL::Frame{};
  auto jacobian = KDL::Jacobian{ static_cast<unsigned int>(chain.n_joints) };
  if (!chain.transform(joint_positions, tcp, frame) || !chain.jacobian(joint_positions, tcp.p, jacobian))
  {
    return {};
  }

  const auto transform = [&] {
    auto transform = Eigen::Isometry3d{};
    tf::transformKDLToEigen(frame, transform);
    return transform;
  }();

  auto marker_array = visualization_msgs::MarkerArray{};
  marker_array.markers.reserve(2);
  {
    std_msgs::ColorRGBA transparent_green;
    transparent_green.g = 1.0;
    transparent_green.a = 0.5;
    marker_array.markers.push_back(ellipsoidMarker(jacobian.data.topRows<3>(), transform, chain.root,
                                                   ns_prefix + "translational_ellipsoid", transparent_green));
  }
  {
    std_msgs::ColorRGBA transparent_red;
    transparent_red.r = 1.0;
    transparent_red.a = 0.5;
    marker_array.markers.push_back(ellipsoidMarker(jacobian.data.bottomRows<3>(), transform, chain.root,
                                                   ns_prefix + "rotational_ellipsoid", transparent_red));
  }

  return marker_array;
}

visualization_msgs::MarkerArray desiredEllipsoidMarkers(const Ellipsoid& desired_ellipsoid, const std::string& frame_id,
                                                        const KDL::Frame& transform)
{
  auto pseudoU = Eigen::Matrix<double, 6, 6>{};
  auto pseudoS = Eigen::Matrix<double, 6, 6>{ Eigen::Matrix<double, 6, 6>::Zero() };
  for (std::size_t i = 0; i < 6; ++i)
  {
    pseudoU.col(i) = desired_ellipsoid[i].unit;
    pseudoS(i, i) = desired_ellipsoid[i].len;
  }
  const auto pseudojac = pseudoU * pseudoS;

  const auto eigen_transform = [&] {
    auto eigen_transform = Eigen::Isometry3d{};
    tf::transformKDLToEigen(transform, eigen_transform);
    return eigen_transform;
  }();

  auto markers = visualization_msgs::MarkerArray{};
  markers.markers.reserve(2);
  {
    std_msgs::ColorRGBA transparent_light_green;
    transparent_light_green.r = 0.2;
    transparent_light_green.b = 0.2;
    transparent_light_green.g = 1.0;
    transparent_light_green.a = 0.5;

    markers.markers.push_back(ellipsoidMarker(pseudojac.topRows<3>(), eigen_transform, frame_id,
                                              "desired_translational_ellipsoid", transparent_light_green));
  }
  {
    std_msgs::ColorRGBA transparent_light_red;
    transparent_light_red.r = 1.0;
    transparent_light_red.b = 0.2;
    transparent_light_red.g = 0.2;
    transparent_light_red.a = 0.5;

    markers.markers.push_back(ellipsoidMarker(pseudojac.bottomRows<3>(), eigen_transform, frame_id,
                                              "desired_rotational_ellipsoid", transparent_light_red));
  }
  return markers;
}

}  // namespace manipulability_metrics
