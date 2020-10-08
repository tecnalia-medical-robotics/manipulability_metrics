/*
 * Filename: damm.cpp
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

#include <manipulability_metrics/metrics/damm.h>
#include <manipulability_metrics/util/ellipsoid.h>
#include <manipulability_metrics/util/similarity.h>

#include <Eigen/SVD>

#include <numeric>

namespace manipulability_metrics
{
double damm(const DualChain& dual_chain, const DualTcp& tcp, const KDL::JntArray& left_joint_positions,
            const KDL::JntArray& right_joint_positions)
{
  auto left_jac = KDL::Jacobian{ static_cast<unsigned int>(dual_chain.leftChain().n_joints) };
  dual_chain.leftChain().jacobian(left_joint_positions, tcp.leftTcp().p, left_jac);
  auto right_jac = KDL::Jacobian{ static_cast<unsigned int>(dual_chain.rightChain().n_joints) };
  dual_chain.rightChain().jacobian(right_joint_positions, tcp.rightTcp().p, right_jac);

  auto left_ellipsoid = ellipsoidFromJacobian(left_jac.data);
  auto right_ellipsoid = ellipsoidFromJacobian(right_jac.data);

  return std::max(volumeIntersection(left_ellipsoid, right_jac.data),
                  volumeIntersection(right_ellipsoid, left_jac.data));
}
}  // namespace manipulability_metrics
