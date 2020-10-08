/*
 * Filename: minimum_singular_value.cpp
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

#include <manipulability_metrics/metrics/minimum_singular_value.h>

#include <Eigen/SVD>

namespace manipulability_metrics
{
double minimumSingularValue(const Chain& chain, const KDL::JntArray& joint_positions)
{
  return minimumSingularValue(chain, KDL::Vector::Zero(), joint_positions);
}

double minimumSingularValue(const Chain& chain, const KDL::Vector& tcp_offset, const KDL::JntArray& joint_positions)
{
  auto jac = KDL::Jacobian{ static_cast<unsigned int>(chain.n_joints) };
  chain.jacobian(joint_positions, tcp_offset, jac);

  auto jac_svd = Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>>{ jac.data };

  return jac_svd.singularValues()(jac_svd.singularValues().rows() - 1);
}
}  // namespace manipulability_metrics
