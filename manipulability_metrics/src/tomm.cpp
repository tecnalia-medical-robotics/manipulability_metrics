/*
 * Filename: tomm.cpp
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

#include <manipulability_metrics/metrics/tomm.h>
#include <manipulability_metrics/util/similarity.h>

namespace manipulability_metrics
{
double tomm(const Chain& chain, const Ellipsoid& desired_ellipsoid, const KDL::JntArray& joint_positions)
{
  return tomm(chain, KDL::Vector{}, desired_ellipsoid, joint_positions);
}

double tomm(const Chain& chain, const KDL::Vector& tcp_offset, const Ellipsoid& desired_ellipsoid,
            const KDL::JntArray& joint_positions)
{
  auto jac = KDL::Jacobian{ static_cast<unsigned int>(chain.n_joints) };
  chain.jacobian(joint_positions, tcp_offset, jac);
  return inverseShapeDiscrepancy(desired_ellipsoid, jac.data);
}
}  // namespace manipulability_metrics
