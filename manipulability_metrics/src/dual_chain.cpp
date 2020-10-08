/*
 * Filename: dual_chain.cpp
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

namespace manipulability_metrics
{
DualTcp::DualTcp(const DualChain& dual_chain, TcpBase base, const KDL::Frame& transform,
                 const KDL::JntArray& left_joint_positions, const KDL::JntArray& right_joint_positions)
{
  auto left_transform = KDL::Frame{};
  dual_chain.leftChain().transform(left_joint_positions, left_transform);
  auto right_transform = KDL::Frame{};
  dual_chain.rightChain().transform(right_joint_positions, right_transform);

  switch (base)
  {
    case TcpBase::Base:
      left_tcp_ = left_transform.Inverse() * transform;
      right_tcp_ = right_transform.Inverse() * transform;
      break;
    case TcpBase::Left:
      left_tcp_ = transform;
      right_tcp_ = right_transform.Inverse() * left_transform * transform;
      break;
    case TcpBase::Right:
      left_tcp_ = left_transform.Inverse() * right_transform * transform;
      right_tcp_ = transform;
      break;
    default:
      throw std::invalid_argument("Incorrect specification for dual-arm TCP");
  }
}
}  // namespace manipulability_metrics
