/*
 * Filename: dual_chain.h
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

#ifndef MANIPULABILITY_METRICS_DUAL_CHAIN_H
#define MANIPULABILITY_METRICS_DUAL_CHAIN_H

#include <manipulability_metrics/kinematics/chain.h>

namespace manipulability_metrics
{
class DualChain
{
public:
  DualChain(const urdf::ModelInterface& model, const std::string& root, const std::string& left_tip,
            const std::string& right_tip)
    : left_chain_(model, root, left_tip), right_chain_(model, root, right_tip)
  {
  }

  const Chain& leftChain() const
  {
    return left_chain_;
  }

  const Chain& rightChain() const
  {
    return right_chain_;
  }

private:
  const Chain left_chain_;
  const Chain right_chain_;
};

enum class TcpBase
{
  Left,
  Right,
  Base
};

class DualTcp
{
public:
  DualTcp(const DualChain& dual_chain, TcpBase base, const KDL::Frame& transform,
          const KDL::JntArray& left_joint_positions, const KDL::JntArray& right_joint_positions);

  const KDL::Frame& leftTcp() const
  {
    return left_tcp_;
  }

  const KDL::Frame& rightTcp() const
  {
    return right_tcp_;
  }

private:
  KDL::Frame left_tcp_;
  KDL::Frame right_tcp_;
};
}  // namespace manipulability_metrics
#endif
