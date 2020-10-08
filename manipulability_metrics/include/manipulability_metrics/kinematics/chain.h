/*
 * Filename: chain.h
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

#ifndef MANIPULABILITY_METRICS_CHAIN_H
#define MANIPULABILITY_METRICS_CHAIN_H

#include <urdf_model/model.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace manipulability_metrics
{
class Chain
{
public:
  Chain(const urdf::ModelInterface& model, const std::string& root, const std::string& tip);

  bool transform(const KDL::JntArray& joint_positions, KDL::Frame& transform) const;

  bool transform(const KDL::JntArray& joint_positions, const KDL::Frame& tcp, KDL::Frame& transform) const;

  bool jacobian(const KDL::JntArray& joint_positions, KDL::Jacobian& jacobian) const;

  bool jacobian(const KDL::JntArray& joint_positions, const KDL::Vector& tcp_offset, KDL::Jacobian& jacobian) const;

private:
  KDL::Chain chain_;

  mutable KDL::ChainJntToJacSolver jac_solver_;

public:
  const std::size_t n_joints;

  const std::vector<std::string> joint_names;

  const std::string root;

  const std::string tip;
};
}  // namespace manipulability_metrics

#endif
