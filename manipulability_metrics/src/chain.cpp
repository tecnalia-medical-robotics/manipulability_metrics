/*
 * Filename: chain.cpp
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

#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl_parser/kdl_parser.hpp>

namespace manipulability_metrics
{
KDL::Chain parseChain(const urdf::ModelInterface& model, const std::string& root, const std::string& tip)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    throw std::runtime_error("Failed to parse model");
  }

  KDL::Chain chain;
  if (!tree.getChain(root, tip, chain))
  {
    throw std::runtime_error("Failed to find chain in tree");
  }

  return chain;
}

std::vector<std::string> chainJointNames(const KDL::Chain& chain)
{
  std::vector<std::string> joint_names;
  for (auto& segment : chain.segments)
  {
    if (segment.getJoint().getType() != KDL::Joint::None)
    {
      joint_names.push_back(segment.getJoint().getName());
    }
  }
  return joint_names;
}

Chain::Chain(const urdf::ModelInterface& model, const std::string& root, const std::string& tip)
  : chain_(parseChain(model, root, tip))
  , jac_solver_(chain_)
  , n_joints(chain_.getNrOfJoints())
  , joint_names(chainJointNames(chain_))
  , root(root)
  , tip(tip)
{
}

bool Chain::transform(const KDL::JntArray& joint_positions, KDL::Frame& transform) const
{
  KDL::ChainFkSolverPos_recursive fksolver(chain_);
  return fksolver.JntToCart(joint_positions, transform) == KDL::SolverI::E_NOERROR;
}

bool Chain::transform(const KDL::JntArray& joint_positions, const KDL::Frame& tcp, KDL::Frame& transform) const
{
  if (!Chain::transform(joint_positions, transform))
  {
    return false;
  }
  transform = transform * tcp;
  return true;
}

bool Chain::jacobian(const KDL::JntArray& joint_positions, KDL::Jacobian& jacobian) const
{
  return jac_solver_.JntToJac(joint_positions, jacobian) == KDL::SolverI::E_NOERROR;
}

bool Chain::jacobian(const KDL::JntArray& joint_positions, const KDL::Vector& tcp_offset, KDL::Jacobian& jacobian) const
{
  if (!Chain::jacobian(joint_positions, jacobian))
  {
    return false;
  }

  auto transform = KDL::Frame{};
  if (!Chain::transform(joint_positions, transform))
  {
    return false;
  }

  jacobian.changeRefPoint(transform.M * tcp_offset);
  return true;
}
}  // namespace manipulability_metrics
