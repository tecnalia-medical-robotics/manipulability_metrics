/*
 * Filename: damm.h
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

#ifndef MANIPULABILITY_METRICS_DAMM_H
#define MANIPULABILITY_METRICS_DAMM_H

#include <manipulability_metrics/kinematics/dual_chain.h>

namespace manipulability_metrics
{
double damm(const DualChain& dual_chain, const DualTcp& tcp, const KDL::JntArray& left_joint_positions,
            const KDL::JntArray& right_joint_positions);
}  // namespace manipulability_metrics

#endif
