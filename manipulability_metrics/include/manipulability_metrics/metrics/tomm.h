/*
 * Filename: tomm.h
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

#ifndef MANIPULABILITY_METRICS_TOMM_H
#define MANIPULABILITY_METRICS_TOMM_H

#include <manipulability_metrics/kinematics/chain.h>
#include <manipulability_metrics/util/ellipsoid.h>

namespace manipulability_metrics
{
double tomm(const Chain& chain, const Ellipsoid& desired_ellipsoid, const KDL::JntArray& joint_positions);
double tomm(const Chain& chain, const KDL::Vector& tcp_offset, const Ellipsoid& desired_ellipsoid,
            const KDL::JntArray& joint_positions);
}  // namespace manipulability_metrics

#endif
