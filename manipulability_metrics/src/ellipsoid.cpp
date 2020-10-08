/*
 * Filename: ellipsoid.cpp
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

#include <manipulability_metrics/util/ellipsoid.h>

#include <Eigen/SVD>

namespace manipulability_metrics
{
Ellipsoid ellipsoidFromJacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian)
{
  auto jac_svd = Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>>{ jacobian, Eigen::ComputeFullU };

  return { { { jac_svd.matrixU().col(0), jac_svd.singularValues()(0) },
             { jac_svd.matrixU().col(1), jac_svd.singularValues()(1) },
             { jac_svd.matrixU().col(2), jac_svd.singularValues()(2) },
             { jac_svd.matrixU().col(3), jac_svd.singularValues()(3) },
             { jac_svd.matrixU().col(4), jac_svd.singularValues()(4) },
             { jac_svd.matrixU().col(5), jac_svd.singularValues()(5) } } };
}
}  // namespace manipulability_metrics
