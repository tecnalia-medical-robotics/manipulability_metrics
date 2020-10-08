/*
 * Filename: similarity.cpp
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

#include <manipulability_metrics/util/similarity.h>

#include <Eigen/LU>

#include <numeric>

namespace manipulability_metrics
{
double volumeIntersection(const Ellipsoid& desired_ellipsoid, const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian)
{
  // Force evaluation by casting to avoid multiple (lazy) computations when iterating
  auto jjti = static_cast<Eigen::Matrix<double, 6, 6>>((jacobian * jacobian.transpose()).inverse());

  return std::accumulate(cbegin(desired_ellipsoid), cend(desired_ellipsoid), 1.0, [&](auto volume, const auto& ax) {
    double nu = 1.0 / sqrt(ax.unit.transpose() * jjti * ax.unit);
    return volume * std::min(nu, ax.len);
  });
}

double inverseShapeDiscrepancy(const Ellipsoid& desired_ellipsoid,
                               const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian)
{
  // Force evaluation by casting to avoid multiple (lazy) computations when iterating
  auto jjti = static_cast<Eigen::Matrix<double, 6, 6>>((jacobian * jacobian.transpose()).inverse());

  return 1.0 / std::accumulate(cbegin(desired_ellipsoid), cend(desired_ellipsoid),
                               std::numeric_limits<double>::epsilon(), [&](auto sq_diff, const auto& ax) {
                                 double nu = 1.0 / sqrt(ax.unit.transpose() * jjti * ax.unit);
                                 return sq_diff + pow(nu - ax.len, 2.0);
                               });
}

double dualVolumeIntersection(const Ellipsoid& desired_ellipsoid,
                              const Eigen::Matrix<double, 6, Eigen::Dynamic>& left_jacobian,
                              const Eigen::Matrix<double, 6, Eigen::Dynamic>& right_jacobian)
{
  const auto left_jjti =
      static_cast<Eigen::Matrix<double, 6, 6>>((left_jacobian * left_jacobian.transpose()).inverse());
  const auto right_jjti =
      static_cast<Eigen::Matrix<double, 6, 6>>((right_jacobian * right_jacobian.transpose()).inverse());

  double volume = 1.0;
  for (auto i = 0; i < 6; ++i)
  {
    double left_nu = 1.0 / sqrt(desired_ellipsoid[i].unit.transpose() * left_jjti * desired_ellipsoid[i].unit);
    double right_nu = 1.0 / sqrt(desired_ellipsoid[i].unit.transpose() * right_jjti * desired_ellipsoid[i].unit);
    volume *= std::min({ desired_ellipsoid[i].len, left_nu, right_nu });
  }

  return volume;
}

double dualInverseShapeDiscrepancy(const Ellipsoid& desired_ellipsoid,
                                   const Eigen::Matrix<double, 6, Eigen::Dynamic>& left_jacobian,
                                   const Eigen::Matrix<double, 6, Eigen::Dynamic>& right_jacobian)
{
  const auto left_jjti =
      static_cast<Eigen::Matrix<double, 6, 6>>((left_jacobian * left_jacobian.transpose()).inverse());
  const auto right_jjti =
      static_cast<Eigen::Matrix<double, 6, 6>>((right_jacobian * right_jacobian.transpose()).inverse());

  double sq_diff = 0.0;
  for (auto i = 0; i < 6; ++i)
  {
    double left_nu = 1.0 / sqrt(desired_ellipsoid[i].unit.transpose() * left_jjti * desired_ellipsoid[i].unit);
    double right_nu = 1.0 / sqrt(desired_ellipsoid[i].unit.transpose() * right_jjti * desired_ellipsoid[i].unit);
    sq_diff += pow(desired_ellipsoid[i].len - std::min(left_nu, right_nu), 2.0);
  }

  return 1.0 / (sq_diff + std::numeric_limits<double>::epsilon());
}
}  // namespace manipulability_metrics
