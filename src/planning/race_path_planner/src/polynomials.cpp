// Copyright 2022 Siddharth Saha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "race_path_planner/polynomials.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

namespace race
{
namespace race_path_planner
{

QuarticPolynomial::QuarticPolynomial(QuinticPolynomialParams & params)
{
  a0 = params.x_start;
  a1 = params.v_start;
  a2 = params.a_start / 2;

  Eigen::Vector2d x;
  calc_x(params, x);

  a3 = x(0);
  a4 = x(1);
}

void QuarticPolynomial::calc_x(QuinticPolynomialParams & params, Eigen::Vector2d & x)
{
  Eigen::Matrix2d A;
  Eigen::Vector2d b;

  A << 3.0 * std::pow(params.t, 2.0),
    4.0 * std::pow(params.t, 3.0),
    6.0 * params.t,
    12.0 * std::pow(params.t, 2.0);

  b << params.v_end - a1 - (2 * a2 * params.t),
    params.a_end - (2 * a2);

  x = A.completeOrthogonalDecomposition().solve(b);
}

double QuarticPolynomial::calc_point(double t)
{
  return a0 + (a1 * t) + (a2 * std::pow(t, 2.0)) +
         (a3 * std::pow(t, 3.0)) + (a4 * std::pow(t, 4.0));
}

double QuarticPolynomial::calc_first_derivative(double t)
{
  return a1 + (2.0 * a2 * t) +
         (3.0 * a3 * std::pow(t, 2.0)) +
         (4.0 * a4 * std::pow(t, 3.0));
}

double QuarticPolynomial::calc_second_derivative(double t)
{
  return (2.0 * a2) + (6.0 * a3 * t) +
         (12.0 * a4 * std::pow(t, 2.0));
}

double QuarticPolynomial::calc_third_derivative(double t)
{
  return (6.0 * a3) + (24.0 * a4 * t);
}

QuinticPolynomial::QuinticPolynomial(QuinticPolynomialParams & params)
{
  a0 = params.x_start;
  a1 = params.v_start;
  a2 = params.a_start / 2;

  Eigen::Vector3d x;
  calc_x(params, x);

  a3 = x(0);
  a4 = x(1);
  a5 = x(2);
}

void QuinticPolynomial::calc_x(QuinticPolynomialParams & params, Eigen::Vector3d & x)
{
  Eigen::Matrix3d A;
  Eigen::Vector3d b;

  A << std::pow(params.t, 3.0),
    std::pow(params.t, 4.0),
    std::pow(params.t, 5.0),
    3.0 * std::pow(params.t, 2.0),
    4.0 * std::pow(params.t, 3.0),
    5.0 * std::pow(params.t, 4.0),
    6.0 * params.t,
    12.0 * std::pow(params.t, 2.0),
    20.0 * std::pow(params.t, 3.0);

  b << params.x_end - a0 - (a1 * params.t) -
    (a2 * std::pow(params.t, 2.0)),
    params.v_end - a1 - (2 * a2 * params.t),
    params.a_end - (2 * a2);

  x = A.completeOrthogonalDecomposition().solve(b);
}

double QuinticPolynomial::calc_point(double t)
{
  return a0 + (a1 * t) + (a2 * std::pow(t, 2.0)) +
         (a3 * std::pow(t, 3.0)) + (a4 * std::pow(t, 4.0)) +
         (a5 * std::pow(t, 5.0));
}

double QuinticPolynomial::calc_first_derivative(double t)
{
  return a1 + (2.0 * a2 * t) +
         (3.0 * a3 * std::pow(t, 2.0)) +
         (4.0 * a4 * std::pow(t, 3.0)) +
         (5.0 * a5 * std::pow(t, 4.0));
}

double QuinticPolynomial::calc_second_derivative(double t)
{
  return (2.0 * a2) + (6.0 * a3 * t) +
         (12.0 * a4 * std::pow(t, 2.0)) +
         (20.0 * a5 * std::pow(t, 3.0));
}

double QuinticPolynomial::calc_third_derivative(double t)
{
  return (6.0 * a3) + (24.0 * a4 * t) +
         (60.0 * a5 * std::pow(t, 2.0));
}
}  // namespace race_path_planner
}  // namespace race
