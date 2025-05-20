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

/// \copyright Copyright 2022 Siddharth Saha
/// \file
/// \brief This file defines the polynomials class (quartic and quintic).

#ifndef RACE_PATH_PLANNER__POLYNOMIALS_HPP_
#define RACE_PATH_PLANNER__POLYNOMIALS_HPP_

#include <Eigen/Core>

namespace race
{
namespace race_path_planner
{
struct QuinticPolynomialParams
{
  double x_start;
  double v_start;
  double a_start;
  double x_end;
  double v_end;
  double a_end;
  double t;
};

class QuarticPolynomial
{
public:
  explicit QuarticPolynomial(QuinticPolynomialParams & params);
  double calc_point(double t);
  double calc_first_derivative(double t);
  double calc_second_derivative(double t);
  double calc_third_derivative(double t);

private:
  void calc_x(QuinticPolynomialParams & params, Eigen::Vector2d & x);

  double a0;
  double a1;
  double a2;
  double a3;
  double a4;
};

class QuinticPolynomial
{
public:
  explicit QuinticPolynomial(QuinticPolynomialParams & params);
  double calc_point(double t);
  double calc_first_derivative(double t);
  double calc_second_derivative(double t);
  double calc_third_derivative(double t);

private:
  void calc_x(QuinticPolynomialParams & params, Eigen::Vector3d & x);

  double a0;
  double a1;
  double a2;
  double a3;
  double a4;
  double a5;
};
}  // namespace race_path_planner
}  // namespace race

#endif  // RACE_PATH_PLANNER__POLYNOMIALS_HPP_
