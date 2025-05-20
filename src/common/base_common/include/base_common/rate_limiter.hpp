// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef BASE_COMMON__RATE_LIMITER_HPP_
#define BASE_COMMON__RATE_LIMITER_HPP_

#include <math.h>
#include <cmath>
#include <iostream>

class RateLimiter
{
public:
  // constructors
  RateLimiter()
  : output(0.0),
    rate(0.0),
    default_dt(0.0) {}
  RateLimiter(const double & rate, const double & default_dt)
  : output(0.0),
    rate(rate),
    default_dt(default_dt) {}
  // functions
  double update(const double & input)
  {
    if (isnan(input)) {
      return output;
    }
    const double range = rate * default_dt;
    output = std::clamp(input, output - range, output + range);
    return output;
  }
  double update(const double & input, const double & dt)
  {
    if (isnan(input)) {
      return output;
    }
    const double range = rate * dt;
    output = std::clamp(input, output - range, output + range);
    return output;
  }

  double get_output() const {return output;}
  void set_output(const double & output) {this->output = output;}
  double get_rate() const {return rate;}
  void set_rate(const double & rate) {this->rate = rate;}

private:
  double output;
  double rate;
  double default_dt;
};

#endif  // BASE_COMMON__RATE_LIMITER_HPP_"
