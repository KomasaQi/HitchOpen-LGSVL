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

#ifndef BASE_COMMON__CYCLE_PROFILER_HPP_
#define BASE_COMMON__CYCLE_PROFILER_HPP_

#include <chrono>

#include <boost/circular_buffer.hpp>

template<typename T>
struct Profile
{
  T mean;
  T max;
  T min;
};

template<typename T>
class CycleProfiler
{
  typedef T Duration;
  typedef boost::circular_buffer<Duration> DurationBuffer;
  typedef std::chrono::system_clock::time_point TimePoint;

public:
  CycleProfiler()
  {
  }

  explicit CycleProfiler(const size_t & window)
  : durations_(window)
  {
  }

  void set_window(const size_t & window)
  {
    durations_.resize(window);
  }

  void start()
  {
    last_start_time_ = now();
  }

  void end()
  {
    const Duration duration = std::chrono::duration_cast<Duration>(now() - last_start_time_);
    durations_.push_back(duration);
  }

  size_t capacity()
  {
    return durations_.capacity();
  }

  Profile<Duration> profile()
  {
    Profile<Duration> result{Duration(0), Duration(0), Duration(0)};
    auto size = durations_.size();
    if (size == 0) {
      return result;
    } else {
      result.max = durations_[0];
      result.min = durations_[0];
    }
    for (const auto & duration : durations_) {
      if (duration > result.max) {
        result.max = duration;
      }
      if (duration < result.min) {
        result.min = duration;
      }
      result.mean += duration;
    }
    result.mean /= size;
    return result;
  }

protected:
  DurationBuffer durations_;
  TimePoint last_start_time_;

  TimePoint now()
  {
    return std::chrono::system_clock::now();
  }
};

#endif  // BASE_COMMON__CYCLE_PROFILER_HPP_
