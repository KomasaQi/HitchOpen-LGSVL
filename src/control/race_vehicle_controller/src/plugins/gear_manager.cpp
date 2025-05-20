// Copyright 2024 AI Racing Tech
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

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "race_msgs/msg/vehicle_control_command.hpp"
#include "base_common/pubsub.hpp"

#include "race_vehicle_controller/plugin.hpp"

namespace race
{

class Gear
{
public:
  explicit Gear(const int8_t & index, const double & min_rpm, const double & max_rpm)
  : m_index_(index), m_min_rpm_(min_rpm), m_max_rpm_(max_rpm)
  {}

  explicit Gear(const Gear & gear)
  : m_index_(gear.m_index_), m_min_rpm_(gear.m_min_rpm_), m_max_rpm_(gear.m_max_rpm_)
  {}

  const int8_t & get_gear_number() const
  {
    return m_index_;
  }

  const double & get_min_rpm() const
  {
    return m_min_rpm_;
  }

  const double & get_max_rpm() const
  {
    return m_max_rpm_;
  }

private:
  int8_t m_index_;
  double m_min_rpm_;
  double m_max_rpm_;
};

class Gearbox
{
public:
  Gearbox() {}
  Gearbox(std::vector<Gear> gears, int8_t initial_gear_number)
  : m_gears_(gears)
  {
    if (gears.size() == 0) {
      throw std::logic_error("Number of gears must be greater than zero.");
    }

    if (!set_gear(initial_gear_number)) {
      throw std::runtime_error("Gearbox initialization failed.");
    }
  }

  /**
   * @brief Check if the RPM is below minimum, and check if the current gear is not the lowest gear
   *
   * @param rpm current engine PRM
   * @return if both conditions are true
   */
  bool should_downshift(const double & rpm) const
  {
    return rpm < m_current_gear_->get_min_rpm() && !is_lowest_gear();
  }

  /**
   * @brief Check if the RPM is above maximum, and check if the current gear is not the top gear
   *
   * @param rpm current engine PRM
   * @return if both conditions are true
   */
  bool should_upshift(const double & rpm) const
  {
    return rpm > m_current_gear_->get_max_rpm() && !is_top_gear();
  }

  /**
   * @brief Check if the current gear is the lowest gear
   *
   * @return if the current gear is the lowest gear
   */
  bool is_lowest_gear() const
  {
    return std::find_if(
      m_gears_.begin(), m_gears_.end(), [this](const Gear & g) {
        return g.get_gear_number() < m_current_gear_->get_gear_number();
      }) == m_gears_.end();
  }

  /**
   * @brief Check if the current gear is the top gear
   *
   * @return if the current gear is the top gear
   */
  bool is_top_gear() const
  {
    return std::find_if(
      m_gears_.begin(), m_gears_.end(), [this](const Gear & g) {
        return g.get_gear_number() > m_current_gear_->get_gear_number();
      }) == m_gears_.end();
  }

  /**
   * @brief Downshift. User is responsible for checking feasibility with `should_downshift`.
   *
   * @return If downshift succeeds. Error will be printed to std:cerr and the operation will be ignored.
   */
  bool downshift()
  {
    return set_gear(m_current_gear_->get_gear_number() - 1);
  }

  /**
   * @brief Upshift. User is responsible for checking feasibility with `should_upshift`.
   *
   * @return If upshift succeeds. Error will be printed to `std:cerr` and the operation will be ignored.
   */
  bool upshift()
  {
    return set_gear(m_current_gear_->get_gear_number() + 1);
  }

  /**
   * @brief Force set gear
   *
   * @param gear_number
   * @return If gearshift succeeds.
   */
  bool set_gear(const int8_t & gear_number)
  {
    const auto target_gear =
      std::find_if(
      m_gears_.begin(), m_gears_.end(), [gear_number](const Gear & g) {
        return g.get_gear_number() == gear_number;
      });
    if (target_gear == m_gears_.end()) {
      std::cerr << "Cannot set gear to " << gear_number;
      return false;
    } else {
      m_current_gear_ = target_gear;
      return true;
    }
  }

  /**
   * @brief Get the current gear number
   *
   * @return gear number
   */
  int8_t get_current_gear_number()
  {
    return m_current_gear_->get_gear_number();
  }

  /**
   * @brief Check if gear number is valid
   *
   * @param gear_number
   * @return if valid
   */
  bool has_gear(const int8_t & gear_number)
  {
    return std::find_if(
      m_gears_.begin(), m_gears_.end(), [gear_number](const Gear & g) {
        return g.get_gear_number() == gear_number;
      }) != m_gears_.end();
  }

private:
  std::vector<Gear> m_gears_;
  std::vector<Gear>::const_iterator m_current_gear_;
};

enum class GearShiftStatus : uint8_t
{
  UNINITIALIZED,
  AVAILABLE,
  LOCKEDOUT,
  UPSHIFTING,
  DOWNSHIFTING
};

struct ATManagerConfig
{
  typedef std::shared_ptr<ATManagerConfig> SharedPtr;
  std::vector<int8_t> gear_numbers;
  std::vector<double> min_rpms;
  std::vector<double> max_rpms;
  int8_t start_gear;
  double gear_change_wait_sec;
};

/**
 * @brief Automatic transmission manager
 *
 */
class ATManager
{
public:
  typedef std::unique_ptr<ATManager> UniquePtr;
  ATManager(rclcpp::Node & parent, const ATManagerConfig & config)
  : parent_(parent)
  {
    if (!(config.gear_numbers.size() == config.min_rpms.size() &&
      config.gear_numbers.size() == config.max_rpms.size()))
    {
      throw std::logic_error("Quantity mismatch: gear numbers, min rpms, and max rpms.");
    }
    auto gears = std::vector<Gear>();
    for (size_t i = 0; i < config.gear_numbers.size(); i++) {
      gears.push_back(Gear(config.gear_numbers[i], config.min_rpms[i], config.max_rpms[i]));
    }
    m_gearbox_ = Gearbox(gears, config.start_gear);
    m_gear_change_wait_sec_ = config.gear_change_wait_sec;
    gear_command_ = config.start_gear;
  }

  /**
   * @brief step the model when new gear information is available
   *
   * @param timestamp_sec timestamp in second
   * @param gear_number actual physical gear number
   * @param gear_shift_status gear shift status code from Raptor
   * @param rpm engine rpm
   */
  void update(
    const double & timestamp_sec, const int8_t & gear_number,
    const uint8_t & gear_shift_status, const double & rpm)
  {
    try {
      const auto status = static_cast<GearShiftStatus>(gear_shift_status);
      if (!(status >= GearShiftStatus::UNINITIALIZED && status <= GearShiftStatus::DOWNSHIFTING)) {
        throw 42;
      }
      m_gear_shift_status_ = status;
    } catch (...) {
      RCLCPP_ERROR(parent_.get_logger(), "Gearbox: invalid gearshift status.");
      return;
    }
    if (m_gearbox_.has_gear(gear_number)) {
      m_physical_gear_number_ = gear_number;
    }
    current_time_sec = timestamp_sec;
    current_rpm_ = rpm;
    step();
  }

  /**
   * @brief Get the desired gear command
   *
   * @return int8_t gear command
   */
  uint8_t get_gear_command() const
  {
    return gear_command_;
  }

  bool is_waiting_for_gear_shift() const
  {
    return waiting_for_shift_;
  }

  double get_last_gear_shift_time() const
  {
    return last_gearshift_time_sec;
  }

private:
  Gearbox m_gearbox_;
  int8_t m_physical_gear_number_ = 0;
  uint8_t m_physical_gear_shift_status = 0;
  double current_time_sec = -1.0;
  double current_rpm_ = 0.0;
  double last_gearshift_time_sec = -1.0;
  bool waiting_for_shift_ = false;
  GearShiftStatus m_gear_shift_status_;
  int8_t gear_command_ = 0;
  rclcpp::Node & parent_;

  double m_gear_change_wait_sec_ = 0.0;

  void step()
  {
    static constexpr auto ENGINE_STALL_RPM = 500U;
    const auto engine_running = current_rpm_ > ENGINE_STALL_RPM;
    if (!engine_running) {
      return;            // Don't do anything when the engine is not running
    }
    if (current_time_sec < 0.0) {
      return;            // Don't do anything when the first update has not come through
    }
    const auto may_shift = (m_gear_shift_status_ == GearShiftStatus::AVAILABLE ||
      m_gear_shift_status_ == GearShiftStatus::UNINITIALIZED ||
      m_gear_shift_status_ == GearShiftStatus::LOCKEDOUT);
    if (waiting_for_shift_) {
      if ((current_time_sec - last_gearshift_time_sec) < m_gear_change_wait_sec_) {
        return;               // Don't do anything if a gear shift is in progress
      }
      if (m_gear_shift_status_ == GearShiftStatus::DOWNSHIFTING ||
        m_gear_shift_status_ == GearShiftStatus::UPSHIFTING ||
        m_gear_shift_status_ == GearShiftStatus::UNINITIALIZED)
      {
        RCLCPP_WARN(
          parent_.get_logger(),
          "Gear shifting taking exceptionally long and still in progress.");
        last_gearshift_time_sec = current_time_sec;
        return;
      } else if (m_gear_shift_status_ == GearShiftStatus::LOCKEDOUT) {
        RCLCPP_WARN(parent_.get_logger(), "Gear shifting locked out!");
        last_gearshift_time_sec = current_time_sec;
        return;
      } else if (m_gear_shift_status_ == GearShiftStatus::AVAILABLE) {
        waiting_for_shift_ = false;
        if (m_physical_gear_number_ != m_gearbox_.get_current_gear_number()) {
          RCLCPP_WARN(parent_.get_logger(), "Gear shifting successful.");
          m_gearbox_.set_gear(m_physical_gear_number_);
        } else {
          RCLCPP_WARN(parent_.get_logger(), "Gear shifting failed. Gear did not change.");
          gear_command_ = m_physical_gear_number_;
        }
      }

    } else if (may_shift) {
      // When not waiting for gear shift, check if possible to gear shift.
      if (m_gearbox_.get_current_gear_number() != m_physical_gear_number_) {
        RCLCPP_WARN(
          parent_.get_logger(), "Gear number mismatch. Actual gear is %d. Forcing reset.",
          m_physical_gear_number_);
        m_gearbox_.set_gear(m_physical_gear_number_);
      } else if (m_gearbox_.should_downshift(current_rpm_)) {
        RCLCPP_WARN(parent_.get_logger(), "Commanding downshift");
        gear_command_ -= 1;  // TODO(haoru): lookup gear box to see what's the actual next gear
        last_gearshift_time_sec = current_time_sec;
        waiting_for_shift_ = true;
      } else if (m_gearbox_.should_upshift(current_rpm_)) {
        RCLCPP_WARN(parent_.get_logger(), "Commanding upshift");
        gear_command_ += 1;  // TODO(haoru): lookup gear box to see what's the actual next gear
        last_gearshift_time_sec = current_time_sec;
        waiting_for_shift_ = true;
      }
    }
  }
};

class GearManagerPlugin : public RvcPlugin
{
public:
  bool configure() override
  {
    auto gear_numbers_param = node().declare_parameter(
      "gear_manager.gear_numbers",
      std::vector<int64_t>{});

    auto config = ATManagerConfig{
      std::vector<int8_t>(gear_numbers_param.begin(), gear_numbers_param.end()),
      node().declare_parameter("gear_manager.min_rpms", std::vector<double>{}),
      node().declare_parameter("gear_manager.max_rpms", std::vector<double>{}),
      static_cast<int8_t>(node().declare_parameter("gear_manager.start_gear", 1)),
      node().declare_parameter("gear_manager.gear_change_wait_sec", 0.5)
    };
    at_manager_ = std::make_unique<ATManager>(node(), config);
    return true;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (has_engine_report_) {
      output_cmd.gear_cmd = static_cast<uint8_t>(at_manager_->get_gear_command());
    } else {
      RCLCPP_INFO_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 1000, "AT Manager Plugin: waiting for engine report.");
    }
    state().telemetry->gear_command = output_cmd.gear_cmd;
    state().telemetry->is_in_gearshift_cooldown = at_manager_->is_waiting_for_gear_shift();
    state().telemetry->last_gearshift_time_sec = at_manager_->get_last_gear_shift_time();
    return true;
  }

  void on_engine_report_update() override
  {
    const auto msg = const_state().engine_report;
    at_manager_->update(
      rclcpp::Time(
        msg->stamp).seconds(), msg->current_gear, msg->gear_shift_status, msg->engine_rpm);
    has_engine_report_ = true;
  }

private:
  ATManager::UniquePtr at_manager_{};
  bool has_engine_report_{false};
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::GearManagerPlugin, race::RvcPlugin)
