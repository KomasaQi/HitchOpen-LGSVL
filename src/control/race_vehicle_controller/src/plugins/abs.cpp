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

#include <memory>
#include <fstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_model/vehicle_model.hpp"
#include "vehicle_model/ros_param_loader.hpp"

#include "race_vehicle_controller/external/pid_controller.hpp"
#include "base_common/low_pass_filter.hpp"

#include "race_vehicle_controller/plugin.hpp"
#include "race_msgs/msg/engine_report.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"

using race_msgs::msg::WheelSpeedReport;
#define TEST_ABS false
#define DEBUG false

namespace race
{
class ABS : public RvcPlugin
{
public:
  bool configure() override
  {
    started_braking = false;
    for (int i = 0; i < delay_steps; i++) {
      last_commands[i] = 0.;
    }

    abs_pid = PidController(
      "abs_pid", PidCoefficients{
        node().declare_parameter<double>("abs.abs_pid.kp", 25000.0),
        node().declare_parameter<double>("abs.abs_pid.ki", 2.4),
        node().declare_parameter<double>("abs.abs_pid.kd", 0.5),
        node().declare_parameter<double>("abs.abs_pid.min_cmd", 500.0),
        node().declare_parameter<double>("abs.abs_pid.max_cmd", 4000.0),
        node().declare_parameter<double>("abs.abs_pid.min_i", -1000.0),
        node().declare_parameter<double>("abs.abs_pid.max_i", 3000.0)});
    target_slip = node().declare_parameter<double>("abs.abs_pid.target_slip", 0.1);
    i_start = node().declare_parameter<double>("abs.abs_pid.i_start", 1000.);
    trigger = node().declare_parameter<double>("abs.abs_pid.trigger", 1000.0);
    moi = node().declare_parameter<double>("abs.abs_mpc.moi", 9.3);
    wheel_radius = node().declare_parameter<double>("abs.wheel_radius", 0.31);
    delay_steps = node().declare_parameter<int>("abs.abs_mpc.delay_steps", 2);
    time_step = node().declare_parameter<double>("abs.abs_mpc.time_step", 0.05);
    use_mpc = node().declare_parameter<int>("abs.use_mpc", 1);
    K_tao = node().declare_parameter<double>("abs.abs_mpc.K_tao", 10.);
    lon_control_type_ = 0;
    Fz0 = node().declare_parameter<double>("lon_tire_params.Fz0");
    pdx1 = node().declare_parameter<double>("lon_tire_params.pdx1");
    pdx2 = node().declare_parameter<double>("lon_tire_params.pdx2");
    // pdx3 = node().declare_parameter<double>("lon_tire_params.pdx3");

    pcx1 = node().declare_parameter<double>("lon_tire_params.pcx1");

    pkx1 = node().declare_parameter<double>("lon_tire_params.pkx1");
    pkx2 = node().declare_parameter<double>("lon_tire_params.pkx2");
    pkx3 = node().declare_parameter<double>("lon_tire_params.pkx3");

    pex1 = node().declare_parameter<double>("lon_tire_params.pex1");
    pex2 = node().declare_parameter<double>("lon_tire_params.pex2");
    pex3 = node().declare_parameter<double>("lon_tire_params.pex3");

    phx1 = node().declare_parameter<double>("lon_tire_params.phx1");
    phx2 = node().declare_parameter<double>("lon_tire_params.phx2");

    pvx1 = node().declare_parameter<double>("lon_tire_params.pvx1");
    pvx2 = node().declare_parameter<double>("lon_tire_params.pvx2");

    const auto output_cutoff_freq = 3.0;
    brake_filter_ = LowPassFilter(
      output_cutoff_freq,
      const_config().control_output_interval_sec
    );
    wheels_sub_ = node().create_subscription<WheelSpeedReport>(
      "wheel_speeds",
      rclcpp::SensorDataQoS(),
      [&](WheelSpeedReport::SharedPtr msg) {on_wheel_speeds_received(msg);});
    abs_pid.reset_integral_error(i_start);
    return true;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    if (!const_state().all_input_received() || !const_state().path) {
      RCLCPP_WARN(node().get_logger(), "returned due to no state update or no input received");
      return true;
    }
    const auto & current_speed = const_state().kin_state->speed_mps;
    if (lon_control_type_ == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      if (const_state().kin_state->pose.pose.position.x > 80 && !started_braking && TEST_ABS) {
        started_braking = true;
        abs_pid.reset_integral_error(i_start);
      }
      auto radii = wheel_radius;
      auto wheel_omega = 0.;
      if (last_wheel_speeds) {
        wheel_omega = (last_wheel_speeds->rear_left + last_wheel_speeds->rear_right) / 2.;
      }
      auto slip = (current_speed - wheel_omega * radii) / current_speed;
      if (DEBUG) {
        RCLCPP_DEBUG(node().get_logger(), "Slip %f", slip);
      }
      double brake_abs = 0.;
      if (started_braking || (output_cmd.brake_cmd > trigger)) {
        brake_abs = abs_pid.update(
          target_slip - slip,
          const_config().control_output_interval_sec);
      }


      if (DEBUG) {
        RCLCPP_DEBUG(node().get_logger(), "Brake init : %f", output_cmd.brake_cmd);
      }
      if (started_braking || (output_cmd.brake_cmd > trigger)) {
        output_cmd.accelerator_cmd = 0;
        if (use_mpc == 1) {
          output_cmd.brake_cmd = calc_req_torque(slip, current_speed);
        } else {
          output_cmd.brake_cmd = brake_abs;
        }
        if (current_speed != last_speed) {
          last_speed = current_speed;
        }
      }
      state().telemetry->brake_cmd = output_cmd.brake_cmd;
    }
    return true;
  }

  double calc_long_tire_force(double slip, double long_speed)
  {
    double Fz_f = 0., Fz_r = 0.;
    model().calc_norm_force(long_speed, 0., Fz_f, Fz_r);
    Fz_r /= 2;
    double dfz = (Fz_r - Fz0) / Fz0;
    double mu = pdx1 + pdx2 * dfz;
    double Dx = mu * Fz_r;
    double Cx = pcx1;
    double BCDx = Fz_r * (pkx1 + pkx2 * dfz) * exp(pkx3 * dfz);
    double Bx = BCDx / (Cx * Dx);
    double Ex = (pex1 + pex2 * dfz + pex3 * dfz * dfz);
    double shx = phx1 + phx2 * dfz;
    double svx = pvx1 + pvx2 * dfz;
    double kx = slip + shx;
    double Fx = Dx * sin(Cx * atan(Bx * kx - Ex * (Bx * kx - atan(Bx * kx)))) + svx;
    return -Fx;
  }

  void predict_slip_speed(double torque, double & slip, double & long_speed)
  {
    double ratio = 1 - slip;
    double Ft = calc_long_tire_force(slip, long_speed);
    double Fr = model().calc_resistance(long_speed, 0);
    double term1 = ratio * (2 * Ft + Fr) /
      (model().get_config().chassis_config->total_mass * long_speed);
    slip += ((torque + Ft * wheel_radius) * wheel_radius / (moi * long_speed) + term1) * time_step;
    long_speed += ((2 * Ft + Fr) / model().get_config().chassis_config->total_mass) * time_step;
  }

  double calc_req_torque(double slip, double long_speed)
  {
    if (long_speed == last_speed) {return last_torque;}
    for (int i = 0; i < delay_steps; i++) {
      predict_slip_speed(last_commands[i], slip, long_speed);
      if (DEBUG) {
        RCLCPP_DEBUG(node().get_logger(), "Predicted : %f %f", slip, long_speed);
      }
    }
    double ratio = 1 - slip;
    double Ft = calc_long_tire_force(slip, long_speed);
    if (DEBUG) {
      RCLCPP_DEBUG(node().get_logger(), "Tire force : %f", Ft);
    }
    double Fr = model().calc_resistance(long_speed, 0);
    double term1 = ratio * (2 * Ft + Fr) /
      (model().get_config().chassis_config->total_mass * long_speed);
    if (DEBUG) {
      RCLCPP_DEBUG(node().get_logger(), "term1 : %f", term1);
    }
    double desired_der = K_tao * (target_slip - slip);
    double req_torque = (desired_der - term1) * moi * long_speed / wheel_radius - Ft * wheel_radius;
    // last_speed = long_speed;
    last_torque = std::min(std::max(2. * req_torque, 20. * std::max(long_speed, 50.)), 3000.);
    for (int i = 0; i < delay_steps - 1; i++) {
      last_commands[i] = last_commands[i + 1];
    }
    last_commands[delay_steps - 1] = last_torque;
    if (DEBUG) {
      RCLCPP_DEBUG(node().get_logger(), "Req torque is : %f", last_torque);
    }
    // return 4000.;
    return last_torque;
  }

private:
  void on_wheel_speeds_received(WheelSpeedReport::SharedPtr msg)
  {
    last_wheel_speeds = msg;
  }
  PidController acc_pid_;
  PidController abs_pid;

  double Fz0, pdx1, pdx2;
  double pcx1, pkx1, pkx2, pkx3;
  double pex1, pex2, pex3;
  double phx1, phx2;
  double pvx1, pvx2;
  double K_tao;
  double time_step, moi, wheel_radius;
  int delay_steps, use_mpc;
  bool started_braking = false;
  double target_slip, trigger, i_start;
  double last_speed = 0.;
  double last_torque = 0.;
  double last_commands[100];
  LowPassFilter brake_filter_;
  rclcpp::Subscription<WheelSpeedReport>::SharedPtr wheels_sub_;
  WheelSpeedReport::SharedPtr last_wheel_speeds;
  uint8_t lon_control_type_;
};
}   // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::ABS, race::RvcPlugin)
