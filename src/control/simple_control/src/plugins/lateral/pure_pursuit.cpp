#include <simple_control/controller_interface.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <boost/any.hpp>
#include <math.h>
#include "std_msgs/msg/u_int8.hpp"
#include "simple_control/MFASMC_Controller.hpp"
namespace control
{
    namespace lateral
    {
        class PurePursuitController : public ControllerInterface
        {
        public:
            PurePursuitController() = default;

            void setup(rclcpp::Node *node) override
            {
                ControllerInterface::setup(node);
                // Get parameters
                look_ahead_distance_ = node->declare_parameter("lateral.pure_pursuit.look_ahead_distance", 15.0);
                wheel_base_ = node->declare_parameter("lateral.pure_pursuit.wheel_base", 2.7);
                max_steering_angle_ = node->declare_parameter("lateral.pure_pursuit.max_steering_angle", 0.52);
                smoothing_factor_ = node->declare_parameter("lateral.pure_pursuit.smoothing_factor", 0.5);

                // Get Compensate Parameters
                max_add_steering_angle_ = node->declare_parameter("lateral.mfasmc.max_add_steering_angle", 0.0349);
                kp_ = node->declare_parameter("lateral.pid_comp.kp", 0.02);
                ki_ = node->declare_parameter("lateral.pid_comp.ki", 0.001);
                kd_ = node->declare_parameter("lateral.pid_comp.kd", 0.01);
                integrate_saturate_ = node->declare_parameter("lateral.pid_comp.int_sat", 0.1);
                k_lat_err_ = node->declare_parameter("lateral.mfasmc.k_lat_err", -1.0);
                k_th_err_ = node->declare_parameter("lateral.mfasmc.k_th_err", 3.0);

                // control_value save
                previous_steering_ = 0.0;
            }

            void runStep(const State::SharedPtr state,
                         autoware_control_msgs::msg::Control &control, std_msgs::msg::UInt8 &gear_cmd, uint8_t vehicle_flag) override
            {
                try
                {
                    if (!state->vehicle_state)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run pure pursuit controller because vehicle state is not available");
                        return;
                    }
                    if (!state->local_path)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run pure pursuit controller because local path is not available");
                        return;
                    }
                    const auto vehicle_state = state->vehicle_state;
                    const auto path = state->local_path;

                    // Find look-ahead point
                    auto target_point = findLookAheadPoint(vehicle_state, path);

                    // Calculate steering angle using pure pursuit formula
                    double alpha =  atan2(target_point.y - vehicle_state->y, target_point.x - vehicle_state->x) - vehicle_state->yaw;
                    double alpha_normalized = atan2(sin(alpha), cos(alpha));
                    double steering = atan2(2.0 * wheel_base_ * sin(alpha_normalized),
                                                 look_ahead_distance_);

                    // Limit steering angle
                    steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_) * -1;

                    // Apply smoothing
                    steering = smoothing_factor_ * steering + (1.0 - smoothing_factor_) * previous_steering_;
                    previous_steering_ = steering;

                    RCLCPP_DEBUG_STREAM(logger_, "vehicle_state (x, y, yaw): (" << vehicle_state->x << ", " << vehicle_state->y << ", " << vehicle_state->yaw << ")");
                    RCLCPP_DEBUG_STREAM(logger_, "target_point (x, y): (" << target_point.x << ", " << target_point.y << ")");
                    RCLCPP_DEBUG_STREAM(logger_, "steering: " << (steering * 180.0 / M_PI) << " deg, alpha_normalized: " << alpha_normalized << "\n");

                    //********************************** 在这里加入横向跟踪误差的输出********************************
                    // 横向跟踪误差计算
                    struct Point2D { double x, y; };
                    Point2D closest_point;
                    double min_distance = std::numeric_limits<double>::max();

                    // 查找最近点
                    for (const auto &point : path->points) {
                        double dx = point.pose.position.x - vehicle_state->x;
                        double dy = point.pose.position.y - vehicle_state->y;
                        double dist = sqrt(dx*dx + dy*dy);
                        if (dist < min_distance) {
                            min_distance = dist;
                            closest_point = {point.pose.position.x, point.pose.position.y};
                        }
                    }

                    // 计算路径切线方向（处理边界情况）
                    size_t closest_idx = 0;
                    for (size_t i=0; i<path->points.size(); i++) {
                        double dx = path->points[i].pose.position.x - vehicle_state->x;
                        double dy = path->points[i].pose.position.y - vehicle_state->y;
                        if (sqrt(dx*dx + dy*dy) == min_distance) {
                            closest_idx = i;
                            break;
                        }
                    }
                    size_t prev_idx = (closest_idx == 0) ? 0 : closest_idx-1;
                    size_t next_idx = (closest_idx == path->points.size()-1) ? path->points.size()-1 : closest_idx+1;

                    double ax = path->points[prev_idx].pose.position.x;
                    double ay = path->points[prev_idx].pose.position.y;
                    double bx = path->points[next_idx].pose.position.x;
                    double by = path->points[next_idx].pose.position.y;
                    double px = vehicle_state->x;
                    double py = vehicle_state->y;

                    // 向量AB = (bx-ax, by-ay)，向量AP = (px-ax, py-ay)
                    double abx = bx - ax, aby = by - ay;
                    double apx = px - ax, apy = py - ay;
                    double cross = abx * apy - aby * apx;
                    double len = sqrt(abx*abx + aby*aby);
                    double lateral_error = (len == 0) ? 0.0 : cross / len;

                    // 输出横向跟踪误差
                    std::cout << "Lateral Tracking Error: " << lateral_error << " m" << std::endl;

                    // 航向误差计算
                    double path_yaw = atan2(by - ay, bx - ax);
                    double heading_error = atan2(sin(path_yaw - vehicle_state->yaw), cos(path_yaw - vehicle_state->yaw));

                    // 输出航向误差
                    std::cout << "Heading Error: " << heading_error * 180.0 / M_PI << " deg" << std::endl;

                    //**********************************横向误差和航向误差部分结束***************************************

                    // 在此处加入根据横向误差和航向误差对steering进行PID补偿的代码
                    // ================== PID补偿代码开始 ==================
                    // 1. 计算总误差（横向误差与航向误差加权和）
                    double total_error = k_lat_err_ * lateral_error + k_th_err_ * heading_error;

                    // 2. 积分项计算（带饱和限制）
                    integral_error_ += total_error;
                    // 积分饱和处理
                    integral_error_ = std::clamp(integral_error_, -integrate_saturate_, integrate_saturate_);

                    // 3. 微分项计算（当前误差 - 前一次误差）
                    double derivative_error = total_error - previous_total_error_;

                    // 4. PID输出计算
                    double pid_output = kp_ * total_error + 
                                        ki_ * integral_error_ + 
                                        kd_ * derivative_error;

                    // 5. 限制PID补偿量不超过最大允许调整范围
                    pid_output = std::clamp(pid_output, -max_add_steering_angle_, max_add_steering_angle_);

                    // 6. 应用PID补偿到原始转向角
                    steering += pid_output;

                    // 7. 重新限制转向角在物理极限内（避免补偿后超限）
                    steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_);

                    // 8. 保存当前总误差用于下次微分计算
                    previous_total_error_ = total_error;
                    // ================== PID补偿代码结束 ==================



                    // Apply steering command
                    control.lateral.steering_tire_angle = steering;
                    control.lateral.is_defined_steering_tire_rotation_rate = false;
                }
                catch (const boost::bad_any_cast &e)
                {
                    RCLCPP_ERROR(logger_, "Error casting state variables: %s", e.what());
                }
            }

            const char *get_plugin_name() override
            {
                return "pure_pursuit";
            }

        private:
            double look_ahead_distance_;
            double wheel_base_;
            double max_steering_angle_;
            double smoothing_factor_;
            double previous_steering_;
            double max_add_steering_angle_;
            double kp_;
            double ki_;
            double kd_;
            double integrate_saturate_;
            double k_lat_err_;
            double k_th_err_;

            // ================== 新增PID成员变量 ==================
            double integral_error_ = 0.0;       // 积分误差（需初始化）
            double previous_total_error_ = 0.0;  // 前一次总误差（用于微分计算）
            // ================== 成员变量结束 ==================

            struct Point2D
            {
                double x;
                double y;
            };

            Point2D findLookAheadPoint(const VehicleState::SharedPtr vehicle,
                                       const autoware_planning_msgs::msg::Path::SharedPtr path)
            {
                // Point2D closest_point{path->points[0].pose.position.x,
                //                       path->points[0].pose.position.y};
                // double min_distance = std::numeric_limits<double>::max();

                // // Find the first point that is at least look_ahead_distance_ away
                // for (const auto &point : path->points)
                // {
                //     double dx = point.pose.position.x - vehicle->x;
                //     double dy = point.pose.position.y - vehicle->y;
                //     double distance = std::sqrt(dx * dx + dy * dy);

                //     if (distance >= look_ahead_distance_)
                //     {
                //         RCLCPP_DEBUG_STREAM(logger_, "look-ahead point: " << point.pose.position.x << ", " << point.pose.position.y << " distance: " << distance);
                //         return Point2D{point.pose.position.x, point.pose.position.y};
                //     }
                // }

                // If no point is found, return the last point
                return Point2D{path->points.back().pose.position.x,
                               path->points.back().pose.position.y};
            }
        };

    } // namespace lateral
} // namespace control

PLUGINLIB_EXPORT_CLASS(control::lateral::PurePursuitController, control::ControllerInterface)
