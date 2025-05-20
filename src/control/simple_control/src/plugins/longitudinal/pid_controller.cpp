#include <simple_control/controller_interface.hpp>
#include <boost/any.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "std_msgs/msg/u_int8.hpp"
namespace control
{
    namespace longitudinal
    {

        class PIDLonController : public ControllerInterface
        {
        public:
            PIDLonController() = default; // Add default constructor

            void setup(rclcpp::Node *node) override
            {
                ControllerInterface::setup(node);
                // Get PID parameters
                kp_ = node->declare_parameter("longitudinal.pid.kp", 0.5);
                ki_ = node->declare_parameter("longitudinal.pid.ki", 0.1);
                kd_ = node->declare_parameter("longitudinal.pid.kd", 0.1);

                // Get throttle limits
                min_throttle_ = node->declare_parameter("longitudinal.pid.min_throttle", -100.0);
                max_throttle_ = node->declare_parameter("longitudinal.pid.max_throttle", 100.0);
                

                max_velocity_ = node->declare_parameter("longitudinal.pid.max_velocity", 10.0);
                // Initialize error terms
                prev_error_ = 0.0;
                integral_error_ = 0.0;
                abs_activate_ = false;
                abs_counter_ = 0;
                last_time_ = node->now();

                // Get Gear changing infos
                launch_ = node->declare_parameter("longitudinal.gear.launch",true);
                max_gear_ = node->declare_parameter("longitudinal.gear.max_gear",6);
                neutral_gear_ = node->declare_parameter("longitudinal.gear.neutral_gear",0);
                shift_rpm_0_to_1_ = node->declare_parameter("longitudinal.gear.shift_rpm_0_to_1",3000.0);
                shift_spd_1_to_2_ = node->declare_parameter("longitudinal.gear.shift_spd_1_to_2",15.0);
                shift_spd_2_to_3_ = node->declare_parameter("longitudinal.gear.shift_spd_2_to_3",25.0);
                shift_spd_3_to_4_ = node->declare_parameter("longitudinal.gear.shift_spd_3_to_4",35.0);
                shift_spd_4_to_5_ = node->declare_parameter("longitudinal.gear.shift_spd_4_to_5",45.0);
                shift_spd_5_to_6_ = node->declare_parameter("longitudinal.gear.shift_spd_5_to_6",55.0);

                // TCS and Slip ratio estimation 
                front_coeff_ = node->declare_parameter("longitudinal.wheel.front_coeff",3.3326);
                rear_coeff_ = node->declare_parameter("longitudinal.wheel.rear_coeff",3.2370);
                max_slip_ratio_ = node->declare_parameter("longitudinal.tcs.slip_ratio_max",0.15);
                min_slip_ratio_ = node->declare_parameter("longitudinal.tcs.slip_ratio_min",-0.15);
                tcs_power_ratio_ = node->declare_parameter("longitudinal.tcs.tcs_power_ratio",0.2);
                abs_brake_ratio_ = node->declare_parameter("longitudinal.tcs.abs_brake_ratio",0.2);
                abs_interval_ = node->declare_parameter("longitudinal.tcs.abs_interval",3);
                prev_slip_ratio_ = 0;
                prev_derivative_slip_ = 0;
                pred_step_ = node->declare_parameter("longitudinal.tcs.pred_step",2);
                
                // Sliding Mode Control parameters for ABS
                k_smc_ = node->declare_parameter("longitudinal.tcs.k_smc", 5.0);      // 滑模控制增益
                lambda_smc_ = node->declare_parameter("longitudinal.tcs.lambda_smc", 2.0); // 滑模面参数
                eta_smc_ = node->declare_parameter("longitudinal.tcs.eta_smc", 0.5);    // 抖振抑制参数


            }

            void runStep(const State::SharedPtr state,
                         autoware_control_msgs::msg::Control &control,  std_msgs::msg::UInt8 &gear_cmd, uint8_t vehicle_flag) override
            {
                try
                {
                    if (!state->vehicle_state)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run PID controller because vehicle state is not available");
                        return;
                    }
                    if (!state->local_path)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run PID controller because local path is not available");
                        return;
                    }
                    const auto vehicle_state = state->vehicle_state;
                    const auto path = state->local_path;

                    // Get target velocity from path, capped by max_velocity_
                    double target_velocity = std::min(static_cast<double>(path->points.back().longitudinal_velocity_mps), max_velocity_);

                    // Calculate time delta
                    auto current_time = parent_node_->now();
                    double dt = (current_time - last_time_).seconds();
                    last_time_ = current_time;

                    // Calculate error
                    double error = target_velocity - vehicle_state->velocity;

                    // PID calculations
                    integral_error_ += error * dt;
                    double derivative_error = (error - prev_error_) / dt;

                    // Calculate acceleration command
                    double acceleration = kp_ * error +
                                      ki_ * integral_error_ +
                                      kd_ * derivative_error;

                    // TCS komasa adds 
                    double wheel_spd_L1 = vehicle_state->wheel_speed_L1 / front_coeff_;
                    double wheel_spd_L2 = vehicle_state->wheel_speed_L2 / rear_coeff_;
                    double wheel_spd_R1 = vehicle_state->wheel_speed_R1 / front_coeff_;
                    double wheel_spd_R2 = vehicle_state->wheel_speed_R2 / rear_coeff_;

                    double slip_ratio_L2 = 0.0f;
                    double slip_ratio_R2 = 0.0f;

                    if (vehicle_state->acc_x > 0.0) // 当前在加速
                    {
                        slip_ratio_L2 = (wheel_spd_L1 - wheel_spd_L2)/(wheel_spd_L1+0.01f);
                        slip_ratio_R2 = (wheel_spd_R1 - wheel_spd_R2)/(wheel_spd_R1+0.01f);
                    }
                    else // 当前在减速，前轮可能也在制动所以轮速不一定正比于车速
                    {
                        slip_ratio_L2 = (vehicle_state->velocity - wheel_spd_L2)/(vehicle_state->velocity+0.01f);
                        slip_ratio_R2 = (vehicle_state->velocity - wheel_spd_R2)/(vehicle_state->velocity+0.01f);
                    }
                    
                    // 请在这里补充TCS的部分
                    // TCS 和 ABS 部分
                    bool tcs_active = false;
                    // 计算用于滑模控制的滑移率
                    double current_slip_ratio = std::max(slip_ratio_L2,slip_ratio_R2);
                    // 计算滑移率导数
                    double derivative_slip = (current_slip_ratio - prev_slip_ratio_) / dt;
                    double derivative_slip_filtered = 0.5 * prev_derivative_slip_ + 0.5 * derivative_slip; // 低通滤波

                    double target_slip_ratio = max_slip_ratio_;

                    std::cout << "当前车速：" << vehicle_state->velocity << "当前左后轮滑移率: " << slip_ratio_L2 << " 当前右后轮滑移率: " << slip_ratio_R2 << std::endl;
                    // 检查左后轮滑移率是否超出范围
                    if (slip_ratio_R2 < min_slip_ratio_|| slip_ratio_L2 < min_slip_ratio_) {
                        tcs_active = true;
                        std::cout << "<<<<******************TCS activate******************>>>>" << std::endl;
                    }

                    // 检查右后轮滑移率是否超出范围
                    double pred_next_slip_ratio = current_slip_ratio + pred_step_*derivative_slip_filtered*dt;
                    if (current_slip_ratio > max_slip_ratio_ || pred_next_slip_ratio > max_slip_ratio_) {
                        abs_activate_ = true;
                        abs_counter_ = 0;
                        std::cout << "<<<<******************ABS activate******************>>>>" << std::endl;
                        
                    }
                    else{
                        if (abs_activate_){ // 如果当前滑移率不超，但是仍然处于abs模式还没回来
                            abs_counter_ += 1; // 进行一次计数
                            if (abs_counter_ >= abs_interval_){
                                abs_activate_ = false; // 关闭当前的abs状态
                            }
                        }
                    }



                    // Clamp throttle to limits
                    acceleration = std::clamp(acceleration, min_throttle_, max_throttle_);
                    // RCLCPP_DEBUG_STREAM(logger_, "Throttle: " << throttle << " Target velocity: " << target_velocity << " Current velocity: " << vehicle_state->velocity);
                    // Update control command
                
      
                    // Store error for next iteration
                    prev_error_ = error;


                    // Gear Control
                    uint8_t the_gear_cmd = vehicle_state->gear; // 默认保持当前档位
                    double throttle = 0.0;
                    double braking = 0.0;

                    std::cout << "当前vehicle_flag的状态为" << static_cast<int>(vehicle_flag) << std::endl;

                    // 紧急停止状态处理
                    if (vehicle_flag != 1u) {
                        // 紧急停止：全力刹车并逐步降档到空挡
                        braking = min_throttle_;
                        
                        // 序列式变速箱只能逐个降档
                        if (vehicle_state->gear > 0u) {
                            the_gear_cmd = vehicle_state->gear - 1u;
                        }
                        
                        // 紧急状态下忽略油门控制
                        throttle = 0.0;
                    } 
                    // 正常比赛状态
                    else {
                        // 处理油门和刹车控制
                        if (acceleration > 0.0) {
                            throttle = std::clamp(acceleration, 0.0, max_throttle_);
                            braking = 0.0;
                        } else {
                            throttle = 0.0;
                            braking = std::clamp(acceleration, min_throttle_, 0.0);
                        }
                        
                        // 正常行驶的换挡逻辑
                        if (throttle > 0.0) {
                            // 根据速度决定升档逻辑
                            if (vehicle_state->velocity < shift_spd_1_to_2_) {
                                // 一档起步逻辑
                                if (vehicle_state->gear == 0u) {
                                    // 空挡且转速足够，可以挂一档准备起步
                                    if (vehicle_state->engine_rpm > shift_rpm_0_to_1_) {
                                        the_gear_cmd = 1u;
                                    }
                                } else if (vehicle_state->gear == 1u) {
                                    // 已经在一档，保持
                                    the_gear_cmd = 1u;
                                } else {
                                    // 当前在更高档位但速度太低，应该降档
                                    the_gear_cmd = vehicle_state->gear - 1u;
                                }
                            } 
                            // 根据速度范围决定合适的档位
                            else if (vehicle_state->velocity < shift_spd_2_to_3_) {

                                if (vehicle_state->gear < 2u){ 
                                    // 当前档位低于设定档位，应该升档
                                    the_gear_cmd = vehicle_state->gear + 1u;
                                } 
                                else if(vehicle_state->gear > 2u){
                                    // 当前在更高档位但速度太低，应该降档
                                    the_gear_cmd = vehicle_state->gear - 1u;
                                }

                            } 
                            else if (vehicle_state->velocity < shift_spd_3_to_4_) {
                                if (vehicle_state->gear < 3u){ 
                                    // 当前档位低于设定档位，应该升档
                                    the_gear_cmd = vehicle_state->gear + 1u;
                                } 
                                else if(vehicle_state->gear > 3u){
                                    // 当前在更高档位但速度太低，应该降档
                                    the_gear_cmd = vehicle_state->gear - 1u;
                                }
                            } 
                            else if (vehicle_state->velocity < shift_spd_4_to_5_) {
                                if (vehicle_state->gear < 4u){ 
                                    // 当前档位低于设定档位，应该升档
                                    the_gear_cmd = vehicle_state->gear + 1u;
                                } 
                                else if(vehicle_state->gear > 4u){
                                    // 当前在更高档位但速度太低，应该降档
                                    the_gear_cmd = vehicle_state->gear - 1u;
                                }
                            } 
                            else if (vehicle_state->velocity < shift_spd_5_to_6_) {
                                if (vehicle_state->gear < 5u){ 
                                    // 当前档位低于设定档位，应该升档
                                    the_gear_cmd = vehicle_state->gear + 1u;
                                } 
                                else if(vehicle_state->gear > 5u){
                                    // 当前在更高档位但速度太低，应该降档
                                    the_gear_cmd = vehicle_state->gear - 1u;
                                }
                            } 
                            else {
                                if (vehicle_state->gear < 6u){ 
                                    // 当前档位低于设定档位，应该升档
                                    the_gear_cmd = vehicle_state->gear + 1u;
                                } 
                            }
                        } 
                        // 刹车时的降档逻辑
                        else if (braking > 0.0) {
                            // 根据当前速度和刹车情况决定是否需要降档
                            if (vehicle_state->gear > 0u) {
                                // 根据当前速度判断合适的目标档位
                                uint8_t target_gear = 0u;
                                
                                if (vehicle_state->velocity >= shift_spd_5_to_6_+5) target_gear = 6u;
                                else if (vehicle_state->velocity >= shift_spd_4_to_5_+5) target_gear = 5u;
                                else if (vehicle_state->velocity >= shift_spd_3_to_4_+5) target_gear = 4u;
                                else if (vehicle_state->velocity >= shift_spd_2_to_3_+5) target_gear = 3u;
                                else if (vehicle_state->velocity >= shift_spd_1_to_2_+5) target_gear = 2u;
                                else target_gear = 1u;
                                
                                // 序列式变速箱只能逐个降档
                                if (vehicle_state->gear > target_gear) {
                                    the_gear_cmd = vehicle_state->gear - 1u;
                                }
                            }
                        }
                    }

                    // 特殊情况：空挡准备弹射起步
                    if (vehicle_state->gear == 0u) {
                        // 空挡时油门由外部控制，用于弹射起步
                        throttle = std::clamp(acceleration, 0.0, max_throttle_);
                    }


                    // 如果 TCS 或者 ABS 被激活，调整油门开度
                    if (tcs_active) {
                        
                        // 可以根据滑移率超出范围的程度来调整油门开度的减少量
                        // 这里简单地将油门/制动开度减少一定比例，例如 80%
                        throttle *= tcs_power_ratio_;
                        // // 确保调整后的油门开度仍在有效范围内
                        // throttle = std::clamp(throttle, min_throttle_, max_throttle_);
                    }
                    if (abs_activate_){
                        braking *= abs_brake_ratio_*std::pow(abs_counter_+1,2);
                        // // 滑模控制计算
                        // double s = derivative_slip_filtered + lambda_smc_ * (current_slip_ratio - target_slip_ratio); // 滑模面
                        
                        // // 设计控制律 u = u_eq + u_s
                        // // 这里简化处理，直接使用滑模控制律
                        // double braking_smc = -k_smc_ * (current_slip_ratio - target_slip_ratio) - eta_smc_ * std::clamp(s, -0.1,0.1);

                        // braking = braking_smc * abs_brake_ratio_;
                        // braking = std::clamp(braking, min_throttle_, 0.0);
                    }

                    prev_slip_ratio_ = current_slip_ratio;

                    gear_cmd.data = the_gear_cmd;

                    control.longitudinal.acceleration = throttle;
                    control.longitudinal.jerk = braking;
                    control.longitudinal.is_defined_acceleration = true;

                }
                catch (const boost::bad_any_cast &e)
                {
                    RCLCPP_ERROR(logger_,
                                 "Error casting state variables: %s", e.what());
                }
            }

            const char *get_plugin_name() override
            {
                return "pid_controller";
            }

        private:
            // PID gains
            double kp_;
            double ki_;
            double kd_;

            // Throttle limits
            double min_throttle_;
            double max_throttle_;

            // Error terms
            double prev_error_;
            double integral_error_;

            // Timing
            rclcpp::Time last_time_;

            // Max velocity
            double max_velocity_;

            // Gear Infomation
            uint8_t max_gear_;
            uint8_t neutral_gear_;
            bool launch_;
        
            // TCS
            double front_coeff_;
            double rear_coeff_;
            double max_slip_ratio_;
            double min_slip_ratio_;
            double tcs_power_ratio_;
            double abs_brake_ratio_;

            // Gear Control
            double shift_rpm_0_to_1_;
            double shift_spd_1_to_2_;
            double shift_spd_2_to_3_;
            double shift_spd_3_to_4_;
            double shift_spd_4_to_5_;
            double shift_spd_5_to_6_;

            // 滑模控制参数
            double k_smc_;        // 滑模控制增益
            double lambda_smc_;   // 滑模面参数
            double eta_smc_;      // 抖振抑制参数
            
            // 滑模控制状态变量
            double prev_slip_ratio_;
            double prev_derivative_slip_;
            bool abs_activate_;
            int abs_counter_;
            int abs_interval_;
            int pred_step_;

            
        };
    } // namespace longitudinal
} // namespace control

PLUGINLIB_EXPORT_CLASS(control::longitudinal::PIDLonController, control::ControllerInterface)
