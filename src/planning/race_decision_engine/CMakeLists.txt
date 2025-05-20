cmake_minimum_required(VERSION 3.8)
project(race_decision_engine)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_BASE_HEADER
  include/${PROJECT_NAME}/base/base_lifecycle_node.hpp
  include/${PROJECT_NAME}/base/base_bt_lifecycle_node.hpp
)

set(${PROJECT_NAME}_BASE_SRC
    src/base/base_lifecycle_node.cpp
    src/base/base_bt_lifecycle_node.cpp
)

set(${PROJECT_NAME}_COMMON_HEADER
  include/${PROJECT_NAME}/common/input.hpp
  include/${PROJECT_NAME}/common/output.hpp
  include/${PROJECT_NAME}/common/utils.hpp
)

set(${PROJECT_NAME}_COMMON_SRC
  src/common/utils.cpp
)

set(${PROJECT_NAME}_BT_NODES_HEADER
  include/${PROJECT_NAME}/bt_nodes/action_nodes/print_valid_ttl.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_car_state.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_closest_rival_car.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_joystick_state.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/keep_joystick_state.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/go_joystick_autonomous.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/stop_car_safe.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/stop_car_immediate.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/stop_car_emergency.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_green_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_yellow_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_yellow_catchup_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_round_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_round_overtaking_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_round_catchup_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_pit_road_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_pit_lane_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_pit_ttl_index.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_push2pass_active.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_push2pass_inactive.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_race_ttl_index.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_left_ttl_index.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_right_ttl_index.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_optimal_ttl_index.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_pit_crawl_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/increment_lap.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_flags.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_ttl_index.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_speed_limit.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_to_pitting.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/reduce_to_tsp_speed.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/reset_scale_factor.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_scale_factor_from_lookup.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_second_closest_rival_car.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/create_false_rival_car.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/create_false_sec_rival_car.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/fill_rival_car_info.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/fill_sec_rival_car_info.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_allowed_to_overtake.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_attacker_overtake_complete.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_attacker_overtake_started.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_defender_overtake_complete.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_defender_overtake_started.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_highest_speed_rival_car.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_follow_mode.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_no_gap.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_attacker_preparing_gap.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_attacker_attacking_gap.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_defender_overtaken_gap.hpp
  include/${PROJECT_NAME}/bt_nodes/action_nodes/set_gap.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_localization_timeout.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_localization_emergency.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_input_manual_command_timeout.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_input_manual_command_emergency.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_race_control_timeout.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_opp_car_detection_timeout.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_use_perception.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_use_params.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_purple_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_red_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_black_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_yellow_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_green_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_green_speed_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_blue_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_checkered_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_ekill_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_fcy_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_no_track_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_stop_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_attacker_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_defender_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_orange_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_pitting.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_wgreen_flag.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_box.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_crawl.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_entrance.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_exit.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_lane.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_road.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_straight.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_yellow_speed_range.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_back_straight.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_left_ttl.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_passing_zone_from_lookup.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_pit_ttl.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_in_round_speed_range.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_crossed_sf.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_crossed_passing_line.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_tsp_enabled.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_scale_factor_enabled.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_sec_rival_car_front_gap_lesser_than.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_sec_rival_car_exists.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_sec_rival_car_back_gap_lesser_than.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_sec_rival_car_front.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_sec_rival_car_back.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_low_level_fault_safe_stop.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_low_level_fault_immediate_stop.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_low_level_fault_emergency_stop.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_low_level_fault_timeout.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_allowed_to_overtake.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_attacker_overtake_complete.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_attacker_overtake_started.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_crossed_passing_sf.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_defender_overtake_complete.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_defender_overtake_started.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_ego_car_left.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_ego_car_near_race_line.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_ego_car_right.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_flag_changed_from_green.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_flag_changed_from_wgreen.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_no_tracked_objects.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_back_gap_greater_than.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_front_gap_greater_than.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_back_gap_lesser_than.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_front_gap_lesser_than.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_exists.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_ego_ttls_match.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_front.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_back.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_left.hpp
  include/${PROJECT_NAME}/bt_nodes/condition_nodes/if_rival_car_right.hpp
)

set(${PROJECT_NAME}_BT_NODES_SRC
  src/bt_nodes/action_nodes/print_valid_ttl.cpp
  src/bt_nodes/action_nodes/set_car_state.cpp
  src/bt_nodes/action_nodes/set_closest_rival_car.cpp
  src/bt_nodes/action_nodes/set_joystick_state.cpp
  src/bt_nodes/action_nodes/keep_joystick_state.cpp
  src/bt_nodes/action_nodes/go_joystick_autonomous.cpp
  src/bt_nodes/action_nodes/stop_car_safe.cpp
  src/bt_nodes/action_nodes/stop_car_immediate.cpp
  src/bt_nodes/action_nodes/stop_car_emergency.cpp
  src/bt_nodes/action_nodes/set_green_speed.cpp
  src/bt_nodes/action_nodes/set_yellow_speed.cpp
  src/bt_nodes/action_nodes/set_yellow_catchup_speed.cpp
  src/bt_nodes/action_nodes/set_round_speed.cpp
  src/bt_nodes/action_nodes/set_round_overtaking_speed.cpp
  src/bt_nodes/action_nodes/set_round_catchup_speed.cpp
  src/bt_nodes/action_nodes/set_pit_road_speed.cpp
  src/bt_nodes/action_nodes/set_pit_lane_speed.cpp
  src/bt_nodes/action_nodes/set_pit_ttl_index.cpp
  src/bt_nodes/action_nodes/set_push2pass_active.cpp
  src/bt_nodes/action_nodes/set_push2pass_inactive.cpp
  src/bt_nodes/action_nodes/set_race_ttl_index.cpp
  src/bt_nodes/action_nodes/set_left_ttl_index.cpp
  src/bt_nodes/action_nodes/set_right_ttl_index.cpp
  src/bt_nodes/action_nodes/set_optimal_ttl_index.cpp
  src/bt_nodes/action_nodes/set_pit_crawl_speed.cpp
  src/bt_nodes/action_nodes/increment_lap.cpp
  src/bt_nodes/action_nodes/set_flags.cpp
  src/bt_nodes/action_nodes/set_ttl_index.cpp
  src/bt_nodes/action_nodes/set_speed_limit.cpp
  src/bt_nodes/action_nodes/set_to_pitting.cpp
  src/bt_nodes/action_nodes/reduce_to_tsp_speed.cpp
  src/bt_nodes/action_nodes/reset_scale_factor.cpp
  src/bt_nodes/action_nodes/set_scale_factor_from_lookup.cpp
  src/bt_nodes/action_nodes/set_second_closest_rival_car.cpp
  src/bt_nodes/action_nodes/create_false_rival_car.cpp
  src/bt_nodes/action_nodes/create_false_sec_rival_car.cpp
  src/bt_nodes/action_nodes/fill_rival_car_info.cpp
  src/bt_nodes/action_nodes/fill_sec_rival_car_info.cpp
  src/bt_nodes/action_nodes/set_allowed_to_overtake.cpp
  src/bt_nodes/action_nodes/set_attacker_overtake_complete.cpp
  src/bt_nodes/action_nodes/set_attacker_overtake_started.cpp
  src/bt_nodes/action_nodes/set_defender_overtake_complete.cpp
  src/bt_nodes/action_nodes/set_defender_overtake_started.cpp
  src/bt_nodes/action_nodes/set_highest_speed_rival_car.cpp
  src/bt_nodes/action_nodes/set_follow_mode.cpp
  src/bt_nodes/action_nodes/set_attacker_preparing_gap.cpp
  src/bt_nodes/action_nodes/set_attacker_attacking_gap.cpp
  src/bt_nodes/action_nodes/set_defender_overtaken_gap.cpp
  src/bt_nodes/action_nodes/set_no_gap.cpp
  src/bt_nodes/action_nodes/set_gap.cpp
  src/bt_nodes/condition_nodes/if_localization_timeout.cpp
  src/bt_nodes/condition_nodes/if_localization_emergency.cpp
  src/bt_nodes/condition_nodes/if_input_manual_command_emergency.cpp
  src/bt_nodes/condition_nodes/if_input_manual_command_timeout.cpp
  src/bt_nodes/condition_nodes/if_race_control_timeout.cpp
  src/bt_nodes/condition_nodes/if_opp_car_detection_timeout.cpp
  src/bt_nodes/condition_nodes/if_use_perception.cpp
  src/bt_nodes/condition_nodes/if_use_params.cpp
  src/bt_nodes/condition_nodes/if_purple_flag.cpp
  src/bt_nodes/condition_nodes/if_red_flag.cpp
  src/bt_nodes/condition_nodes/if_black_flag.cpp
  src/bt_nodes/condition_nodes/if_yellow_flag.cpp
  src/bt_nodes/condition_nodes/if_green_flag.cpp
  src/bt_nodes/condition_nodes/if_green_speed_flag.cpp
  src/bt_nodes/condition_nodes/if_blue_flag.cpp
  src/bt_nodes/condition_nodes/if_checkered_flag.cpp
  src/bt_nodes/condition_nodes/if_ekill_flag.cpp
  src/bt_nodes/condition_nodes/if_fcy_flag.cpp
  src/bt_nodes/condition_nodes/if_no_track_flag.cpp
  src/bt_nodes/condition_nodes/if_stop_flag.cpp
  src/bt_nodes/condition_nodes/if_attacker_flag.cpp
  src/bt_nodes/condition_nodes/if_defender_flag.cpp
  src/bt_nodes/condition_nodes/if_orange_flag.cpp
  src/bt_nodes/condition_nodes/if_pitting.cpp
  src/bt_nodes/condition_nodes/if_wgreen_flag.cpp
  src/bt_nodes/condition_nodes/if_in_pit_box.cpp
  src/bt_nodes/condition_nodes/if_in_pit_crawl.cpp
  src/bt_nodes/condition_nodes/if_in_pit_entrance.cpp
  src/bt_nodes/condition_nodes/if_in_pit_exit.cpp
  src/bt_nodes/condition_nodes/if_in_pit_lane.cpp
  src/bt_nodes/condition_nodes/if_in_pit_road.cpp
  src/bt_nodes/condition_nodes/if_in_straight.cpp
  src/bt_nodes/condition_nodes/if_in_yellow_speed_range.cpp
  src/bt_nodes/condition_nodes/if_in_back_straight.cpp
  src/bt_nodes/condition_nodes/if_in_left_ttl.cpp
  src/bt_nodes/condition_nodes/if_in_passing_zone_from_lookup.cpp
  src/bt_nodes/condition_nodes/if_in_pit_ttl.cpp
  src/bt_nodes/condition_nodes/if_in_round_speed_range.cpp
  src/bt_nodes/condition_nodes/if_crossed_sf.cpp
  src/bt_nodes/condition_nodes/if_crossed_passing_line.cpp
  src/bt_nodes/condition_nodes/if_tsp_enabled.cpp
  src/bt_nodes/condition_nodes/if_scale_factor_enabled.cpp
  src/bt_nodes/condition_nodes/if_sec_rival_car_front_gap_lesser_than.cpp
  src/bt_nodes/condition_nodes/if_sec_rival_car_exists.cpp
  src/bt_nodes/condition_nodes/if_sec_rival_car_back_gap_lesser_than.cpp
  src/bt_nodes/condition_nodes/if_sec_rival_car_front.cpp
  src/bt_nodes/condition_nodes/if_sec_rival_car_back.cpp
  src/bt_nodes/condition_nodes/if_low_level_fault_safe_stop.cpp
  src/bt_nodes/condition_nodes/if_low_level_fault_immediate_stop.cpp
  src/bt_nodes/condition_nodes/if_low_level_fault_emergency_stop.cpp
  src/bt_nodes/condition_nodes/if_low_level_fault_timeout.cpp
  src/bt_nodes/condition_nodes/if_allowed_to_overtake.cpp
  src/bt_nodes/condition_nodes/if_attacker_overtake_complete.cpp
  src/bt_nodes/condition_nodes/if_attacker_overtake_started.cpp
  src/bt_nodes/condition_nodes/if_crossed_passing_sf.cpp
  src/bt_nodes/condition_nodes/if_defender_overtake_complete.cpp
  src/bt_nodes/condition_nodes/if_defender_overtake_started.cpp
  src/bt_nodes/condition_nodes/if_ego_car_left.cpp
  src/bt_nodes/condition_nodes/if_ego_car_near_race_line.cpp
  src/bt_nodes/condition_nodes/if_ego_car_right.cpp
  src/bt_nodes/condition_nodes/if_flag_changed_from_green.cpp
  src/bt_nodes/condition_nodes/if_flag_changed_from_wgreen.cpp
  src/bt_nodes/condition_nodes/if_no_tracked_objects.cpp
  src/bt_nodes/condition_nodes/if_rival_car_back_gap_greater_than.cpp
  src/bt_nodes/condition_nodes/if_rival_car_front_gap_greater_than.cpp
  src/bt_nodes/condition_nodes/if_rival_car_back_gap_lesser_than.cpp
  src/bt_nodes/condition_nodes/if_rival_car_front_gap_lesser_than.cpp
  src/bt_nodes/condition_nodes/if_rival_car_exists.cpp
  src/bt_nodes/condition_nodes/if_rival_ego_ttls_match.cpp
  src/bt_nodes/condition_nodes/if_rival_car_front.cpp
  src/bt_nodes/condition_nodes/if_rival_car_back.cpp
  src/bt_nodes/condition_nodes/if_rival_car_left.cpp
  src/bt_nodes/condition_nodes/if_rival_car_right.cpp
)

set(IAC_BT_HEADER
  include/${PROJECT_NAME}/impl/iac_bt.hpp
)

set(IAC_BT_SRC
  src/impl/iac_bt.cpp
)

generate_parameter_library(
  race_decision_engine_parameters
  resources/parameters.yaml
  include/${PROJECT_NAME}/custom_validators.hpp
)

ament_auto_add_library(${PROJECT_NAME}_COMMON SHARED
    ${${PROJECT_NAME}_COMMON_HEADER}
    ${${PROJECT_NAME}_COMMON_SRC}
)

target_link_libraries(${PROJECT_NAME}_COMMON
  race_decision_engine_parameters
)

ament_auto_add_library(${PROJECT_NAME}_BT_NODES SHARED
  ${${PROJECT_NAME}_BT_NODES_HEADER}
  ${${PROJECT_NAME}_BT_NODES_SRC}
)

target_link_libraries(${PROJECT_NAME}_BT_NODES
  race_decision_engine_parameters
  ${PROJECT_NAME}_COMMON
)

ament_auto_add_library(${PROJECT_NAME}_BASE SHARED
    ${${PROJECT_NAME}_BASE_HEADER}
    ${${PROJECT_NAME}_BASE_SRC}
)

target_link_libraries(${PROJECT_NAME}_BASE
  race_decision_engine_parameters
  ${PROJECT_NAME}_COMMON
  ${PROJECT_NAME}_BT_NODES
)

ament_auto_add_library(IAC_BT SHARED
  ${IAC_BT_HEADER}
  ${IAC_BT_SRC}
)

target_link_libraries(IAC_BT
  race_decision_engine_parameters
  ${PROJECT_NAME}_COMMON
  ${PROJECT_NAME}_BT_NODES
  ${PROJECT_NAME}_BASE
)

rclcpp_components_register_node(IAC_BT
  PLUGIN "race::planning::race_decision_engine::nodes::implementations::IacBt"
  EXECUTABLE iac_bt_node_exe
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  param
  launch
)
