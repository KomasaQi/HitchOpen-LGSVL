# Copyright 2022 Siddharth Saha

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.events import matches_action
import datetime
from pathlib import Path
from base_common import get_param_file, get_sim_time_launch_arg, check_val_in_list, get_share_file


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir", default_value="None", description="TTL Directory to use"
    )
    race_type_arg = DeclareLaunchArgument(
        "race_type", default_value="None", description="Race Type"
    )
    config = get_param_file("race_decision_engine")
    bt_file = get_share_file(
        "race_decision_engine", "config", "behavior_trees", "multi_car_iac_bt.xml"
    )

    rde_log_path = Path("src/planning/race_decision_engine/logs")
    rde_log_path.mkdir(parents=True, exist_ok=True)

    rde_log_file = str(
        rde_log_path / f"rde_fbl_{datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')}.fbl"
    )

    rde_node = LifecycleNode(
        package="race_decision_engine",
        executable="iac_bt_node_exe",
        name="race_decision_engine_node",
        output="screen",
        namespace="",
        parameters=[
            config,
            use_sim_time,
            {
                "ttl_csv_dir": LaunchConfiguration("ttl_dir"),
                "behavior_tree.bt_fp": bt_file,
                "behavior_tree.log_fp": rde_log_file,
            },
        ],
        remappings=[
            ("manual_cmd_filtered", "/rde/input_manual_command"),
            ("ttc", "/rde/trajectory_command"),
            ("localization_state", "/vehicle/state"),
            ("manual_cmd_raw", "/joystick/control_command"),
            ("race_control", "/rc_to_ct"),
            ("opp_cars", "/tracked_objects"),
            ("low_level_fault_report", "/vehicle/low_level_fault_report"),
            ("push2pass_report", "/vehicle/push2pass_report"),
        ],
        emulate_tty=True,
        condition=IfCondition(
            check_val_in_list(
                "race_type",
                [
                    "HAWAII_GOKART_UHMCP",
                    "IAC_LVMS",
                    "IAC_PUTNAM",
                    "IAC_KS",
                    "SVL_PUTNAM",
                    "SVL_LVMS",
                    "AW_SIM_IAC_IMS",
                    "AW_SIM_IAC_LVMS",
                    "AW_SIM_IAC_KS",
                    "AW_SIM_IAC_PUTNAM",
                    "IAC_LOR",
                    "IAC_KSC",
                    "IAC_IMS",
                ],
            )
        ),
    )

    rde_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rde_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(rde_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_configure")),
    )

    rde_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=rde_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(rde_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_activate")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_configure", default_value="true"),
            DeclareLaunchArgument("auto_activate", default_value="true"),
            declare_use_sim_time_cmd,
            ttl_dir_arg,
            race_type_arg,
            rde_node,
            rde_configure_event_handler,
            rde_activate_event_handler,
        ]
    )
