# Copyright (C) 2023 Unmanned Life
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


# This is a ROS2 launch file which starts the psdk_wrapper_node,
# configures it and activates it.

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg
import launch

# this is to fix too quick activation
# from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch.actions import LogInfo
#from lifecycle_msgs.msg import State

def generate_launch_description():
    """Launch the psdk_wrapper_node."""

    # Create LaunchConfiguration variables
    namespace = LaunchConfiguration("namespace")
    link_config_file_path = LaunchConfiguration("link_config_file_path")
    psdk_params_file_path = LaunchConfiguration("psdk_params_file_path")
    hms_return_codes_file = LaunchConfiguration("hms_return_codes_file")

    # Declare the namespace launch argument
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="wrapper",
        description="Namespace of the node",
    )

    # Declare wrapper parameters
    psdk_params_default_value = PathJoinSubstitution(
        [FindPackageShare("psdk_wrapper"), "cfg", "psdk_params.yaml"]
    )
    declare_psdk_params_cmd = DeclareLaunchArgument(
        "psdk_params_file_path",
        default_value=psdk_params_default_value,
        description="DJI PSDK ROS2 parameters",
    )

    # Declare link configuration file path
    link_config_default_value = PathJoinSubstitution(
        [FindPackageShare("psdk_wrapper"), "cfg", "link_config.json"]
    )

    declare_link_config_cmd = DeclareLaunchArgument(
        "link_config_file_path",
        default_value=link_config_default_value,
        description="DJI PSDK link configuration file path",
    )

    # Declare HMS known error codes JSON file path
    hms_codes_file_default_value = "hms.json"

    declare_hms_codes_cmd = DeclareLaunchArgument(
        "hms_return_codes_file",
        default_value=hms_codes_file_default_value,
        description="Path to JSON file with known DJI return codes",
    )

    hms_return_codes_path = PathJoinSubstitution(
        [FindPackageShare("psdk_wrapper"), "cfg", hms_return_codes_file]
    )

    declare_frame_prefix_arg = DeclareLaunchArgument(
        "tf_frame_prefix",
        default_value=namespace,
        description="TF frame prefix",
    )

    declare_location_arg = DeclareLaunchArgument(
        "location",
        default_value="granso",
        description="Location to use for local coordinate system",
    )
    declare_sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Is simulation used for the platform",
    )

    # Prepare the wrapper node
    wrapper_node = LifecycleNode(
        package="psdk_wrapper",
        executable="psdk_wrapper_node",
        name="psdk_wrapper_node",
        output="screen",
        namespace=namespace,
        parameters=[
            psdk_params_file_path,
            {
                "link_config_file_path": link_config_file_path,
                "hms_return_codes_path": hms_return_codes_path,
                "tf_frame_prefix": LaunchConfiguration("tf_frame_prefix"),
                "location": LaunchConfiguration("location"),
                "sim": LaunchConfiguration("sim"),
            },
        ],
    )

    # Configure lifecycle node
    wrapper_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate lifecycle node
    wrapper_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )


    wrapper_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=wrapper_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription()

    ld.add_action(LogInfo(msg=["tf_frame_prefix=", tf_frame_prefix_arg]))
    ld.add_action(LogInfo(msg=["location=", declare_location_arg]))

    # Declare Launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_frame_prefix_arg)
    ld.add_action(declare_location_arg)
    ld.add_action(declare_sim_arg)
    ld.add_action(declare_psdk_params_cmd)
    ld.add_action(declare_link_config_cmd)
    ld.add_action(declare_hms_codes_cmd)

    ld.add_action(wrapper_node)
    ld.add_action(wrapper_configure_trans_event)

    if False:
        ld.add_action(wrapper_activate_trans_event)
        # The above fire almost simultaneously, so:
        # ACTIVATE sometimes runs before CONFIGURE finishes - leads to an error
    else:
        # instead this handler is used
        ld.add_action(wrapper_activate_handler)

    return ld
