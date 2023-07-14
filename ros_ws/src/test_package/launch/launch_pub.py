"""Launch a lifecycle ZED node and the Robot State Publisher"""

import os


import launch
from launch import LaunchIntrospector

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg

def generate_launch_description():
    # Launch Description
    ld = launch.LaunchDescription()

    # Prepare the ZED node
    zed_node = LifecycleNode(
        namespace = 'test',        # must match the namespace in config -> YAML
        name = 'publisher',        # must match the node name in config -> YAML
        package = 'test_package',
        executable = 'publisher',
        output = 'screen',
        parameters = [
        ]
    )

    # Make the ZED node take the 'configure' transition
    zed_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = launch.events.matchers.matches_action(zed_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the ZED node take the 'activate' transition
    zed_activate_trans_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matchers.matches_action(zed_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    # When the ZED node reaches the 'inactive' state, make it take the 'activate' transition and start the Robot State Publisher
    zed_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            goal_state = 'inactive',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'INACTIVE' state, start the 'Robot State Publisher' node and 'activating'." ),
                # Change State event ( inactive -> active )
                zed_activate_trans_event,
            ],
        )
    )

    # When the ZED node reaches the 'active' state, log a message.
    zed_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node = zed_node,
            goal_state = 'active',
            entities = [
                # Log
                LogInfo( msg = "'ZED' reached the 'ACTIVE' state" ),
            ],
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action( zed_inactive_state_handler )
    ld.add_action( zed_active_state_handler )
    ld.add_action( zed_node )
    ld.add_action( zed_configure_trans_event)

    return ld