from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from ament_index_python.packages import get_package_share_directory
import os
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo, Shutdown    
from launch.actions import TimerAction
def generate_launch_description():


    frontier_node = LifecycleNode(name='get_frontier_service', namespace='',
        package='wavefront_frontier', executable='get_frontier_service', output='screen')





    register_event_handler_for_frontier_activation = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=frontier_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="node 'frontiernode' reached the 'configured' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(frontier_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    register_event_handler_for_frontier_shutdown = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=frontier_node, goal_state='finalized',
            entities=[
                LogInfo(
                    msg="node 'frontiernode' reached the 'finalized' state, shutting down process."),
                Shutdown(reason="all lifecycle nodes finalized"),
            ],
        )
    )

    change_to_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(frontier_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )



    return LaunchDescription([
        register_event_handler_for_frontier_activation,
        register_event_handler_for_frontier_shutdown,
        frontier_node,
        change_to_configure

    ])



    """Run lifecycle nodes via launch."""
    ld = launch.LaunchDescription()

    # Prepare the talker node.
    talker_node = launch_ros.actions.LifecycleNode(
        name='talker', namespace='',
        package='lifecycle', executable='lifecycle_talker', output='screen')

    # When the talker reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_talker_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'talker' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(talker_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the talker node reaches the 'active' state, log a message and start the listener node.
    register_event_handler_for_talker_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=talker_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'talker' reached the 'active' state, launching 'listener'."),
                launch_ros.actions.LifecycleNode(
                    name='listener', namespace='',
                    package='lifecycle', executable='lifecycle_listener', output='screen'),
            ],
        )
    )

    # Make the talker node take the 'configure' transition.
    emit_event_to_request_that_talker_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(talker_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_talker_reaches_inactive_state)
    ld.add_action(register_event_handler_for_talker_reaches_active_state)
    ld.add_action(talker_node)
    ld.add_action(emit_event_to_request_that_talker_does_configure_transition)


