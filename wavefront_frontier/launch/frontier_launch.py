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