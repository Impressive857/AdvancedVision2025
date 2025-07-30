import launch
import launch_ros


def generate_launch_description():
    
    vision_system_node = launch_ros.actions.Node(
        package="vision_system", executable="vision_system_node", output="screen"
    )

    return launch.LaunchDescription(
        [
            vision_system_node
        ]
    )
