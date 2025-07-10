import launch
import launch_ros


def generate_launch_description():
    # 产生launch描述
    
    action_node_img2map_node = launch_ros.actions.Node(
        package="vision_system", executable="vision_system_node", output="screen"
    )

    return launch.LaunchDescription(
        [
            # action动作
            action_node_img2map_node
        ]
    )
