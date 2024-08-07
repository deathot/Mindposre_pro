import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():

    # Get the path of the YAML configuration file
    pkg_dir = get_package_share_directory("service_client")
    config_file = os.path.join(
        pkg_dir, "config", "hdmi_sample_image_folder.yaml")

    # For a single image yaml
    # Create a list of Node instances for the service_client nodes
    service_client_nodes = [
        Node(
            package='service_client',
            executable='service_client',
            parameters=[config_file],
        )
    ]

    delayed_service_client_node = TimerAction(
        period=3.0, actions=service_client_nodes)

    service_server_node = Node(
        package='service_client',
        executable='service_server',
        parameters=[config_file],
    )

    # Combine all the nodes into the LaunchDescription
    return LaunchDescription([
        delayed_service_client_node,
        service_server_node,
    ])
