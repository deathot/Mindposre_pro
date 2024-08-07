from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    
    start_server = Node(package="dofbot_moveit",
                        executable="dofbot_server",
                        output="screen")
    
    start_model = Node(package="dofbot_garbage_yolov5",
                       executable="block_cls",
                       output="screen")
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dofbot_moveit"), "rviz", "dofbot.rviz"]
    )
    
    rviz2_node = Node(package="rviz2",
                       executable="rviz2",
                       name="rviz2",
                       output="log",
                       arguments=['-d', rviz_config_file])
    
    delayed_rviz2_node = TimerAction(period=18.0, actions=[rviz2_node])
    
    return LaunchDescription(
        [
            start_server,
            start_model,
            delayed_rviz2_node
        ]
    )
    
    
    