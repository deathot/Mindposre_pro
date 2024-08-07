from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace="AclRos",
            package='ros2_dvpp2_resize',
            executable='ros2_dvpp2_resize_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                        {"img_src_param": "SUBMSG"}, # LOCALIMG SUBMSG
                        {"pub_local_topic_name": "/resized_local_image"},
                        {"pub_sub_topic_name": "/resized_submsg_image"},
                        {"sub_topic_name": "/yuvimage_raw"},
                        {"yuvimg_file_name": "/root/ros2_workspace_dvpp/src/ros2_dvpp2_resize/data/dvpp_vpc_1920x1080_nv12.yuv"},
                        {"inputDataWidth": 1920},
                        {"inputDataHeight": 1080},
                        {"outputDataWidth": 960},
                        {"outputDataHeight": 540},
                        {"publish_interval": 500}
                    ]
        ),
        launch_ros.actions.Node(
            namespace="AclRos",
            package='ros2_dvpp2_resize',
            executable='publisher_yuvmsg_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                        {"pub_yuv_image_raw": "/yuvimage_raw"},
                        {"yuvimg_file_name": "/root/ros2_workspace_dvpp/src/ros2_dvpp2_resize/data/dvpp_vpc_1920x1080_nv12.yuv"},
                        {"inputDataWidth": 1920},
                        {"inputDataHeight": 1080},
                        {"publish_interval": 500}
                    ]
        )
    ])