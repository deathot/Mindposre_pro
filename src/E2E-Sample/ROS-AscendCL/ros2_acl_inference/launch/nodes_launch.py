# Copyright (c) Huawei Technologies Co., Ltd. 2023. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace="AclRos",
            package='ros2_acl_inference', 
            executable='inference_node', 
            output='screen',
            emulate_tty=True,
            parameters=[
                {"model_file": "/root/dev_ws/src/ros2_acl_inference/model/yolov3.om"}, # 根据个人网络模型（如yolov3.om）的绝对地址进行修改
                {"model_name": "yolov3.om"}, # 网络模型的名称
                {"pub_topic_name": "/detection_result"}, # 发布目标检测结果话题的名称
                {"sub_topic_name": "/image_raw"}, # 订阅图像输入话题的名称
                {"modelWidth": 416}, # 网络模型的输入尺寸（宽）
                {"modelHeight": 416} # 网络模型的输入尺寸（高）
            ]
        ),
    ])
