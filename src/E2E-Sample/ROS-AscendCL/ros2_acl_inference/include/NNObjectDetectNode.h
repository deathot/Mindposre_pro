/**
 *  Copyright (c) Huawei Technologies Co., Ltd. 2023. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */
 
#ifndef NN_OBJECT_DETECTION_NODE_H
#define NN_OBJECT_DETECTION_NODE_H

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "sensor_msgs/msg/image.hpp"
#include "NNBaseNode.h"
#include "AclInterfaceImpl.h"
#include "Utils.h"


namespace AclRos
{
    // 用户根据任务自定义的NNBaseNode派生类
    class NNObjectDetectNode : public NNBaseNode
    {
    public:
        explicit NNObjectDetectNode(std::string nodeName);
        
        ~NNObjectDetectNode();

    protected:
        // 实现基类的纯虚接口，用于配置模型参数
        Result SetModelPara() override;

    private:
        // ROS topic publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr msgPublisher = nullptr;

        // ROS topic subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr msgSubscription = nullptr;

        // ROS topic订阅回调函数
        void RosImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        // 由缩放后的yuv图像构造网络模型的输入
        Result CreateInputFromResizedImg(ImageData& image, std::vector<DataInfo> &inputData, uint32_t modelWidth, uint32_t modelHeight);

        // 自定义对模型输出的后处理
        Result Postprocess(std::vector<DataInfo> outputData, const sensor_msgs::msg::Image::ConstSharedPtr imgMsg);
        
        // 画目标检测框
        cv::Mat DrawBoundBoxToImage(std::vector<BBox>& detectionResults, cv::Mat imgMat);
    };
} // namespace AclRos

#endif
