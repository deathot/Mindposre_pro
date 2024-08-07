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
 
#include <iostream>
#include <cstdio>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

#include "NNObjectDetectNode.h"

namespace {
    const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
        "aeroplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter",
        "bench", "bird", "cat", "dog", "horse",
        "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie",
        "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana",
        "apple", "sandwich", "orange", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake", "chair",
        "sofa", "potted plant", "bed", "dining table", "toilet",
        "TV monitor", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush" };

    enum class BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };

    const uint32_t lineSolid = 2;
    
    // opencv draw label params.
    const double fountScale = 0.5;
    const cv::Scalar fontColor(0, 0, 255);
    const uint32_t labelOffset = 11;

    // opencv color list for boundingbox
    const std::vector<cv::Scalar> colors {
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };
}

namespace AclRos
{
    NNObjectDetectNode::NNObjectDetectNode(std::string nodeName): NNBaseNode(nodeName)
    {
        this->declare_parameter<std::string>("model_file", "");
        this->declare_parameter<std::string>("model_name", "");
        this->declare_parameter<std::string>("pub_topic_name", "");
        this->declare_parameter<std::string>("sub_topic_name", "");
        this->declare_parameter<int>("modelWidth", 416);
        this->declare_parameter<int>("modelHeight", 416);

        std::stringstream ss;
        ss << "Param:" << "\n model_file:" << this->get_parameter("model_file").get_parameter_value().get<std::string>()
           << "\n model_name:" << this->get_parameter("model_name").get_parameter_value().get<std::string>();
        RCLCPP_INFO(rclcpp::get_logger("acl_ros2_node"), "%s", ss.str().c_str());

        // 使用基类Init()接口初始化，配置模型参数、开辟内存并加载模型
        Result ret = Init();
        if (ret != SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("acl_ros2_node"), "Init failed!");
            rclcpp::shutdown(); // TODO: 初始化失败最好不要杀掉整个进程
            return;
        }

        // 创建目标检测结果ROS msg的publisher
        std::string pubTopicName = this->get_parameter("pub_topic_name").get_parameter_value().get<std::string>();
        RCLCPP_INFO(rclcpp::get_logger("acl_ros2_node"), "Create publisher with topic: %s", pubTopicName.c_str());
        msgPublisher = this->create_publisher<sensor_msgs::msg::Image>(pubTopicName, 10);

        // 订阅图片话题并绑定回调函数
        std::string subTopicName = this->get_parameter("sub_topic_name").get_parameter_value().get<std::string>();
        RCLCPP_INFO(rclcpp::get_logger("acl_ros2_node"), "Create subscription with topic: %s", subTopicName.c_str());
        msgSubscription = this->create_subscription<sensor_msgs::msg::Image>(
                                            subTopicName,
                                            10,
                                            std::bind(&NNObjectDetectNode::RosImgCallback, this, std::placeholders::_1));
    }

    NNObjectDetectNode::~NNObjectDetectNode() 
    {
    }

    Result NNObjectDetectNode::SetModelPara()
    {
        if (!modelPara) return FAILED;

        std::string modelFile = this->get_parameter("model_file").get_parameter_value().get<std::string>();
        std::string modelName = this->get_parameter("model_name").get_parameter_value().get<std::string>();
        
        modelPara->modelFile = modelFile;
        modelPara->modelName = modelName;
    }

    void NNObjectDetectNode::RosImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg)
    {
        // 接收到一个图片msg
        if (!imgMsg) {
            RCLCPP_DEBUG(rclcpp::get_logger("acl_ros2_node"), "Get img failed");
            return;
        }

        if (!rclcpp::ok()) {
            return;
        }

        std::stringstream ss;
        ss << "Recved img encoding: " << imgMsg->encoding
            << ", h: " << imgMsg->height << ", w: " << imgMsg->width
            << ", step: " << imgMsg->step
            << ", frame_id: " << imgMsg->header.frame_id
            << ", stamp: " << imgMsg->header.stamp.sec << "_"
            << imgMsg->header.stamp.nanosec
            << ", data size: " << imgMsg->data.size();
        RCLCPP_INFO(rclcpp::get_logger("acl_ros2_node"), "%s", ss.str().c_str());

        // 用户自定义的前处理逻辑: copy image to device, convert to yuv, and resize
        std::chrono::steady_clock::time_point preStart = std::chrono::steady_clock::now();

        uint32_t modelWidth = this->get_parameter("modelWidth").as_int();
        uint32_t modelHeight = this->get_parameter("modelHeight").as_int();

        ImageData yuvImage;        
        Utils::CvtRosMsgToYuv420sp(imgMsg, yuvImage, imgMsg->encoding, modelWidth, modelHeight);

        // 构建模型输入
        std::vector<DataInfo> inputs;
        std::vector<DataInfo> outputs;
        CreateInputFromResizedImg(yuvImage, inputs, modelWidth, modelHeight); // 得到inputs

        std::chrono::steady_clock::time_point preEnd = std::chrono::steady_clock::now();
        std::cout << "Preprocess time: " << double(std::chrono::duration_cast<std::chrono::milliseconds>(preEnd - 
                                                                            preStart).count()) << " ms" << std::endl;

        // 开始预测，调父类Execute方法，走ACL推理流程
        Execute(inputs, outputs); // 由inputs得到outputs

        // 用户自定义的后处理逻辑
        std::chrono::steady_clock::time_point postStart = std::chrono::steady_clock::now();
        Postprocess(outputs, imgMsg); // 解析outputs得到输出
        std::chrono::steady_clock::time_point postEnd = std::chrono::steady_clock::now();
        std::cout << "Postprocess time: " << double(std::chrono::duration_cast<std::chrono::milliseconds>(postEnd - 
                                                                            postStart).count()) << " ms" << std::endl;
    }

    Result NNObjectDetectNode::CreateInputFromResizedImg(ImageData& resizedImg, std::vector<DataInfo> &inputData, uint32_t modelWidth, uint32_t modelHeight)
    {
        float imageInfo[4] = {(float)modelWidth, (float)modelHeight, (float)modelWidth, (float)modelHeight};
        uint32_t imageInfoSize = sizeof(imageInfo);
        void* buffer = malloc(imageInfoSize);
        memcpy(buffer, imageInfo, imageInfoSize);
        if (buffer == nullptr) {
            ERROR_LOG("ImageInfo buffer malloc failed");
            return FAILED;
        }
        // 填充模型输入inputData
        DataInfo batchInput1;
        batchInput1.data = resizedImg.data.get();
        batchInput1.size = resizedImg.size;
        inputData.push_back(batchInput1);

        DataInfo batchInput2;
        batchInput2.data = buffer;
        batchInput2.size = imageInfoSize;
        inputData.push_back(batchInput2);

        return SUCCESS;
    }

    Result NNObjectDetectNode::Postprocess(std::vector<DataInfo> outputData,
        const sensor_msgs::msg::Image::ConstSharedPtr imgMsg)
    {
        // 此object detection模型有两个输出，分别提领出来
        float* detectData = reinterpret_cast<float *>(outputData[0].data);
        uint32_t* boxNum = reinterpret_cast<uint32_t *>(outputData[1].data);

        // 后处理
        uint32_t totalBox = boxNum[0];
        std::vector<BBox> detectResults;

        uint32_t modelWidth = this->get_parameter("modelWidth").as_int();
        uint32_t modelHeight = this->get_parameter("modelHeight").as_int();

        float widthScale;
        float heightScale;
        // "bgr8" "rgb8"
        if (imgMsg->encoding != "mono8") {
            std::cout << "Using Camera." << std::endl;
            widthScale = (float)imgMsg->width / modelWidth;
            heightScale = (float)imgMsg->height / modelHeight;
        } else if (imgMsg->encoding == "mono8") {
            widthScale = (float)imgMsg->width / modelWidth;
            heightScale = (float)imgMsg->height * 2/3 / modelHeight;
        } else {
            ERROR_LOG("Wrong encoding type: %s", imgMsg->encoding.c_str());
            return FAILED;
        }

        for (uint32_t i = 0; i < totalBox; i++) {
            BBox boundBox;

            uint32_t score = uint32_t(detectData[totalBox * static_cast<uint32_t>(BBoxIndex::SCORE) + i] * 100);
            boundBox.rect.ltX = detectData[totalBox * static_cast<uint32_t>(BBoxIndex::TOPLEFTX) + i] * widthScale;
            boundBox.rect.ltY = detectData[totalBox * static_cast<uint32_t>(BBoxIndex::TOPLEFTY) + i] * heightScale;
            boundBox.rect.rbX = detectData[totalBox * static_cast<uint32_t>(BBoxIndex::BOTTOMRIGHTX) + i] * widthScale;
            boundBox.rect.rbY = detectData[totalBox * static_cast<uint32_t>(BBoxIndex::BOTTOMRIGHTY) + i] * heightScale;

            uint32_t objIndex = (uint32_t)detectData[totalBox * static_cast<uint32_t>(BBoxIndex::LABEL) + i];
            boundBox.text = yolov3Label[objIndex] + std::to_string(score) + "\%";
            printf("%d %d %d %d %s\n", boundBox.rect.ltX, boundBox.rect.ltY, boundBox.rect.rbX,
                boundBox.rect.rbY, boundBox.text.c_str());

            detectResults.emplace_back(boundBox);
        }

        cv_bridge::CvImageConstPtr cvImg = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(imgMsg),
            imgMsg->encoding);
        cv::Mat resultImg = DrawBoundBoxToImage(detectResults, cvImg->image); // 返回检测结果

        // 发布自定义的ROS msg
        if (!msgPublisher) {
            RCLCPP_ERROR(rclcpp::get_logger("acl_ros2_node"), "Invalid msgPublisher");
            return FAILED;
        }

        cv_bridge::CvImage cvi;
        cvi.header.stamp = this->get_clock()->now();
        cvi.header.frame_id = "image_result";
        cvi.encoding = imgMsg->encoding;
        cvi.image = resultImg;
        
        sensor_msgs::msg::Image imgPub;
        cvi.toImageMsg(imgPub);
        msgPublisher->publish(imgPub);

        return SUCCESS;
    }

    cv::Mat NNObjectDetectNode::DrawBoundBoxToImage(std::vector<BBox>& detectionResults, cv::Mat imgMat)
    {
        cv::Mat image = imgMat;
        for (int i = 0; i < detectionResults.size(); ++i) {
            cv::Point p1;
            cv::Point p2;
            p1.x = detectionResults[i].rect.ltX;
            p1.y = detectionResults[i].rect.ltY;
            p2.x = detectionResults[i].rect.rbX;
            p2.y = detectionResults[i].rect.rbY;
            cv::rectangle(image, p1, p2, colors[i % colors.size()], lineSolid);
            cv::putText(image, detectionResults[i].text, cv::Point(p1.x, p1.y + labelOffset),
                cv::FONT_HERSHEY_COMPLEX, fountScale, fontColor);
        }

        return image;
    }
} // namespace AclRos
