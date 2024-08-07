/**
 *  Copyright (c) Huawei Technologies Co., Ltd. 2023.All rights reserved.
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
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <csignal>
#include <getopt.h>
#include <exception>
#include <iostream>
#include <vector>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include "DvppProcess.h"
#include "Utils.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;
using namespace AclRos;
using std::placeholders::_1;

aclrtContext g_context = nullptr;

class DvppResizeSample : public rclcpp::Node {
    DvppProcess dvpp_proc;
public:
    DvppResizeSample() : Node("DvppResizeSample"), countPubLocal(0), countPub(0) 
    {
        AclInit();
        // Dvpp2 initialization
        dvpp_proc.InitResource();

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is to choose image source!";

        this->declare_parameter<std::string>("img_src_param", "LOCALIMG", param_desc); // LOCALIMG SUBMSG
        this->declare_parameter<std::string>("pub_local_topic_name", "/resized_local_image");
        this->declare_parameter<std::string>("pub_sub_topic_name", "/resized_submsg_image");
        this->declare_parameter<std::string>("sub_topic_name", "/yuvimage_raw");
        this->declare_parameter<std::string>("yuvimg_file_name", 
            "/root/ros2_workspace_dvpp/src/ros2_dvpp2_resize/data/dvpp_vpc_1920x1080_nv12.yuv");
        this->declare_parameter<int>("publish_interval", 500);
        this->declare_parameter<int>("inputDataWidth", 1920); // 输入图片的宽
        this->declare_parameter<int>("inputDataHeight", 1080); // 输入图片的高
        this->declare_parameter<int>("outputDataWidth", 960); // 输出图片的宽
        this->declare_parameter<int>("outputDataHeight", 540); // 输出图片的高

        std::string pub_local_topic_name = this->get_parameter("pub_local_topic_name").
            get_parameter_value().get<std::string>();
        std::string pub_sub_topic_name = this->get_parameter("pub_sub_topic_name").
            get_parameter_value().get<std::string>();
        std::string sub_topic_name = this->get_parameter("sub_topic_name").
            get_parameter_value().get<std::string>();
        int publish_interval = this->get_parameter("publish_interval").get_parameter_value().get<int>();

        std::string imgSrcParam = this->get_parameter("img_src_param").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "Hello, you choose %s as image source!", imgSrcParam.c_str());
        // local image publisher
        if (imgSrcParam == "LOCALIMG") {
            publisherLocal = this->create_publisher<sensor_msgs::msg::Image>(pub_local_topic_name, 10);
            timerPub = this->create_wall_timer(std::chrono::milliseconds(publish_interval), 
                                            std::bind(&DvppResizeSample::ResizeLocalCallback, this));
        }
        // subscriber /yuvimage_raw
        if (imgSrcParam == "SUBMSG") {
            subscriptionImg = this->create_subscription<sensor_msgs::msg::Image>(
                sub_topic_name, 10, std::bind(&DvppResizeSample::ResizeSubmsgCallback, this, _1));
            // Msg image publisher
            publisherSub = this->create_publisher<sensor_msgs::msg::Image>(pub_sub_topic_name, 10);
        }
    }

    ~DvppResizeSample()
    {
        dvpp_proc.DestroyResource();
        AclDestoryResource();
    }

private:
    Result AclInit()
    {
        if (isAclInitialized) {
            WARN_LOG("ACL resource has been initialized.\n");
            return SUCCESS;
        }
        std::cout << "===========AclInit()===========" << std::endl;
        aclError aclRet = aclInit(nullptr);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("aclInit failed with %d.\n", aclRet);
            return FAILED;
        }
        // 默认情况下，项目运行在device 0上，当有多个device时，根据实际情况选择device号码

        aclRet = aclrtSetDevice(0);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("aclrtSetDevice(0) failed with %d.\n", aclRet);
            aclRet = aclFinalize();
            if (aclRet != ACL_SUCCESS) {
                ERROR_LOG("finalize acl failed with %d.\n", aclRet);
            }
            return FAILED;
        }

        aclRet = aclrtCreateContext(&g_context, 0);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl create context failed with %d.", aclRet);
            aclRet = aclrtResetDevice(0);
            if (aclRet != ACL_SUCCESS) {
                ERROR_LOG("reset device(0) failed with %d.\n", aclRet);
            }
            aclRet = aclFinalize();
            if (aclRet != ACL_SUCCESS) {
                ERROR_LOG("finalize acl failed with %d.\n", aclRet);
            }
            return FAILED;
        }

        isAclInitialized = true;
        return SUCCESS;
    }

    Result AclDestoryResource()
    {
        if (isAclDestroyed) {
            WARN_LOG("ACL resource has been destroyed.\n");
            return SUCCESS;
        }
        std::cout << "===========AclDestoryResource()===========" << std::endl;
        aclError aclRet = aclrtDestroyContext(g_context);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed with %d.", aclRet);
        }
        aclRet = aclrtResetDevice(0);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("reset device(0) failed with %d.\n", aclRet);
        }
        aclRet = aclFinalize();
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("finalize acl failed with %d.\n", aclRet);
        }

        isAclDestroyed = true;
        return SUCCESS;
    }

public:
    cv::Mat Yuv2Mat(ImageData& inputYuvData)
    {
        int w = inputYuvData.width;
        int h = inputYuvData.height;
        int bufLen = w * h * 3 / 2;
        cv::Mat yuvImg;
        yuvImg.create(h * 3 / 2, w, CV_8UC1);
        memcpy(yuvImg.data, inputYuvData.data.get(), bufLen);
        cv::Mat rgbImg;
        cv::cvtColor(yuvImg, rgbImg, CV_YUV2BGR_NV12); // 显示rgb图像
        return rgbImg;
    }

    Result ResizeLocalCallback()
    {
        ImageData inputData;
        ImageData outputData;
        int32_t interpolation = 0; // 缩放算法，0：Bilinear算法
        inputData.width = this->get_parameter("inputDataWidth").get_parameter_value().get<int>();
        inputData.height = this->get_parameter("inputDataHeight").get_parameter_value().get<int>();
        outputData.width = this->get_parameter("outputDataWidth").get_parameter_value().get<int>();
        outputData.height = this->get_parameter("outputDataHeight").get_parameter_value().get<int>();
        
        dvpp_proc.InitInputDesc(inputData);
        uint32_t dstBufferSize = dvpp_proc.InitOutputDesc(outputData);
        std::string yuvimg_file_name = this->get_parameter("yuvimg_file_name").get_parameter_value().get<std::string>();

        Utils::ReadPicFile(inputData, yuvimg_file_name.c_str());
        uint64_t startResizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        dvpp_proc.Resize(inputData, outputData, interpolation);
        uint64_t endResizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
        double totalResizeTime = double(endResizeTime - startResizeTime) / 1000;
        std::cout << "Resize local image success!" << " totalResizeTime: " << totalResizeTime << " ms " << std::endl;
        
        ImageData notAlignData;
        notAlignData.size = dstBufferSize;
        uint8_t *inputBuf = (uint8_t *)malloc(dstBufferSize);
        if (inputBuf == nullptr) {
            ERROR_LOG("No memory available!\n");
            return FAILED;
        }
        memset(inputBuf, 0, dstBufferSize);
        notAlignData.data.reset(inputBuf, [](uint8_t* p) { delete[](p); });

        dvpp_proc.GetNotAlignBuffer(outputData, notAlignData);

        cv::Mat image = Yuv2Mat(notAlignData); // outputData notAlignData
        cv_bridge::CvImage cviYUV;
        cviYUV.header.stamp = this->get_clock()->now();
        cviYUV.header.frame_id = "image_yuv";
        cviYUV.encoding = "bgr8"; // "mono8"  "bgr8"
        cviYUV.image = image;
        sensor_msgs::msg::Image imYUV;
        cviYUV.toImageMsg(imYUV);
        publisherLocal->publish(imYUV);
        RCLCPP_INFO(this->get_logger(), "Publishing a yuvMsg: '%s'", std::to_string(countPubLocal++).c_str());
        cout << "Publishing a yuvMsg success in ResizeLocalCallback!" << endl;

        return SUCCESS;
    }
    
    Result ResizeSubmsgCallback(const sensor_msgs::msg::Image::SharedPtr msgCam)
    {
        if (!rclcpp::ok()) {
            ERROR_LOG("rclcpp is not okay!\n");
            return FAILED;
        }
        if (!msgCam) {
            RCLCPP_DEBUG(rclcpp::get_logger("ros2_dvpp2_resize_node"), "Get msgCam failed");
            return FAILED;
        }

        cv_bridge::CvImagePtr cvPtrCam = cv_bridge::toCvCopy(msgCam, "mono8");
        uint32_t msgWidth = cvPtrCam->image.cols;
        uint32_t msgHeight = cvPtrCam->image.rows * 2 / 3;
        uint32_t bufferCam = msgWidth * msgHeight * 3 / 2;
        ImageData notAlignDataCam;

        uint8_t *inputBuf = (uint8_t *)malloc(bufferCam);
        if (inputBuf == nullptr) {
            ERROR_LOG("No memory available!\n");
            return FAILED;
        }
        memset(inputBuf, 0, bufferCam);
        notAlignDataCam.data.reset(inputBuf, [](uint8_t* p) { delete[](p); });

        memcpy(notAlignDataCam.data.get(), cvPtrCam->image.data, bufferCam);
        // Receive YUVmsg and resize
        ImageData inputDataMsg;
        inputDataMsg.width = msgWidth;
        inputDataMsg.height = msgHeight;
        ImageData outputDataMsg;
        outputDataMsg.width = this->get_parameter("outputDataWidth").get_parameter_value().get<int>();
        outputDataMsg.height = this->get_parameter("outputDataHeight").get_parameter_value().get<int>();
        int32_t interpolationMsg = 0;
        uint32_t dstBufferSizeInputDataMsg = dvpp_proc.InitInputDesc(inputDataMsg);

        inputBuf = (uint8_t *)malloc(dstBufferSizeInputDataMsg);
        if (inputBuf == nullptr) {
            ERROR_LOG("No memory available!\n");
            return FAILED;
        }
        memset(inputBuf, 0, dstBufferSizeInputDataMsg);
        inputDataMsg.data.reset(inputBuf, [](uint8_t* p) { delete[](p); });

        uint32_t dstBufferSizeOutputDataMsg  = dvpp_proc.InitOutputDesc(outputDataMsg);
        outputDataMsg.size = dstBufferSizeOutputDataMsg;
        memcpy(inputDataMsg.data.get(), cvPtrCam->image.data, bufferCam);
        uint64_t startResizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        dvpp_proc.Resize(inputDataMsg, outputDataMsg, interpolationMsg);
        uint64_t endResizeTime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        double totalResizeTime = double(endResizeTime - startResizeTime) / 1000;
        std::cout << "Resize submsg success!" << " totalResizeTime: " << totalResizeTime << " ms " << std::endl;
        
        ImageData notAlignDataMsg;

        inputBuf = (uint8_t *)malloc(dstBufferSizeOutputDataMsg);
        if (inputBuf == nullptr) {
            ERROR_LOG("No memory available!\n");
            return FAILED;
        }
        memset(inputBuf, 0, dstBufferSizeOutputDataMsg);
        notAlignDataMsg.data.reset(inputBuf, [](uint8_t* p) { delete[](p); });

        dvpp_proc.GetNotAlignBuffer(outputDataMsg, notAlignDataMsg);

        // 通过ROS2发布缩放后的yuv图片话题
        cv::Mat imageMsg = Yuv2Mat(notAlignDataMsg);
        cv_bridge::CvImage cviYUVMsg;
        cviYUVMsg.header.stamp = this->get_clock()->now();
        cviYUVMsg.header.frame_id = "image_yuv_msg";
        cviYUVMsg.encoding = "bgr8";
        cviYUVMsg.image = imageMsg;
        sensor_msgs::msg::Image imYUVMsg;
        cviYUVMsg.toImageMsg(imYUVMsg);
        publisherSub->publish(imYUVMsg);
        RCLCPP_INFO(this->get_logger(), "Publishing a yuvMsg: '%s'", std::to_string(countPub++).c_str());
        cout << "Publishing a yuvMsg success in ResizeSubmsgCallback!" << endl;

        return SUCCESS;
    }

private:
    rclcpp::TimerBase::SharedPtr timerPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherLocal;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherSub;
    size_t countPubLocal;
    size_t countPub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionImg;
    const sensor_msgs::msg::Image::SharedPtr msgCam;
    bool isAclInitialized = false; // acl系统资源是否已经初始化
    bool isAclDestroyed = false; // acl系统资源是否已经销毁
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("DvppResizeSample"), "This is DvppResizeSample!");
    rclcpp::spin(std::make_shared<DvppResizeSample>());
    rclcpp::shutdown();
    return 0;
}