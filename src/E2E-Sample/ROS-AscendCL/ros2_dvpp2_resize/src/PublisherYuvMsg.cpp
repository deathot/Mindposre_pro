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

class YUVMsgPublisher : public rclcpp::Node {
    DvppProcess dvpp_proc;
public:
    YUVMsgPublisher() : Node("YUVMsgPublisher"), countPub(0)
    {
        AclInit();
        // dvpp2 initialization
        dvpp_proc.InitResource();
        this->declare_parameter<std::string>("pub_yuv_image_raw", "/yuvimage_raw");
        this->declare_parameter<std::string>("yuvimg_file_name", 
            "/root/ros2_workspace_dvpp/src/ros2_dvpp2_resize/data/dvpp_vpc_1920x1080_nv12.yuv");
        this->declare_parameter<int>("publish_interval", 500);
        this->declare_parameter<int>("inputDataWidth", 1920);
        this->declare_parameter<int>("inputDataHeight", 1080);
        
        std::string pub_yuv_image_raw = this->get_parameter("pub_yuv_image_raw").
            get_parameter_value().get<std::string>();
        int publish_interval = this->get_parameter("publish_interval").get_parameter_value().get<int>();
        // publish the yuv msg
        publisherYUV = this->create_publisher<sensor_msgs::msg::Image>(pub_yuv_image_raw, 10);
        timerPub = this->create_wall_timer(std::chrono::milliseconds(publish_interval),
        std::bind(&YUVMsgPublisher::PublishCallback, this));
    }

    ~YUVMsgPublisher()
    {
        AclDestoryResource();
    }

    cv::Mat Yuv2Mat(ImageData& inputYuvData)
    {
        int w = inputYuvData.width;
        int h = inputYuvData.height;
        int bufLen = w * h * 3 / 2;
        cv::Mat yuvImg;
        yuvImg.create(h * 3 / 2, w, CV_8UC1);
        memcpy(yuvImg.data, inputYuvData.data.get(), bufLen);
        return yuvImg;
    }

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

private:
    Result PublishCallback()
    {
        ImageData inputData; // 输入本地yuv图片的宽高等基本信息
        inputData.width = this->get_parameter("inputDataWidth").get_parameter_value().get<int>();
        inputData.height = this->get_parameter("inputDataHeight").get_parameter_value().get<int>();
        dvpp_proc.InitInputDesc(inputData);
        std::string yuvimg_file_name = this->get_parameter("yuvimg_file_name").get_parameter_value().get<std::string>();
        Utils::ReadPicFile(inputData, yuvimg_file_name.c_str());

        cv::Mat image = Yuv2Mat(inputData);
        cv_bridge::CvImage cviYUV;
        cviYUV.header.stamp = this->get_clock()->now();
        cviYUV.header.frame_id = "image_yuv";
        cviYUV.encoding = "mono8";
        cviYUV.image = image;
        sensor_msgs::msg::Image imYUV;
        cviYUV.toImageMsg(imYUV);
        publisherYUV->publish(imYUV);
        std::string count = std::to_string(countPub++);
        RCLCPP_INFO(this->get_logger(), "Publishing a yuvMsg: '%s'", count.c_str());
        cout << "Only publishing a yuvMsg success!" << endl;

        return SUCCESS;
    }
    rclcpp::TimerBase::SharedPtr timerPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherYUV;
    size_t countPub;
    bool isAclInitialized = false; // acl系统资源是否已经初始化
    bool isAclDestroyed = false; // acl系统资源是否已经销毁
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("YUVMsgPublisher"), "This is YUVMsgPublisher!");
    rclcpp::spin(std::make_shared<YUVMsgPublisher>());
    rclcpp::shutdown();
    return 0;
}