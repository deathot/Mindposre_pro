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

#ifndef DVPP_PROCESS_H
#define DVPP_PROCESS_H

#include "hi_dvpp.h"
#include "acl.h"
#include "Utils.h"

namespace AclRos{

    /*! @class DvppProcess
    *
    * DvppProcess类中主要包含dvpp2中的一些处理流程函数。
    *
    */

    class DvppProcess {
    public:
        /**
         * @brief : DvppProcess类的构造函数
        */
        DvppProcess();

        /**
         * @brief : DvppProcess类的析构函数
        */
        ~DvppProcess();

        /**
         * @brief : 初始化dvpp2系统运行管理资源
        */
        Result InitResource();

        /**
         * @brief : 销毁dvpp2系统运行管理资源
        */
        Result DestroyResource();

        /**
         * @brief : 初始化输入图片
        */
        uint32_t InitInputDesc(ImageData& input_data);

        /**
         * @brief : 初始化输出图片
        */
        uint32_t InitOutputDesc(ImageData& output_data);

        /**
         * @brief : 获取未对齐的原始图片
         * 由于vpc(vision preprocessing core)的要求，图片的输出可能包含冗余的数据，如果不需要这些数据，
         * 可以使用GetNotAlignBuffer来获得真实的输出图片。
         * @param [in] src: 输入图片，基于Utils.h中定义的图片数据结构体
         * @param [in] dest: 输出图片，基于Utils.h中定义的图片数据结构体
        */
        Result GetNotAlignBuffer(ImageData& src, ImageData& dest);

        /**
         * @brief : 对图片进行缩放处理
         * 由于vpc(vision preprocessing core)的要求，图片的输出可能包含冗余的数据，如果不需要这些数据，
         * 可以使用GetNotAlignBuffer来获得真实的输出图片
         * @param [in] src: 输入图片，基于Utils.h中定义的图片数据结构体
         * @param [in] dest: 输出图片，基于Utils.h中定义的图片数据结构体
         * @param [in] interpolation: 缩放算法，默认为0。0：Bilinear算法；1：Nearest neighbor算法
        */
        Result Resize(ImageData& src, ImageData& dest, int32_t interpolation);

    private:
        hi_vpc_chn chnId; // 图片处理通道号，取值范围：[0, 256)，通道总数最多256
        hi_vpc_pic_info inputPic; // VPC功能输入图片，是dvpp2图片数据结构体，包含图像信息字典
        hi_vpc_pic_info outputPic; // VPC功能输出图片，是dvpp2图片数据结构体，包含图像信息字典
        bool isDvppInitialized = false; // dvpp2系统资源是否已经初始化
        bool isDvppDestroyed = false; // dvpp2系统资源是否已经销毁
    };

}

#endif