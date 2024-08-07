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
#ifndef ROS2_DVPP2_RESIZE_UTILS_H
#define ROS2_DVPP2_RESIZE_UTILS_H

#include <iostream>
#include <memory>
#include "hi_dvpp.h"
#include "acl.h"

namespace AclRos
{
    struct ImageData {
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t alignWidth = 0;
        uint32_t alignHeight = 0;
        uint32_t size = 0;
        uint32_t format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420; // 默认图片格式：YUV420SP NV12 8bit
        std::shared_ptr<uint8_t> data;
    };

    #define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
    #define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
    #define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR]  " fmt "\n", ##__VA_ARGS__)

    using Result = uint32_t;
    constexpr Result SUCCESS = 0;
    constexpr Result FAILED = 1;

    /*! @class Utils
    *
    * Utils类中主要包含dvpp2中的一些常用的工具函数。
    * 
    */
    class Utils {
    public:
        /**
         * @brief : 从本地读取图片，并转换为自定义的图片数据结构体
         * @param [in] inputPic: 输入图片，基于Utils.h中定义的图片数据结构体
         * @param [in] inputFileName[]: 输入图片所在文件地址的字符串数组指针
         * @return : 0: SUCCESS; 其他: 错误码和FAILED
        */
        static Result ReadPicFile(ImageData& inputPic, const char inputFileName[]);

        /**
         * @brief : 根据图片的格式和vpc对齐要求，计算图像在内存中的跨距（stride）和内存大小，
                    （宽stride表示一行图像在内存中的列数，高stride表示一列图像在内存中的行数）。
         * @param [in] pic: vpc输入参数结构体，包含：图片宽度、图片高度、图片格式和对齐数量
         * @param [out] pic: vpc输入参数结构体，包含：图片宽度内存跨距、图片高度内存跨距、图片内存大小
         * @return : 实际的内存大小
        */
        static uint32_t ConfigureStrideAndBufferSize(
            hi_vpc_pic_info& pic, uint32_t widthAlign=16, uint32_t heightAlign=2, bool widthStride32Align=true);
        
        /**
         * @brief : 申请Device上的内存
         * @param [in] bufSize: 申请内存的大小，单位Byte
         * @param [out] addrPtr: int，指向“Device上已分配内存的指针”的指针地址
         * @return : 0: SUCCESS; 其他: 错误码和FAILED
        */
        static Result DvppMemMalloc(void** addrPtr, uint32_t bufSize);

        /**
         * @brief : 释放Device上的内存
         * @param [in] addrPtr: int，待释放内存的指针地址
         * @return : 0: SUCCESS; 其他: 错误码和FAILED
        */
        static Result DvppMemFree(void* addrPtr);

        /**
         * @brief : 初始化Device上的内存
         * @param [in] picInfo: vpc输入参数结构体
         * @return : 0: SUCCESS; 其他: 错误码
        */
        static Result MemsetBuffer(hi_vpc_pic_info& picInfo);

        /**
         * @brief : 获取具有期望内存大小的目标图片
         * @param [in] srcPic: 原始图片数据结构体
         * @param [in] dstPic: 目标图片数据结构体
         * @param [out] dstPic: 具有期望内存大小的目标图片
         * @return : 0: SUCCESS; 其他: 错误码和FAILED
        */
        static Result GetDstStridePicture(const hi_vpc_pic_info& srcPic, const hi_vpc_pic_info& dstPic);

    };
}

#endif