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

#ifndef ACL_NODE_UTILS_H
#define ACL_NODE_UTILS_H

#include <iostream>
#include <vector>
#include <memory>
#include <sstream>
#include "sensor_msgs/msg/image.hpp"

namespace AclRos
{
    #define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
    #define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
    #define ERROR_LOG(fmt, args...) fprintf(stderr, "[ERROR]   " fmt "\n", ##args)

    #define ALIGN_UP(num, align) (((num) + (align) - 1) & ~((align) - 1))
    #define ALIGN_UP2(num) ALIGN_UP(num, 2)
    #define ALIGN_UP16(num) ALIGN_UP(num, 16)
    #define ALIGN_UP128(num) ALIGN_UP(num, 128)

    typedef enum Result {
        SUCCESS = 0,
        FAILED = 1
    } Result;

    struct Resolution {
        uint32_t width = 0;
        uint32_t height = 0;
    };

    // dvpp要用到的数据结构
    struct ImageData {
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t alignWidth = 0;
        uint32_t alignHeight = 0;
        uint32_t size = 0;
        uint32_t format = 1;
        std::shared_ptr<uint8_t> data;
    };

    struct Rect {
        uint32_t ltX = 0;
        uint32_t ltY = 0;
        uint32_t rbX = 0;
        uint32_t rbY = 0;
    };

    struct BBox {
        Rect rect;
        uint32_t score;
        std::string text;
    };

    /**
     * Utils 主要包含一些常用的工具函数
     */
    class Utils {
    public:
        static Result CvtRosMsgToYuv420sp(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg, ImageData& yuvImage,
                    const std::string encoding, uint32_t modelWidth, uint32_t modelHeight);
    };

    struct DataInfo {
        void* data;
        uint32_t size;
    };

    // 模型参数，用户可对其进行扩展
    struct DnnModelPara {
        std::string modelFile;
        std::string modelName;
    };

} // namespace AclRos

#endif
