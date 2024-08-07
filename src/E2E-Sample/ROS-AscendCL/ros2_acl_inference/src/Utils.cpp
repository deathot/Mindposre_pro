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

#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <dirent.h>
#include <vector>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <cv_bridge/cv_bridge.h>

#include "Utils.h"

namespace AclRos
{
    Result Utils::CvtRosMsgToYuv420sp(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg, 
                                    ImageData& yuvImage,
                                    const std::string encoding, uint32_t modelWidth, uint32_t modelHeight)
    {
        // imgMsg -> cvImg -> yuvImage
        uint32_t alignWidth;
        uint32_t alignHeight;

        // imgMsg中会出现的编码格式："bgr8" "rgb8"
        if (encoding == "bgr8" || encoding == "rgb8") {
            alignWidth = modelWidth;
            alignHeight = modelHeight;

            cv_bridge::CvImageConstPtr cvImg = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(imgMsg), encoding);

            cv::Mat yuvImg;
            cv::Mat resizedCvImg;
            cv::resize(cvImg->image, resizedCvImg, cv::Size(alignWidth, alignHeight));

            uint32_t buflen = alignWidth * alignHeight * 3/2;
            uint8_t *yuvbuf = new uint8_t[buflen]; 

            cv::cvtColor(resizedCvImg, yuvImg, cv::COLOR_BGR2YUV_IYUV);

            memcpy(yuvbuf, yuvImg.data, buflen * sizeof(uint8_t));

            yuvImage.data.reset(yuvbuf, [](uint8_t* p) { delete[](p); });
            yuvImage.size = buflen;
            yuvImage.width = resizedCvImg.cols;
            yuvImage.height = resizedCvImg.rows;
            yuvImage.alignWidth = alignWidth;
            yuvImage.alignHeight = alignHeight;
            
            if (yuvImage.data == nullptr) {
                ERROR_LOG("Wrong encoding type. Create yuvImage failed.");
                return FAILED;
            }
        } else {
            ERROR_LOG("Wrong encoding type: %s", encoding.c_str());
            return FAILED;
        }

        return SUCCESS;
    }

} // namespace AclRos
