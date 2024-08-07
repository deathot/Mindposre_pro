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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "Utils.h"
#include "DvppProcess.h"

namespace AclRos{

    DvppProcess::DvppProcess()
    {
    }

    DvppProcess::~DvppProcess()
    {
    }

    Result DvppProcess::InitResource()
    {
        if (isDvppInitialized) {
            WARN_LOG("DVPP2 resource has been initialized.\n");
            return SUCCESS;
        }
        std::cout << "===========Dvpp2 InitResource()===========" << std::endl;
        int32_t s32Ret = hi_mpi_sys_init();
        if (s32Ret != HI_SUCCESS) {
            ERROR_LOG("hi_mpi_sys_init failed, ret = %#x!\n", s32Ret);
            return FAILED;
        }
        //创建 vpc channel
        hi_vpc_chn_attr stChnAttr {};
        stChnAttr.attr = 0;
        s32Ret = hi_mpi_vpc_sys_create_chn(&chnId, &stChnAttr);
        if (s32Ret != HI_SUCCESS) {
            ERROR_LOG("Call hi_mpi_vpc_sys_create_chn failed, ret = %#x\n", s32Ret);
            hi_mpi_sys_exit();
            return FAILED;
        }

        isDvppInitialized = true;
        return SUCCESS;
    }

    Result DvppProcess::DestroyResource()
    {
        if (isDvppDestroyed) {
            WARN_LOG("DVPP2 resource has been destroyed.\n");
            return SUCCESS;
        }
        std::cout << "===========Dvpp2 DestroyResource()===========" << std::endl;
        // 调用结束，销毁channel.
        int32_t s32Ret = hi_mpi_vpc_destroy_chn(chnId);
        if (s32Ret != HI_SUCCESS) {
            ERROR_LOG("Call hi_mpi_vpc_destroy_chn failed, ret = %#x\n", s32Ret);
            return FAILED;
        }
        // 资源去初始化
        s32Ret = hi_mpi_sys_exit();
        if (s32Ret != HI_SUCCESS) {
            ERROR_LOG("Call hi_mpi_sys_exit failed, ret = %#x\n", s32Ret);
            return FAILED;
        }

        isDvppDestroyed = true;
        return SUCCESS;
    }

    uint32_t DvppProcess::InitInputDesc(ImageData& input_data)
    {
        
        inputPic.picture_width = input_data.width;
        inputPic.picture_height = input_data.height;
        inputPic.picture_format = static_cast<hi_pixel_format>(input_data.format);
        //vpc接口对齐要求为16*2，使用默认参数即可
        uint32_t dstBufferSize = Utils::ConfigureStrideAndBufferSize(inputPic);
        input_data.size = inputPic.picture_buffer_size;
        return dstBufferSize;
    }

    uint32_t DvppProcess::InitOutputDesc(ImageData& output_data)
    {
        outputPic.picture_width = output_data.width;
        outputPic.picture_height = output_data.height;
        outputPic.picture_format = static_cast<hi_pixel_format>(output_data.format); // HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420
        //vpc接口对齐要求为16*2，使用默认参数即可
        uint32_t dstBufferSize = Utils::ConfigureStrideAndBufferSize(outputPic);
        output_data.size = outputPic.picture_buffer_size;
        return dstBufferSize;
    }

    Result DvppProcess::GetNotAlignBuffer(ImageData& src, ImageData& dest)
    {
        hi_vpc_pic_info notAlignPic;
        notAlignPic.picture_width = src.width;
        notAlignPic.picture_height = src.height;
        notAlignPic.picture_format = static_cast<hi_pixel_format>(src.format);

        Utils::ConfigureStrideAndBufferSize(notAlignPic, 1, 1, false);

        notAlignPic.picture_address = src.data.get();

        hi_vpc_pic_info notAlignPicTmp = notAlignPic;

        // 获取缩放后的图片
        Utils::GetDstStridePicture(notAlignPicTmp, notAlignPic);

        dest.width = notAlignPic.picture_width;
        dest.height = notAlignPic.picture_height;
        dest.format = static_cast<uint32_t>(notAlignPic.picture_format);
        dest.data.reset(reinterpret_cast<uint8_t *>(notAlignPic.picture_address), 
        [](uint8_t* p){hi_mpi_dvpp_free((void *)p);});

        return SUCCESS;
    }

Result DvppProcess::Resize(ImageData& src, ImageData& dest, int32_t interpolation = 0)
    {
        int32_t ret = 0;
        // 构造resize输入图片结构体
        inputPic.picture_width = src.width;
        inputPic.picture_height = src.height;
        inputPic.picture_format = static_cast<hi_pixel_format>(src.format);
        // vpc接口对齐要求为16*2，使用默认参数即可
        Utils::ConfigureStrideAndBufferSize(inputPic);

        // 构造resize输出图片结构体
        outputPic.picture_width = dest.width;
        outputPic.picture_height = dest.height;
        outputPic.picture_format = static_cast<hi_pixel_format>(dest.format);

        // image data数据给inputPic
        inputPic.picture_address = src.data.get();
        
        Utils::ConfigureStrideAndBufferSize(outputPic);
        // 创建输出内存空间
        ret = Utils::DvppMemMalloc(&outputPic.picture_address, outputPic.picture_buffer_size);
        if (ret != HI_SUCCESS) {
            ERROR_LOG("Output buffer alloc failed!\n");
            Utils::DvppMemFree(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            return FAILED;
        }
        Utils::MemsetBuffer(outputPic);

        // 调用hi_mpi_vpc_resize接口进行resize处理
        uint32_t taskID = 0;
        ret = hi_mpi_vpc_resize(chnId, &inputPic, &outputPic, 0, 0, interpolation, &taskID, -1);
        if (ret != HI_SUCCESS) {
            ERROR_LOG("hi_mpi_vpc_resize failed, ret = %#x!\n", ret);
            Utils::DvppMemFree(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            Utils::DvppMemFree(outputPic.picture_address);
            outputPic.picture_address = nullptr;
            return FAILED;
        }

        // 等待resize处理完毕
        uint32_t taskIDResult = taskID;
        ret = hi_mpi_vpc_get_process_result(chnId, taskIDResult, -1);
        if (ret != HI_SUCCESS) {
            ERROR_LOG("hi_mpi_vpc_get_process_result failed, ret = %#x!\n", ret);
            Utils::DvppMemFree(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            Utils::DvppMemFree(outputPic.picture_address);
            outputPic.picture_address = nullptr;
            return FAILED;
        }
        // 传回给imagedata
        dest.data.reset(reinterpret_cast<uint8_t *>(outputPic.picture_address), 
        [](uint8_t* p){hi_mpi_dvpp_free((void *)p);});

        return SUCCESS;
    }
}