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
#include <cv_bridge/cv_bridge.h>
#include "Utils.h"

namespace AclRos
{
    Result Utils::ReadPicFile(ImageData& inputPic, const char inputFileName[]){
        FILE* srcFp = std::fopen(inputFileName, "rb");
        if (srcFp == nullptr) {
            ERROR_LOG("fopen %s failed!\n", inputFileName);
            return FAILED;
        }

        uint8_t *inputBuf = (uint8_t *)malloc(inputPic.size);
        if (inputBuf == nullptr) {
            ERROR_LOG("No memory available!\n");
            return FAILED;
        }
        memset(inputBuf, 0, inputPic.size);
        inputPic.data.reset(inputBuf, [](uint8_t* p) { delete[](p); });

        size_t numread = fread(inputPic.data.get(), 1, inputPic.size, srcFp);
        if (numread < inputPic.size) {
            ERROR_LOG("read input data failed, numread = %zu, data size = %u\n", numread, inputPic.size);
            fclose(srcFp);
            srcFp = nullptr;
            DvppMemFree(inputPic.data.get());
            inputPic.data = nullptr;
            return FAILED;
        }

        fclose(srcFp);
        srcFp = nullptr;
        return SUCCESS;
    }

    uint32_t Utils::ConfigureStrideAndBufferSize(hi_vpc_pic_info& pic, uint32_t widthAlign, uint32_t heightAlign, bool widthStride32Align)
    {
        if ((widthAlign == 0) || (widthAlign > 128) || ((widthAlign & (widthAlign - 1)) != 0)) { // 最大128
            ERROR_LOG("widthAlign = %u, should be power of 2, and between (0, 128]!\n", widthAlign);
            return 0;
        }
        if ((heightAlign == 0) || (heightAlign > 128) || ((heightAlign & (heightAlign - 1)) != 0)) { // 最大128
            ERROR_LOG("heightAlign = %u, should be power of 2, and between (0, 128]!\n", heightAlign);
            return 0;
        }

        uint32_t width = pic.picture_width;
        uint32_t height = pic.picture_height;
        uint32_t format = pic.picture_format;
        uint32_t dstBufferSize = 0; // dstBufferSize is the real size
        uint32_t minWidthAlignNum = 32; // min number of width stride is 32

        if (!widthStride32Align) {
            minWidthAlignNum = 1;
        }

        switch (format) {
            case HI_PIXEL_FORMAT_YUV_400:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign);
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                if (pic.picture_width_stride < minWidthAlignNum) {
                    pic.picture_width_stride = minWidthAlignNum;
                }
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
                dstBufferSize = width * height;
                break;
            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign);
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                if (pic.picture_width_stride < minWidthAlignNum) {
                    pic.picture_width_stride = minWidthAlignNum;
                }
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 3 / 2; // 3/2 times
                dstBufferSize = width * height * 3 / 2; // the real buffer size is 3/2 times the product of height and width
                break;
            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign);
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                if (pic.picture_width_stride < minWidthAlignNum) {
                    pic.picture_width_stride = minWidthAlignNum;
                }
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 2; // 2 times
                dstBufferSize = width * height * 2; // the real buffer size is 2 times the product of height and width
                break;
            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign);
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                if (pic.picture_width_stride < minWidthAlignNum) {
                    pic.picture_width_stride = minWidthAlignNum;
                }
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 2; // 2 times
                dstBufferSize = width * height * 2; // the real buffer size is 2 times the product of height and width
                break;
            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign);
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                if (pic.picture_width_stride < minWidthAlignNum) {
                    pic.picture_width_stride = minWidthAlignNum;
                }
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride * 3; // 3 times
                dstBufferSize = width * height * 3; // the real buffer size is 3 times the product of height and width
                break;
            case HI_PIXEL_FORMAT_YUYV_PACKED_422:
            case HI_PIXEL_FORMAT_UYVY_PACKED_422:
            case HI_PIXEL_FORMAT_YVYU_PACKED_422:
            case HI_PIXEL_FORMAT_VYUY_PACKED_422:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign) * 2; // 2 times
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
                dstBufferSize = width * height * 2; // the real buffer size is 2 times the product of height and width
                break;
            case HI_PIXEL_FORMAT_YUV_PACKED_444:
            case HI_PIXEL_FORMAT_RGB_888:
            case HI_PIXEL_FORMAT_BGR_888:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign) * 3; // 3 times
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
                dstBufferSize = width * height * 3; // the real buffer size is 3 times the product of height and width
                break;
            case HI_PIXEL_FORMAT_ARGB_8888:
            case HI_PIXEL_FORMAT_ABGR_8888:
            case HI_PIXEL_FORMAT_RGBA_8888:
            case HI_PIXEL_FORMAT_BGRA_8888:
            case HI_PIXEL_FORMAT_FLOAT32:
                pic.picture_width_stride = ALIGN_UP(width, widthAlign) * 4; // 4 times
                pic.picture_height_stride = ALIGN_UP(height, heightAlign);
                pic.picture_buffer_size = pic.picture_width_stride * pic.picture_height_stride;
                dstBufferSize = width * height * 4; // the real buffer size is 4 times the product of height and width
                break;
            default:
                ERROR_LOG("Format %u is not supported!\n", format);
                pic.picture_buffer_size = 0;
                dstBufferSize = 0;
                break;
        }

        return dstBufferSize;
    }

    Result Utils::DvppMemMalloc(void** addrPtr, uint32_t bufSize)
    {
        int32_t ret = hi_mpi_dvpp_malloc(0, addrPtr, bufSize);
        if (ret != HI_SUCCESS) {
            ERROR_LOG("Malloc buffer failed, size: %u!\n", bufSize);
            return FAILED;
        }
        return SUCCESS;
    }

    Result Utils::DvppMemFree(void* addrPtr)
    {
        int32_t ret = hi_mpi_dvpp_free(addrPtr);
        if (ret != HI_SUCCESS) {
            ERROR_LOG("Free buffer failed, addr = %p!\n", addrPtr);
            return FAILED;
        }
        return SUCCESS;
    }

    Result Utils::MemsetBuffer(hi_vpc_pic_info& picInfo)
    {
        memset(picInfo.picture_address, 0, picInfo.picture_buffer_size);
        return SUCCESS;
    }

    Result Utils::GetDstStridePicture(const hi_vpc_pic_info& srcPic, const hi_vpc_pic_info& dstPic)
    {
        if (srcPic.picture_format != dstPic.picture_format) {
            ERROR_LOG("srcPic.picture_format(%d) should be same as dstPic.picture_format(%d)\n",
                srcPic.picture_format, dstPic.picture_format);
            return FAILED;
        }

        uint8_t* srcBufY = static_cast<uint8_t*>(srcPic.picture_address);
        uint8_t* dstBufY = static_cast<uint8_t*>(dstPic.picture_address);
        uint8_t* srcBufUV = nullptr;
        uint8_t* dstBufUV = nullptr;
        switch (srcPic.picture_format) {
            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420:
                // copy y component
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                // copy uv component
                srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
                dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
                for (uint32_t i = 0; i < srcPic.picture_height / 2; ++i) { // 1/2 of height
                    memcpy(dstBufUV + i * dstPic.picture_width_stride, srcBufUV + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                break;

            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422:
                // copy y component
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                // copy uv component
                srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
                dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufUV + i * dstPic.picture_width_stride, srcBufUV + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                break;

            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440:
                // copy y component
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                // copy uv component
                srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
                dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
                for (uint32_t i = 0; i < srcPic.picture_height / 2; ++i) {
                    memcpy(dstBufUV + i * dstPic.picture_width_stride * 2, srcBufUV + i * srcPic.picture_width_stride * 2,
                        srcPic.picture_width * 2);
                }
                break;

            case HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444:
            case HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444:
                // copy y component
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                // copy uv component
                srcBufUV = srcBufY + srcPic.picture_width_stride * srcPic.picture_height_stride;
                dstBufUV = dstBufY + dstPic.picture_width_stride * dstPic.picture_height_stride;
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufUV + i * dstPic.picture_width_stride * 2,
                        srcBufUV + i * srcPic.picture_width_stride * 2, srcPic.picture_width * 2);
                }
                break;

            case HI_PIXEL_FORMAT_YUYV_PACKED_422:
            case HI_PIXEL_FORMAT_UYVY_PACKED_422:
            case HI_PIXEL_FORMAT_YVYU_PACKED_422:
            case HI_PIXEL_FORMAT_VYUY_PACKED_422:
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width * 2);
                }
                break;

            case HI_PIXEL_FORMAT_YUV_PACKED_444:
            case HI_PIXEL_FORMAT_RGB_888:
            case HI_PIXEL_FORMAT_BGR_888:
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width * 3);
                }
                break;

            case HI_PIXEL_FORMAT_ARGB_8888:
            case HI_PIXEL_FORMAT_ABGR_8888:
            case HI_PIXEL_FORMAT_RGBA_8888:
            case HI_PIXEL_FORMAT_BGRA_8888:
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width * 4);
                }
                break;

            case HI_PIXEL_FORMAT_YUV_400:
                for (uint32_t i = 0; i < srcPic.picture_height; ++i) {
                    memcpy(dstBufY + i * dstPic.picture_width_stride, srcBufY + i * srcPic.picture_width_stride,
                        srcPic.picture_width);
                }
                break;

            default:
                ERROR_LOG("Format %d is not supported!\n", srcPic.picture_format);
                return FAILED;
        }
        return SUCCESS;
    }
}
