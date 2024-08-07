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

#include <chrono>
#include "AclInterfaceImpl.h"

namespace AclRos
{
    AclInterfaceImpl::AclInterfaceImpl(std::shared_ptr<DnnModelPara> &modelPara) 
        : deviceId(0), context(nullptr), stream(nullptr), modelDesc(nullptr), modelInput(nullptr), modelOutput(nullptr), 
          modelId(0), modelMemSize(0), modelWeightSize(0), modelMemPtr(nullptr), modelWeightPtr(nullptr)
    {
        nodePara = modelPara;
    }

    AclInterfaceImpl::~AclInterfaceImpl()
    {
    }

    Result AclInterfaceImpl::InitAclResource()
    {
        // ACL init
        aclError ret = aclInit(nullptr);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("acl init failed");
            return FAILED;
        }
        INFO_LOG("acl init success");

        // open device
        ret = aclrtSetDevice(deviceId);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("acl open device %d failed", deviceId);
            return FAILED;
        }
        INFO_LOG("open device %d success", deviceId);

        // create context
        ret = aclrtCreateContext(&context, deviceId);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("acl create context failed");
            return FAILED;
        }
        INFO_LOG("create context success");

        // create stream
        ret = aclrtCreateStream(&stream);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("acl create stream failed");
            return FAILED;
        }
        INFO_LOG("create stream success");

        return SUCCESS;
    }

    Result AclInterfaceImpl::DestroyAclResource()
    {
        aclError ret;
        if (stream != nullptr) {
            ret = aclrtDestroyStream(stream);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("destroy stream failed");
            }
            stream = nullptr;
        }
        INFO_LOG("end to destroy stream");

        if (context != nullptr) {
            ret = aclrtDestroyContext(context);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("destroy context failed");
            }
            context = nullptr;
        }
        INFO_LOG("end to destroy context");

        ret = aclrtResetDevice(deviceId);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("reset device failed");
        }
        INFO_LOG("end to reset device is %d", deviceId);

        ret = aclFinalize();
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("finalize acl failed");
        }
        INFO_LOG("end to finalize acl");
    }

    Result AclInterfaceImpl::DestroyModelResource()
    {
        // Unload model
        aclError ret = aclmdlUnload(modelId);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("unload model failed, modelId is %u", modelId);
        }

        if (modelDesc != nullptr) {
            aclmdlDestroyDesc(modelDesc);
            modelDesc = nullptr;
        }

        if (modelMemPtr != nullptr) {
            aclrtFree(modelMemPtr);
            modelMemPtr = nullptr;
            modelMemSize = 0;
        }

        if (modelWeightPtr != nullptr) {
            aclrtFree(modelWeightPtr);
            modelWeightPtr = nullptr;
            modelWeightSize = 0;
        }

        INFO_LOG("unload model success, modelId is %u", modelId);

        // DestroyInput
        if (modelInput != nullptr) {
            for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(modelInput); ++i) {
                aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelInput, i);
                aclDestroyDataBuffer(dataBuffer);
            }
            aclmdlDestroyDataset(modelInput);
            modelInput = nullptr;
        }

        // DestroyOutput
        if (modelOutput != nullptr) {
            for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(modelOutput); ++i) {
                aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, i);
                void* data = aclGetDataBufferAddr(dataBuffer);
                aclrtFree(data);
                aclDestroyDataBuffer(dataBuffer);
            }

            aclmdlDestroyDataset(modelOutput);
            modelOutput = nullptr;
        }
    }

    Result AclInterfaceImpl::LoadModel()
    {
        aclError ret = aclmdlQuerySize(nodePara->modelFile.c_str(), &modelMemSize, &modelWeightSize);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("query model failed, model file is %s", nodePara->modelFile);
            return FAILED;
        }

        ret = aclrtMalloc(&modelMemPtr, modelMemSize, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("malloc buffer for mem failed, require size is %zu", modelMemSize);
            return FAILED;
        }

        ret = aclrtMalloc(&modelWeightPtr, modelWeightSize, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("malloc buffer for weight failed, require size is %zu", modelWeightSize);
            return FAILED;
        }

        ret = aclmdlLoadFromFileWithMem(nodePara->modelFile.c_str(), &modelId, modelMemPtr,
            modelMemSize, modelWeightPtr, modelWeightSize);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("load model from file failed, model file is %s", nodePara->modelFile);
            return FAILED;
        }

        return SUCCESS;
    }

    Result AclInterfaceImpl::CreateDesc()
    {
        modelDesc = aclmdlCreateDesc();
        if (modelDesc == nullptr) {
            ERROR_LOG("create model description failed");
            return FAILED;
        }

        aclError ret = aclmdlGetDesc(modelDesc, modelId);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("get model description failed");
            return FAILED;
        }

        INFO_LOG("create model description success");
        return SUCCESS;
    }

    Result AclInterfaceImpl::CreateOutput()
    {
        if (modelDesc == nullptr) {
            ERROR_LOG("no model description, create ouput failed");
            return FAILED;
        }

        modelOutput = aclmdlCreateDataset();
        if (modelOutput == nullptr) {
            ERROR_LOG("can't create dataset, create output failed");
            return FAILED;
        }

        size_t outputSize = aclmdlGetNumOutputs(modelDesc);
        for (size_t i = 0; i < outputSize; ++i) {
            size_t buffer_size = aclmdlGetOutputSizeByIndex(modelDesc, i);

            void *outputBuffer = nullptr;
            aclError ret = aclrtMalloc(&outputBuffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("can't malloc buffer, size is %zu, create output failed", buffer_size);
                return FAILED;
            }

            aclDataBuffer* outputData = aclCreateDataBuffer(outputBuffer, buffer_size);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("can't create data buffer, create output failed");
                aclrtFree(outputBuffer);
                return FAILED;
            }

            ret = aclmdlAddDatasetBuffer(modelOutput, outputData);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("can't add data buffer, create output failed");
                aclrtFree(outputBuffer);
                aclDestroyDataBuffer(outputData);
                return FAILED;
            }
        }

        INFO_LOG("create model output success");
        return SUCCESS;
    }

    Result AclInterfaceImpl::CreateInput(std::vector<DataInfo> inputData) // 由inputData创建模型输入
    {
        modelInput = aclmdlCreateDataset();
        if (modelInput == nullptr) {
            ERROR_LOG("can't create dataset, create input failed");
            return FAILED;
        }

        for (uint32_t i = 0; i < inputData.size(); i++) {
            aclDataBuffer* dataBuf = aclCreateDataBuffer(inputData[i].data, inputData[i].size);
            if (dataBuf == nullptr) {
                ERROR_LOG("can't create data buffer, create input failed");
                return FAILED;
            }

            aclError ret = aclmdlAddDatasetBuffer(modelInput, dataBuf);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("can't add data buffer, create input failed");
                ret = aclDestroyDataBuffer(dataBuf);
                if (ret != ACL_SUCCESS) {
                    ERROR_LOG("Destroy dataset buffer error %d", ret);
                }
                dataBuf = nullptr;
                return FAILED;
            }
        }

        INFO_LOG("create model input success");
        return SUCCESS;
    }

    Result AclInterfaceImpl::ModelInference(std::vector<DataInfo> &outputData)
    {
        std::chrono::steady_clock::time_point inferenceStart = std::chrono::steady_clock::now();
        aclError ret = aclmdlExecute(modelId, modelInput, modelOutput);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("execute model failed, modelId is %u", modelId);
            return FAILED;
        }
        std::chrono::steady_clock::time_point inferenceEnd = std::chrono::steady_clock::now();
        std::cout << "Inference time: " << double(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   inferenceEnd - inferenceStart).count()) << " ms" << std::endl;
        INFO_LOG("model execute success");

        // 将网络模型输出modelOutput填充到outputData
        uint32_t dataSize = 0;
        for (int idx = 0; idx < aclmdlGetNumOutputs(modelDesc); idx++)
        {
            DataInfo batchOutput;
            batchOutput.data = GetInferenceOuput(dataSize, idx);
            batchOutput.size = sizeof(batchOutput.data);
            outputData.push_back(batchOutput);
        }

        return SUCCESS;
    }

    void* AclInterfaceImpl::GetInferenceOuput(uint32_t& itemDataSize, uint32_t idx)
    {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, idx);
        if (dataBuffer == nullptr) {
            ERROR_LOG("Get the %dth dataset buffer from model inference output failed", idx);
            return nullptr;
        }

        void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
        if (dataBufferDev == nullptr) {
            ERROR_LOG("Get the %dth dataset buffer address from model inference output failed", idx);
            return nullptr;
        }

        size_t bufferSize = aclGetDataBufferSize(dataBuffer);
        if (bufferSize == 0) {
            ERROR_LOG("The %dth dataset buffer size of model inference output is 0", idx);
            return nullptr;
        }

        void* data = dataBufferDev;

        itemDataSize = bufferSize;
        return data;
    }
} // namespace AclRos
