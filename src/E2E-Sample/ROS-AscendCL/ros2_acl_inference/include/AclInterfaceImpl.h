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

#ifndef ACL_INTERFACE_IMPL_H
#define ACL_INTERFACE_IMPL_H

#include "acl.h"
#include "Utils.h"

namespace AclRos
{
    class AclInterfaceImpl
    {
    public:        
        explicit AclInterfaceImpl(std::shared_ptr<DnnModelPara> &modelPara);

        ~AclInterfaceImpl();

        // aclInit, aclrtSetDevice, aclrtCreateContext, aclrtCreateStream, aclrtGetRunMode
        Result InitAclResource(); // 初始化ACL资源

        // aclrtFree, aclrtDestroyStream, aclrtDestroyContext, aclrtResetDevice, aclFinalize
        Result DestroyAclResource(); // 销毁ACL资源

        // aclmdlQuerySize, aclrtMalloc, aclmdlLoadFromFileWithMem
        Result LoadModel(); // 加载模型

        // aclmdlCreateDesc, aclmdlGetDesc
        Result CreateDesc(); // 创建模型描述

        // aclmdlCreateDataset, aclCreateDataBuffer, aclmdlAddDatasetBuffer
        Result CreateInput(std::vector<DataInfo> inputs); // 创建模型输入

        // aclmdlGetNumOutputs, aclmdlGetOutputSizeByIndex, aclrtMalloc, aclCreateDataBuffer, aclmdlAddDatasetBuffer
        Result CreateOutput(); // 创建模型输出

        // aclmdlExecute
        Result ModelInference(std::vector<DataInfo> &outputs); // 执行模型推理

        // aclmdlGetDatasetBuffer, aclGetDataBufferAddr, aclGetDataBufferSize, aclrtMemcpy
        void* GetInferenceOuput(uint32_t& itemDataSize, uint32_t idx); // 提取模型输出

        // aclmdlUnload, aclmdlDestroyDesc, aclDestroyDataBuffer, aclmdlDestroyDataset
        Result DestroyModelResource(); // 销毁模型资源

        aclrtStream& GetAclrtStream() { return stream; }

    private:
        // ACL资源相关成员
        uint32_t deviceId;
        aclrtContext context;
        aclrtStream stream;

        // 接收用户派生类modelPara透传进来的模型参数
        std::shared_ptr<DnnModelPara> nodePara = nullptr;

        // DNN模型相关成员
        aclmdlDesc *modelDesc;
        aclmdlDataset *modelInput;
        aclmdlDataset *modelOutput;
        uint32_t modelId;
        size_t modelMemSize; // 模型执行时所需的工作内存的大小
        size_t modelWeightSize; // 模型执行时所需权值内存的大小
        void *modelMemPtr; // 模型所需工作内存（存放模型执行过程中的临时数据）的地址指针
        void *modelWeightPtr; // 模型权值内存（存放权值数据）的地址指针
    };
} // namespace AclRos

#endif
