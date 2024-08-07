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
 
#include "NNBaseNode.h"

namespace AclRos
{
    NNBaseNode::NNBaseNode(std::string nodeName) : Node(nodeName)
    {
        modelPara = std::make_shared<DnnModelPara>();
        aclNodeImpl = std::make_shared<AclInterfaceImpl>(modelPara);
    }

    NNBaseNode::~NNBaseNode()
    {
        aclNodeImpl->DestroyModelResource();
        aclNodeImpl->DestroyAclResource();
    }

    Result NNBaseNode::Init()
    {
        SetModelPara(); // 在用户派生类中实现该接口，目的是将模型参数透传进aclNodeImpl
        aclNodeImpl->InitAclResource();
        aclNodeImpl->LoadModel();
        aclNodeImpl->CreateDesc();
        aclNodeImpl->CreateOutput();
    }

    Result NNBaseNode::Execute(std::vector<DataInfo> dnnInputs, std::vector<DataInfo> &dnnOutputs)
    {
        aclNodeImpl->CreateInput(dnnInputs); // 由dnnInputs创建模型输入
        aclNodeImpl->ModelInference(dnnOutputs); // 执行推理，得到dnnOutputs
    }
} // namespace AclRos
