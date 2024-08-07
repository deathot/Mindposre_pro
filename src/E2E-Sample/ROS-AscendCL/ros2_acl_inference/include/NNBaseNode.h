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
 
#ifndef NN_BASE_NODE_H
#define NN_BASE_NODE_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "AclInterfaceImpl.h"
#include "Utils.h"

namespace AclRos
{
    class NNBaseNode : public rclcpp::Node
    {
    public:
        explicit NNBaseNode(std::string nodeName);
        
        ~NNBaseNode();

        // 执行初始化流程，只做pipeline的串联，具体的每个初始化步骤由用户（派生类中）实现
        Result Init();

        std::shared_ptr<AclInterfaceImpl> aclNodeImpl; // NNBaseNode的实现类

    protected:
        // 模型参数，需要用户去配置
        std::shared_ptr<DnnModelPara> modelPara = nullptr;

        // 执行推理流程，只做pipeline的串联，具体的每个推理步骤由用户（派生类中）实现
        Result Execute(std::vector<DataInfo> inputs, std::vector<DataInfo> &outputs);

        virtual Result SetModelPara() = 0; // 对modelPara赋值
    };
} // namespace AclRos

#endif
