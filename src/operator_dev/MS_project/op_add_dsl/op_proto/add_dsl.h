/**
 * Copyright (C)  2020-2021. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @brief
 *
 * @version 1.0
 *
 */

#ifndef GE_OP_ADD_DSL_H
#define GE_OP_ADD_DSL_H
#include "graph/operator_reg.h"
namespace ge
{
    // 为AddDsl算子增加算子原型定义
    REG_OP(AddDsl)
        // 注册第一个输入x1,类型取值范围为Float, int32, int64, float166等等
        .INPUT(x1,
               TensorType({DT_FLOAT, DT_INT32, DT_INT64, DT_FLOAT16, DT_INT16,
                           DT_INT8, DT_UINT8, DT_DOUBLE, DT_COMPLEX128,
                           DT_COMPLEX64, DT_STRING}))
        // 注册第二个输入x2
        .INPUT(x2,
               TensorType({DT_FLOAT, DT_INT32, DT_INT64, DT_FLOAT16, DT_INT16,
                           DT_INT8, DT_UINT8, DT_DOUBLE, DT_COMPLEX128,
                           DT_COMPLEX64, DT_STRING}))
        // 注册输出y
        .OUTPUT(y,
                TensorType({DT_FLOAT, DT_INT32, DT_INT64, DT_FLOAT16, DT_INT16,
                            DT_INT8, DT_UINT8, DT_DOUBLE, DT_COMPLEX128,
                            DT_COMPLEX64, DT_STRING}))
        .OP_END_FACTORY_REG(AddDsl)
}

#endif // GE_OP_ADD_DSL_H
