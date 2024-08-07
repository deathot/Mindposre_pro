/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Raspberry Pi Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * 2022.08.25-Add class Drivelistitem function and member variable
 *                 Huawei Technologies Co., Ltd.
 * 
 */

#include "drivelistitem.h"

DriveListItem::DriveListItem(QString device, QString description, quint64 size, bool isUsb, bool isScsi, bool readOnly, QStringList mountpoints, QObject *parent)
    : QObject(parent), _device(device), _description(description), _mountpoints(mountpoints), _size(size), _isUsb(isUsb), _isScsi(isScsi), _isReadOnly(readOnly)
{

}

int DriveListItem::sizeInGb()
{
    return _size / 1000000000;
}

QStringList DriveListItem::getMountpoints()
{
    return _mountpoints;
}

void DriveListItem::setMountpoints(QStringList &mountpoints)
{
    _mountpoints.clear();
    _mountpoints = mountpoints;
}
