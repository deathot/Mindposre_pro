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
 * 2022.08.25-Add class drivelistmodel function processDriveList change 
 *                 Huawei Technologies Co., Ltd.
 * 
 */

#include "drivelistmodel.h"
#include "config.h"
#include "dependencies/drivelist/src/drivelist.hpp"
#include <QSet>
#include <QDebug>

DriveListModel::DriveListModel(QObject *parent)
    : QAbstractListModel(parent)
{
    _rolenames = {
        {deviceRole, "device"},
        {descriptionRole, "description"},
        {sizeRole, "size"},
        {isUsbRole, "isUsb"},
        {isScsiRole, "isScsi"},
        {isReadOnlyRole, "isReadOnly"},
        {mountpointsRole, "mountpoints"}
    };

    // Enumerate drives in seperate thread, but process results in UI thread
    connect(&_thread, SIGNAL(newDriveList(std::vector<Drivelist::DeviceDescriptor>)), SLOT(processDriveList(std::vector<Drivelist::DeviceDescriptor>)));
}

int DriveListModel::rowCount(const QModelIndex &) const
{
    return _drivelist.count();
}

QHash<int, QByteArray> DriveListModel::roleNames() const
{
    return _rolenames;
}

QVariant DriveListModel::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    if (row < 0 || row >= _drivelist.count())
        return QVariant();

    QByteArray propertyName = _rolenames.value(role);
    if (propertyName.isEmpty())
        return QVariant();
    else
        return _drivelist.values().at(row)->property(propertyName);
}

void DriveListModel::processDriveList(std::vector<Drivelist::DeviceDescriptor> l)
{
    bool changes = false;
    bool filterSystemDrives = DRIVELIST_FILTER_SYSTEM_DRIVES;
    QMap<QString, QStringList> drivesInNewList;
    for (auto &i: l)
    {
        // Convert STL vector<string> to Qt QStringList
        QStringList mountpoints;
        for (auto &s: i.mountpoints)
        {
            mountpoints.append(QString::fromStdString(s));
        }

        if (filterSystemDrives)
        {
            if (i.isSystem)
                continue;
        }
        // Should already be caught by isSystem variable, but just in case...
        if (mountpoints.contains("/") || mountpoints.contains("C://"))
            continue;

        // Skip zero-sized devices
        if (i.size == 0)
            continue;

#ifdef Q_OS_DARWIN
        if (i.isVirtual)
            continue;
#endif

        QString deviceNamePlusSize = QString::fromStdString(i.device)+":"+QString::number(i.size);
        if (i.isReadOnly)
            deviceNamePlusSize += "ro";
        drivesInNewList.insert(deviceNamePlusSize, mountpoints);

        if (!_drivelist.contains(deviceNamePlusSize))
        {
            // Found new drive
            if (!changes)
            {
                beginResetModel();
                changes = true;
            }

            _drivelist[deviceNamePlusSize] = new DriveListItem(QString::fromStdString(i.device), QString::fromStdString(i.description), i.size, i.isUSB, i.isSCSI, i.isReadOnly, mountpoints, this);
        }
        else
        {
            QStringList drivepoints = _drivelist[deviceNamePlusSize]->getMountpoints();
            bool isSame = true;
            if (drivepoints.size() == mountpoints.size())
            {
                for (int idx = 0; idx < drivepoints.size(); ++idx)
                {
                    if (drivepoints.at(idx) != mountpoints.at(idx))
                    {
                        isSame = false;
                        break;
                    }
                }
            }
            else
            {
                isSame = false;
            }

            if (!isSame)
            {
                // update drive
                if (!changes)
                {
                    beginResetModel();
                    changes = true;
                }
                _drivelist[deviceNamePlusSize]->setMountpoints(mountpoints);
            }
        }
    }

    // Look for drives removed
    QStringList drivesInOldList = _drivelist.keys();
    for (auto &device: drivesInOldList)
    {
        if (!drivesInNewList.contains(device))
        {
            if (!changes)
            {
                beginResetModel();
                changes = true;
            }

            _drivelist.value(device)->deleteLater();
            _drivelist.remove(device);
        }
    }

    if (changes)
        endResetModel();
}

void DriveListModel::startPolling()
{
    _thread.start();
}

void DriveListModel::stopPolling()
{
    _thread.stop();
}