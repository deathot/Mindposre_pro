/**
 * Copyright 2022 Huawei Technologies Co., Ltd

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include "movesectorrsv.h"
#include "config.h"

#include <QDebug>

MoveSectorRSV::MoveSectorRSV()
{
    memset(_magic, 0, 512);
    _magic[0] = 0x55;
    _magic[1] = 0xAA;
    _magic[2] = 0x55;
    _magic[3] = 0xAA;
}

int MoveSectorRSV::WritePartitionHeader(WinFile &driveFile, quint64 componentsMainOffset, quint64 componentsBackupOffset, quint64 sectorSize)
{
    char componentsMain[512] = {0};
    componentsMain[3] = (componentsMainOffset & 0xFF000000) >> 24;
    componentsMain[2] = (componentsMainOffset & 0x00FF0000) >> 16;
    componentsMain[1] = (componentsMainOffset & 0x0000FF00) >> 8;
    componentsMain[0] = (componentsMainOffset & 0x000000FF);
    char componentsBackup[512] = {0};
    componentsBackup[3] = (componentsBackupOffset & 0xFF000000) >> 24;
    componentsBackup[2] = (componentsBackupOffset & 0x00FF0000) >> 16;
    componentsBackup[1] = (componentsBackupOffset & 0x0000FF00) >> 8;
    componentsBackup[0] = (componentsBackupOffset & 0x000000FF);
    for (int i = 0; i < 92; ++i)
    {
        componentsMain[i + 4] = _componentsBase[i];
        componentsBackup[i + 4] = _componentsBase[i];
    }

    quint64 currentReadOffset = 16 * sectorSize;
    driveFile.seek(currentReadOffset);
    if (driveFile.write(_magic, 512) != 512)
    {
        qDebug() << "Write _magic error:" << driveFile.errorString();
        return -1;
    }

    currentReadOffset = (16 + 1) * sectorSize;
    driveFile.seek(currentReadOffset);
    if (driveFile.write(_magic, 512) != 512)
    {
        qDebug() << "Write _magic 2 error:" << driveFile.errorString();
        return -1;
    }

    currentReadOffset = (16 + 2) * sectorSize;
    driveFile.seek(currentReadOffset);
    if (driveFile.write(componentsMain, 512) != 512)
    {
        qDebug() << "Write componentsMain error:" << driveFile.errorString();
        return -1;
    }

    currentReadOffset = (16 + 3) * sectorSize;
    driveFile.seek(currentReadOffset);
    if (driveFile.write(componentsBackup, 512) != 512)
    {
        qDebug() << "Write componentsBackup error:" << driveFile.errorString();
        return -1;
    }

    return 0;
}

int MoveSectorRSV::WriteComponents(WinFile &driveFile, quint64 sectorEnd, quint64 imgSize, quint64 driveSize)
{
    char* readFileBuf = (char *) qMallocAligned(IMAGEWRITER_VERIFY_BLOCKSIZE, 4096);
    qint64 readSize = 0;
    quint64 currentReadOffset = imgSize;
    quint64 currentWriteOffset = driveSize;
    while (currentReadOffset - sectorEnd > 1)
    {
        // 从后往前读取sectorRsv区域数据
        readSize = qMin( (qint64)IMAGEWRITER_VERIFY_BLOCKSIZE, (qint64)(currentReadOffset - sectorEnd - 1) );
        currentReadOffset = currentReadOffset - readSize;
        driveFile.seek(currentReadOffset);
        qint64 lenRead = driveFile.read(readFileBuf, readSize);
        if (lenRead == -1 || lenRead != readSize)
        {
            qDebug() << "MoveSectorRSV::WriteComponents read drive file failed!";
            qFreeAligned(readFileBuf);
            readFileBuf = nullptr;
            return -1;
        }

        // 写入磁盘末尾区域
        currentWriteOffset = currentWriteOffset - readSize;
        driveFile.seek(currentWriteOffset);
        if (driveFile.write(readFileBuf, readSize) != readSize)
        {
            qDebug() << "MoveSectorRSV::WriteComponents Write error:" << driveFile.errorString() << "while writing len:" << readSize;
            qFreeAligned(readFileBuf);
            readFileBuf = nullptr;
            return -1;
        }
    }

    qFreeAligned(readFileBuf);
    readFileBuf = nullptr;

    if (!driveFile.flush())
    {
        qDebug() << "Error writing to storage (while flushing)";
        return -1;
    } 
    return 0;
}

int MoveSectorRSV::MovingSectorRSV(WinFile &driveFile, quint64 imgSize, quint64 driveSize, quint64 sectorSize)
{
    if (imgSize >= driveSize)
    {
        return 0;
    }

    const quint64 sectorRsv = DEFAULT_SECTOR_SIZE;
    const quint64 sectorEnd = imgSize - sectorRsv - 1;

    // 新地址
    quint64 new_sectorEnd = driveSize - sectorRsv - 1;
    quint64 NEW_COMPONENTS_MAIN_OFFSET = (driveSize - sectorRsv) / sectorSize;
    quint64 NEW_COMPONENTS_BACKUP_OFFSET = NEW_COMPONENTS_MAIN_OFFSET + 73728;

    // 写入头部地址
    if (WritePartitionHeader(driveFile, NEW_COMPONENTS_MAIN_OFFSET, NEW_COMPONENTS_BACKUP_OFFSET, sectorSize) != 0)
    {
        return -1;
    }
    // 移动sectorRsv
    if (WriteComponents(driveFile, sectorEnd, imgSize, driveSize) != 0)
    {
        return -1;
    }

    return 0;
}
