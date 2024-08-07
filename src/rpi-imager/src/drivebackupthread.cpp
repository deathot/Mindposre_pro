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

#include "drivebackupthread.h"
#include "dependencies/drivelist/src/drivelist.hpp"
#include "config.h"
#include <QDebug>
#include <QFileInfo>
#include <QtConcurrent/QtConcurrent>
#include <archive.h>
#include <archive_entry.h>

DriveBackupThread::DriveBackupThread(const QByteArray &backupPath, const QByteArray &device, bool isCompressed, QObject *parent):
    DownloadThread(backupPath, device, "", false, parent), _deviceSize(0), _readbackCompression(isCompressed)
{

}

DriveBackupThread::~DriveBackupThread()
{
    _destroyResources();
}

void DriveBackupThread::_onError(const QString &msg)
{
    _cancelled = true;
    _destroyResources();
    emit error(msg);
}

void DriveBackupThread::_destroyResources()
{
    if (_cachefile.isOpen())
    {
        _cachefile.close();
    }
    if (_file.isOpen())
    {
        _file.close();
    }
    for (auto _volumeFile : _volumeFileVec)
    {
        _volumeFile->unlockVolume();
    }
    _volumeFileVec.clear();
    if (_readFileBuf)
    {
        qFreeAligned(_readFileBuf);
        _readFileBuf = nullptr;
    }
    if (_archive)
    {
        archive_write_close(_archive);
        archive_write_free(_archive);
        _archive = nullptr;
    }
    if (_entry)
    {
        archive_entry_free(_entry);
        _entry = nullptr;
    }
}

bool DriveBackupThread::_checkFreeSpace(const QString &backupPath)
{
    QByteArray lpcwstrDriver = backupPath.toLatin1().left(2);
    ULARGE_INTEGER liFreeBytesAvailable, liTotalBytes, liTotalFreeBytes;
    if ( !GetDiskFreeSpaceEx( lpcwstrDriver, &liFreeBytesAvailable, &liTotalBytes, &liTotalFreeBytes) )
    {
        qDebug() << "GetDiskFreeSpaceEx failed!";
        _onError(tr("Failed to get disk free space %1.").arg(QString(lpcwstrDriver)));
        return false;
    }
    if (_deviceSize > liFreeBytesAvailable.QuadPart)
    {
        qDebug() << "The free disk space is insufficient!";
        _onError(tr("The free disk space is insufficient."));
        return false;
    }
    return true;
}

bool DriveBackupThread::_openAndPrepareDevice()
{
    emit preparationStatusUpdate(tr("opening drive"));
    _file.setFileName(_filename);
    auto allDevice = Drivelist::ListStorageDevices();
    QByteArray devlower = _filename.toLower();
    for (auto i : allDevice)
    {
        if (QByteArray::fromStdString(i.device).toLower() == devlower)
        {
            _deviceSize = i.size;
            _lastDlTotal = _deviceSize;
            for (auto mountpoint : i.mountpoints)
            {
                QByteArray driveLetter = QByteArray::fromStdString(mountpoint);
                if (driveLetter.endsWith("\\"))
                    driveLetter.chop(1);
                std::shared_ptr<WinFile> volumeFile = std::make_shared<WinFile>();
                volumeFile->setFileName("\\\\.\\"+driveLetter);
                if (volumeFile->open(QIODevice::ReadWrite))
                    volumeFile->lockVolume();
                _volumeFileVec.push_back(volumeFile);
            }
        }
    }
    if (!_file.open(QIODevice::ReadWrite | QIODevice::Unbuffered))
    {
        qDebug() << "Device Open failed!";
        _onError(tr("Cannot open storage device %1.").arg(QString(_filename)));
        return false;
    }
    return true;
}

bool DriveBackupThread::_openAndPrepareBackupFile()
{
    QString backupPath = QUrl(_url).toLocalFile();
    if (!_checkFreeSpace(backupPath))
    {
        return false;
    }

    QFileInfo f(backupPath);
    if (f.exists())
    {
        QFile::remove(backupPath);
    }
    _cachefile.setFileName(backupPath);
    if (_cachefile.open(QIODevice::WriteOnly))
    {
        _cachefile.resize(_deviceSize);
    }
    else
    {
        qDebug() << "Error not open cache file.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }
    return true;
}

bool DriveBackupThread::_openAndPrepareBackupCompressedFile()
{
    QString backupPath = QUrl(_url).toLocalFile();
    if (!_checkFreeSpace(backupPath))
    {
        return false;
    }

    QString fileName = backupPath.section('/', -1);
    if (fileName == backupPath)
    {
        qDebug() << "Error get file name.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }
    _compressedPath = backupPath + ".zip";
    wchar_t savePath[_compressedPath.size() + 1];
    _compressedPath.toWCharArray(savePath);
    savePath[_compressedPath.size()] = '\0';
    qDebug() << "_compressedPath" << _compressedPath;
    _archive = archive_write_new();
    if (_archive == nullptr)
    {
        qDebug() << "call archive_write_new failed.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }
    if (archive_write_set_format_zip(_archive) != ARCHIVE_OK )
    {
        qDebug() << "call archive_write_set_format_zip failed.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }
    if (archive_write_open_filename_w(_archive, savePath) != ARCHIVE_OK )
    {
        qDebug() << "call archive_write_open_filename failed.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }

    _entry = archive_entry_new();
    if (!_entry)
    {
        qDebug() << "call archive_entry_new failed.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }
    archive_entry_set_pathname(_entry, fileName.toLatin1());
    archive_entry_set_perm(_entry, 0777);
    archive_entry_set_filetype(_entry, AE_IFREG);
    archive_entry_set_size(_entry, _deviceSize);
    if (archive_write_header(_archive, _entry) != ARCHIVE_OK )
    {
        qDebug() << "call archive_write_header failed.";
        _onError(tr("Error opening backup file for writing."));
        return false;
    }

    return true;
}

void DriveBackupThread::_readbackRun()
{
    if (!_openAndPrepareBackupFile())
    {
        _destroyResources();
        return;
    }

    emit preparationStatusUpdate("staring backup!");
    _readFileBuf = (char *) qMallocAligned(IMAGEWRITER_VERIFY_BLOCKSIZE, 4096);
    while (_lastDlNow < _deviceSize && !_cancelled)
    {
        qint64 lenRead = _file.read(_readFileBuf, qMin((qint64) IMAGEWRITER_VERIFY_BLOCKSIZE, (qint64) (_deviceSize - _lastDlNow) ));
        if (lenRead == -1)
        {
            _onError(tr("Error reading from storage.<br>"
                        "SD card may be broken."));
            _cachefile.remove();
            return;
        }
        if (_cachefile.write(_readFileBuf, lenRead) != lenRead)
        {
            qDebug() << "Error write backup File!";
            _cachefile.remove();
            _onError(tr("Error writing backup File."));
            return;
        }

        _lastDlNow += lenRead;
        _bytesWritten += lenRead;
    }

    if (_cancelled && _cachefile.isOpen())
    {
        _cachefile.remove();
    }
    emit finalizing();
    _destroyResources();
    _successful = true;
    if (!_cancelled)
    {
        emit success();
    }
}

void DriveBackupThread::_compressAndreadbackRun()
{
    if (!_openAndPrepareBackupCompressedFile())
    {
        _destroyResources();
        return;
    }

    emit preparationStatusUpdate("staring backup!");
    _readFileBuf = (char *) qMallocAligned(IMAGEWRITER_VERIFY_BLOCKSIZE, 4096);
    while (_lastDlNow < _deviceSize && !_cancelled)
    {
        qint64 lenRead = _file.read(_readFileBuf, qMin((qint64) IMAGEWRITER_VERIFY_BLOCKSIZE, (qint64) (_deviceSize - _lastDlNow) ));
        if (lenRead == -1)
        {
            _onError(tr("Error reading from storage.<br>"
                        "SD card may be broken."));
            QFile::remove(_compressedPath);
            return;
        }
        if (archive_write_data(_archive, _readFileBuf, lenRead) != lenRead)
        {
            qDebug() << "Error write backup File!";
            _onError(tr("Error writing backup File."));
            QFile::remove(_compressedPath);
            return;
        }

        _lastDlNow += lenRead;
        _bytesWritten += lenRead;
    }

    emit finalizing();
    _destroyResources();
    _successful = true;
    if (!_cancelled)
    {
        emit success();
    }
    else
    {
        QFile::remove(_compressedPath);
    }
}

void DriveBackupThread::run()
{
#ifndef Q_OS_WIN
    _onError(tr("This function only supports windows."));
    return;
#endif
    if (!_openAndPrepareDevice())
    {
        return;
    }

    if (_readbackCompression)
    {
        _compressAndreadbackRun();
    }
    else
    {
        _readbackRun();
    }
}
