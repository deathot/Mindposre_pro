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
 * 2022.08.25-Changed modify the constructor LocalFileExtractThread
 *                 Huawei Technologies Co., Ltd.
 * 
 */

#include "localfileextractthread.h"
#include "config.h"

LocalFileExtractThread::LocalFileExtractThread(const QByteArray &url, const QByteArray &dst, const QByteArray &expectedHash, bool isExtensible, QObject *parent)
    : DownloadExtractThread(url, dst, expectedHash, isExtensible, parent)
{
    _inputBuf = (char *) qMallocAligned(IMAGEWRITER_UNCOMPRESSED_BLOCKSIZE, 4096);
}

LocalFileExtractThread::~LocalFileExtractThread()
{
    _cancelled = true;
    wait();
    qFreeAligned(_inputBuf);
}

void LocalFileExtractThread::_cancelExtract()
{
    _cancelled = true;
    if (_inputfile.isOpen())
        _inputfile.close();
}

void LocalFileExtractThread::run()
{
    if (isImage() && !_openAndPrepareDevice())
        return;

    emit preparationStatusUpdate(tr("opening image file"));
    _timer.start();
    _inputfile.setFileName( QUrl(_url).toLocalFile() );
    if (!_inputfile.open(_inputfile.ReadOnly))
    {
        _onDownloadError(tr("Error opening image file"));
        _closeFiles();
        return;
    }
    _lastDlTotal = _inputfile.size();

    if (isImage())
        extractImageRun();
    else
        extractMultiFileRun();

    if (_cancelled)
        _closeFiles();
}

ssize_t LocalFileExtractThread::_on_read(struct archive *, const void **buff)
{
    if (_cancelled)
        return -1;

    *buff = _inputBuf;
    ssize_t len = _inputfile.read(_inputBuf, IMAGEWRITER_UNCOMPRESSED_BLOCKSIZE);

    if (len > 0)
    {
        _lastDlNow += len;
        if (!_isImage)
        {
            _inputHash.addData(_inputBuf, len);
        }
    }

    return len;
}

int LocalFileExtractThread::_on_close(struct archive *)
{
    _inputfile.close();
    return 0;
}
