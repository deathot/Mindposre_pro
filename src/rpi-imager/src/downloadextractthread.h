#ifndef DOWNLOADEXTRACTTHREAD_H
#define DOWNLOADEXTRACTTHREAD_H

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
 * 2022.08.25-Changed modify the constructor DownloadExtractThread
 *                 Huawei Technologies Co., Ltd.
 * 
 */


#include "downloadthread.h"
#include <deque>
#include <condition_variable>
#include <QtConcurrent/QtConcurrent>

class _extractThreadClass;

class DownloadExtractThread : public DownloadThread
{
    Q_OBJECT
public:
    /*
     * Constructor
     *
     * - url: URL to download
     * - localfolder: Folder to extract archive to
     */
    explicit DownloadExtractThread(const QByteArray &url, const QByteArray &localfilename = "", const QByteArray &expectedHash = "",
                                   bool isExtensible = false, QObject *parent = nullptr);

    virtual ~DownloadExtractThread();
    virtual void cancelDownload();
    virtual void extractImageRun();
    virtual void extractMultiFileRun();
    virtual bool isImage();
    virtual void enableMultipleFileExtraction();

protected:
    char *_abuf[2];
    size_t _abufsize;
    _extractThreadClass *_extractThread;
    std::deque<QByteArray> _queue;
    static const int MAX_QUEUE_SIZE;
    std::mutex _queueMutex;
    std::condition_variable _cv;
    bool _ethreadStarted, _isImage;
    AcceleratedCryptographicHash _inputHash;
    int _activeBuf;
    bool _writeThreadStarted;
    QFuture<size_t> _writeFuture;

    QByteArray _popQueue();
    void _pushQueue(const char *data, size_t len);
    void _cancelExtract();
    virtual size_t _writeData(const char *buf, size_t len);
    virtual void _onDownloadSuccess();
    virtual void _onDownloadError(const QString &msg);

    virtual ssize_t _on_read(struct archive *a, const void **buff);
    virtual int _on_close(struct archive *a);

    static ssize_t _archive_read(struct archive *a, void *client_data, const void **buff);
    static int _archive_close(struct archive *a, void *client_data);
};

#endif // DOWNLOADEXTRACTTHREAD_H
