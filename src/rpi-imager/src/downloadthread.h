#ifndef DOWNLOADTHREAD_H
#define DOWNLOADTHREAD_H

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
 * 2022.08.25-Add class DownloadThread function and member variable
 *                 Huawei Technologies Co., Ltd.
 * 
 */


#include <QString>
#include <QThread>
#include <QFile>
#include <QElapsedTimer>
#include <fstream>
#include <atomic>
#include <time.h>
#include <curl/curl.h>
#include "acceleratedcryptographichash.h"
#include "movesectorrsv.h"
#include "config.h"

#ifdef Q_OS_WIN
#include "windows/winfile.h"
#endif
#ifdef Q_OS_DARWIN
#include "mac/macfile.h"
#endif


class DownloadThread : public QThread
{
    Q_OBJECT
public:
    /*
     * Constructor
     *
     * - url: URL to download
     * - localfilename: File name to save downloaded file as. If empty, store data in memory buffer
     */
    explicit DownloadThread(const QByteArray &url, const QByteArray &localfilename = "", const QByteArray &expectedHash = "",
                            bool isExtensible = false, QObject *parent = nullptr);

    /*
     * Destructor
     *
     * Waits until download is complete
     * If this is not desired, call cancelDownload() first
     */
    virtual ~DownloadThread();

    /*
     * Cancel download
     *
     * Async function. Returns immedeately, but can take a second before download actually stops
     */
    virtual void cancelDownload();

    /*
     * Set proxy server.
     * Specify a string like this: user:pass@proxyserver:8080/
     * Used globally, for all connections
     */
    static void setProxy(const QByteArray &proxy);

    /*
     * Returns proxy server used
     */
    static QByteArray proxy();

    /*
     * Set user-agent header string
     */
    void setUserAgent(const QByteArray &ua);

    /*
     * Returns true if download has been successful
     */
    bool successfull();

    /*
     * Returns the downloaded data if saved to memory buffer instead of file
     */
    QByteArray data();

    /*
     * Delete downloaded file
     */
    void deleteDownloadedFile();

    /*
     * Return last-modified date (if available) as unix timestamp
     * (seconds since 1970)
     */
    time_t lastModified();

    /*
     * Return current server time as unix timestamp
     */
    time_t serverTime();

    /*
     * Enable/disable verification
     */
    void setVerifyEnabled(bool verify);

    /*
     * Enable disk cache
     */
    void setCacheFile(const QString &filename, qint64 filesize = 0);

    /*
     * Set input buffer size
     */
    void setInputBufferSize(int len);

    /*
     * Enable image customization
     */
    void setImageCustomization(const QByteArray &config, const QByteArray &cmdline, const QByteArray &firstrun, const QByteArray &cloudinit, const QByteArray &cloudinitNetwork, const QByteArray &initFormat);

    /*
     * Thread safe download progress query functions
     */
    uint64_t dlNow();
    uint64_t dlTotal();
    uint64_t verifyNow();
    uint64_t verifyTotal();
    uint64_t bytesWritten();

    virtual bool isImage();
    size_t _writeFile(const char *buf, size_t len);

signals:
    void success();
    void error(QString msg);
    void cacheFileUpdated(QByteArray sha256);
    void finalizing();
    void preparationStatusUpdate(QString msg);

protected:
    virtual void run();
    virtual void _onDownloadSuccess();
    virtual void _onDownloadError(const QString &msg);

    void _hashData(const char *buf, size_t len);
    void _writeComplete();
    bool _verify();
    int _authopen(const QByteArray &filename);
    bool _openAndPrepareDevice();
    void _writeCache(const char *buf, size_t len);
    qint64 _sectorsWritten();
    void _closeFiles();
    QByteArray _fileGetContentsTrimmed(const QString &filename);
    bool _customizeImage();
    bool _callDiskpart(const QByteArray &data);
    QByteArray _getRandomSign();

    /*
     * libcurl callbacks
     */
    virtual size_t _writeData(const char *buf, size_t len);
    bool _progress(curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);
    void _header(const std::string &header);

    static size_t _curl_write_callback(char *ptr, size_t size, size_t nmemb, void *userdata);
    static int _curl_xferinfo_callback(void *userdata, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);
    static size_t _curl_header_callback( void *ptr, size_t size, size_t nmemb, void *userdata);

    CURL *_c;
    curl_off_t _startOffset;
    std::atomic<std::uint64_t> _lastDlTotal, _lastDlNow, _verifyTotal, _lastVerifyNow, _bytesWritten;
    std::uint64_t _lastFailureOffset;
    qint64 _sectorsStart;
    QByteArray _url, _useragent, _buf, _filename, _lastError, _expectedHash, _config, _cmdline, _firstrun, _cloudinit, _cloudinitNetwork, _initFormat;
    char *_firstBlock;
    size_t _firstBlockSize;
    static QByteArray _proxy;
    static int _curlCount;
    bool _cancelled, _successful, _verifyEnabled, _cacheEnabled, _ejectEnabled;
    time_t _lastModified, _serverTime, _lastFailureTime;
    QElapsedTimer _timer;
    int _inputBufferSize;

    MoveSectorRSV _moveSectorRSV;
    quint64 _fileSize{0};
    quint64 _sectorSize{0};
    bool _isExtensible;
    static std::atomic<bool> _checkRemoteFile, _compliantRemoteFile;

#ifdef Q_OS_WIN
    WinFile _file, _volumeFile;
    QByteArray _nr;
#elif defined(Q_OS_DARWIN)
    MacFile _file;
#else
    QFile _file;
#endif
    QFile _cachefile;

    AcceleratedCryptographicHash _writehash, _verifyhash;
};

#endif // DOWNLOADTHREAD_H
