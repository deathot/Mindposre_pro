#ifndef IMAGEWRITER_H
#define IMAGEWRITER_H

/**
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
 * 2022.08.25-Changed class ImageWriter function and member variable 
 *                 Huawei Technologies Co., Ltd.
 * 
*/
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QFileInfo>
#include <QFile>
#include <QObject>
#include <QTimer>
#include <QUrl>
#include <QSettings>
#include <QVariant>
#include "config.h"
#include "powersaveblocker.h"
#include "drivelistmodel.h"

class QQmlApplicationEngine;
class DownloadThread;
class QNetworkReply;
class QWinTaskbarButton;
class QTranslator;

class ImageWriter : public QObject
{
    Q_OBJECT
public:
    explicit ImageWriter(QObject *parent = nullptr);
    virtual ~ImageWriter();
    void setEngine(QQmlApplicationEngine *engine);

    /* Set URL to download from, and if known download length and uncompressed length */
    Q_INVOKABLE void setSrc(const QUrl &url, int operateType, quint64 downloadLen = 0, quint64 extrLen = 0, QByteArray expectedHash = "", bool multifilesinzip = false,
                            QString parentcategory = "", QString osname = "", QByteArray initFormat = "", bool isExtensible = false);

    /* Get operation type */
    Q_INVOKABLE int getOperation();

    // 下载
    Q_INVOKABLE void httpDownLoad(const QString &url);

    // 验证文件是否存在
    Q_INVOKABLE bool fileIsExist(QString filename);

    // 写入ip配置
    Q_INVOKABLE bool writeIpInfo(QString filePath, QString ipData);

    // 在qml中调用，实现暂停
    Q_INVOKABLE void waitTime(qint16 time=1);

    // 设置翻译
    Q_INVOKABLE void sendLanguage(QString msg);

    /* Set device to write to */
    Q_INVOKABLE void setDst(const QString &device, quint64 deviceSize = 0);

    /* Enable/disable verification */
    Q_INVOKABLE void setVerifyEnabled(bool verify);

    /* Returns true if src and dst are set */
    Q_INVOKABLE bool readyToWrite();

    /* Start writing */
    Q_INVOKABLE void startWrite();

    /* Cancel write */
    Q_INVOKABLE void cancelWrite();

    /* Return true if url is in our local disk cache */
    Q_INVOKABLE bool isCached(const QUrl &url, const QByteArray &sha256);

    /* Start polling the list of available drives */
    Q_INVOKABLE void startDriveListPolling();

    /* Stop polling the list of available drives */
    Q_INVOKABLE void stopDriveListPolling();

    /* Return list of available drives. Call startDriveListPolling() first */
    DriveListModel *getDriveList();

    /* Utility function to return filename part from URL */
    Q_INVOKABLE QString fileNameFromUrl(const QUrl &url);

    /* Function to return OS list URL */
    Q_INVOKABLE QUrl constantOsListUrl() const;

    /* Function to return version */
    Q_INVOKABLE QString constantVersion() const;

    /* Returns true if version argument is newer than current program */
    Q_INVOKABLE bool isVersionNewer(const QString &version);

    /* Set custom repository */
    Q_INVOKABLE void setCustomOsListUrl(const QUrl &url);

    /* Utility function to open OS file dialog */
    Q_INVOKABLE void openFileDialog();

    /* Return filename part of URL set */
    Q_INVOKABLE QString srcFileName();

    /* Returns true if online */
    Q_INVOKABLE bool isOnline();

    /* Returns true if run on embedded Linux platform */
    Q_INVOKABLE bool isEmbeddedMode();

    /* Mount any USB sticks that can contain source images under /media
       Returns true if at least one device was mounted */
    Q_INVOKABLE bool mountUsbSourceMedia();

    /* Returns a json formatted list of the OS images found on USB stick */
    Q_INVOKABLE QByteArray getUsbSourceOSlist();

    /* Get operation type */
    Q_INVOKABLE void setReadbackCompression(bool isCompressed);

    /* Functions to collect information from computer running imager to make image customization easier */
    Q_INVOKABLE QString getDefaultPubKey();
    Q_INVOKABLE QString getTimezone();
    Q_INVOKABLE QStringList getTimezoneList();
    Q_INVOKABLE QStringList getCountryList();
    Q_INVOKABLE QStringList getKeymapLayoutList();
    Q_INVOKABLE QString getSSID();
    Q_INVOKABLE QString getPSK(const QString &ssid);

    Q_INVOKABLE bool getBoolSetting(const QString &key);
    Q_INVOKABLE void setSetting(const QString &key, const QVariant &value);
    Q_INVOKABLE void setImageCustomization(const QByteArray &config, const QByteArray &cmdline, const QByteArray &firstrun, const QByteArray &cloudinit, const QByteArray &cloudinitNetwork);
    Q_INVOKABLE void setSavedCustomizationSettings(const QVariantMap &map);
    Q_INVOKABLE QVariantMap getSavedCustomizationSettings();
    Q_INVOKABLE void clearSavedCustomizationSettings();
    Q_INVOKABLE bool hasSavedCustomizationSettings();
    Q_INVOKABLE bool imageSupportsCustomization();

    Q_INVOKABLE QString crypt(const QByteArray &password);
    Q_INVOKABLE QString pbkdf2(const QByteArray &psk, const QByteArray &ssid);

    Q_INVOKABLE QStringList getTranslations();
    Q_INVOKABLE QString getCurrentLanguage();
    Q_INVOKABLE QString getCurrentKeyboard();
    Q_INVOKABLE void changeLanguage(const QString &newLanguageName);
    Q_INVOKABLE void changeKeyboard(const QString &newKeymapLayout);
    Q_INVOKABLE bool customRepo();

    void replaceTranslator(QTranslator *trans);
    QString detectPiKeyboard();
    Q_INVOKABLE bool hasMouse();

signals:
    /* We are emiting signals with QVariant as parameters because QML likes it that way */
    void downUpdateexeProgress(QVariant dlnow, QVariant dltotal);
    void downloadProgress(QVariant dlnow, QVariant dltotal);
    void verifyProgress(QVariant now, QVariant total);
    void error(QVariant msg);
    void info(QVariant msg);
    void success();
    void fileSelected(QVariant filename);
    void cancelled();
    void finalizing();
    void networkOnline();
    void preparationStatusUpdate(QVariant msg);

protected slots:

    void startProgressPolling();
    void stopProgressPolling();
    void pollProgress();
    void pollNetwork();
    void syncTime();
    void onSuccess();
    void onError(QString msg);
    void onFileSelected(QString filename);
    void onCancelled();
    void onCacheFileUpdated(QByteArray sha256);
    void onFinalizing();
    void onTimeSyncReply(QNetworkReply *reply);
    void onPreparationStatusUpdate(QString msg);
    void onFinished();
    void onReadyRead();
    void onDownloadProgress(qint64 bytesRead,qint64 totalBytes);

protected:
    QUrl _src, _repo;
    QString _dst, _cacheFileName, _parentCategory, _osName, _currentLang, _currentLangcode, _currentKeyboard;
    QByteArray _expectedHash, _cachedFileHash, _cmdline, _config, _firstrun, _cloudinit, _cloudinitNetwork, _initFormat;
    quint64 _downloadLen, _extrLen, _devLen, _dlnow, _verifynow;
    DriveListModel _drivelist;
    QQmlApplicationEngine *_engine;
    QTimer _polltimer, _networkchecktimer;
    PowerSaveBlocker _powersave;
    DownloadThread *_thread;
    bool _verifyEnabled, _multipleFilesInZip, _cachingEnabled, _embeddedMode, _online, _isExtensible;
    QSettings _settings;
    QMap<QString,QString> _translations;
    QTranslator *_trans;
    OperationType _operateType{OperationType::LOCALDOWN};
    bool _readbackCompression{true};
private:
    QNetworkAccessManager networkManager; //网络管理
    QNetworkReply *reply; //网络响应
    QFile *downloadedFile; //下载保存的临时文件
#ifdef Q_OS_WIN
    QWinTaskbarButton *_taskbarButton;
#endif

    void _parseCompressedFile();
};

#endif // IMAGEWRITER_H
