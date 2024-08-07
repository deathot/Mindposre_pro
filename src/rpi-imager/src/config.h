#ifndef CONFIG_H
#define CONFIG_H

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
 * 2022.08.25-Change global configuration
 *                 Huawei Technologies Co., Ltd.
 * 
 */


/* Repository URL */
#define OSLIST_URL                        "https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/images/images_desc/Ascend-devkit-images-desc.json"

#define OSLIST_URL_EN                     "https://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/images/images_desc/Ascend-devkit-images-desc-en.json"

/* Time synchronization URL (only used on eglfs QPA platform, URL must be HTTP) */
#define TIME_URL                          "http://ascend-repo.obs.cn-east-2.myhuaweicloud.com/Atlas%20200I%20DK%20A2/DevKit/images/images_desc/Ascend-devkit-images-desc.json?time_synchronization"

/* Phone home the name of images downloaded for image popularity ranking */
#define TELEMETRY_URL                     "https://rpi-imager-stats.raspberrypi.com/downloads"

/* Hash algorithm for verifying (uncompressed image) checksum */
#define OSLIST_HASH_ALGORITHM             QCryptographicHash::Sha256

/* Hide system drives from list */
#define DRIVELIST_FILTER_SYSTEM_DRIVES    true

/* Update progressbar every 0.1 second */
#define PROGRESS_UPDATE_INTERVAL          100

/* Block size used for writes (currently used when using .zip images only) */
#define IMAGEWRITER_BLOCKSIZE             1*1024*1024

/* Block size used with uncompressed images */
#define IMAGEWRITER_UNCOMPRESSED_BLOCKSIZE 128*1024

/* Block size used when reading during verify stage */
#define IMAGEWRITER_VERIFY_BLOCKSIZE      128*1024

/* Enable caching */
#define IMAGEWRITER_ENABLE_CACHE_DEFAULT        true

/* Do not cache if it would bring free disk space under 5 GB */
#define IMAGEWRITER_MINIMAL_SPACE_FOR_CACHING   5*1024*1024*1024ll

#define IMAGEWRITER_REMOTE_FILE_MINIMUM_SIZE 1024*1024

enum class OperationType
{
    LOCALDOWN = 1,
    REMOTEDOWN = 2,
    READBACK = 3,
    FORMAT = 4,
    CUSTOMURL = 5,
};

#endif // CONFIG_H
