#!/bin/bash
# Copyright(C) 2021. Huawei Technologies Co.,Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set -e
export LD_LIBRARY_PATH=/root/miniconda3/pkgs/python-3.9.2-h3bd6a85_0_cpython/lib:${LD_LIBRARY_PATH}
if [ -f ~/.cache/gstreamer-1.0/registry.aarch64.bin ]; then
  rm ~/.cache/gstreamer-1.0/registry.aarch64.bin
fi
MODE=$1
INPUT=$2
SPEEDTEST=$3


cd src/

out_path="../out/"
if [ -d "$out_path" ]; then
    rm -rf "$out_path"
else
    echo "file $out_path is not exist."
fi

mkdir -p "$out_path"

if [ ${MODE} = "image" ]; then
	python3 image.py ${INPUT}
elif [ ${MODE} = "video" ]; then
    python3 video.py ${SPEEDTEST}
elif [ ${MODE} = "evaluate" ]; then
    python3 evaluate.py ${INPUT}
else
    echo -e "The mode must be image or video or evaluate"
fi

exit 0