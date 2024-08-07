# coding=utf-8
#
# Copyright 2022 Huawei Technologies Co., Ltd
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
# ============================================================================

import argparse
import base64
import json
import os
import yaml

import numpy as np
import cv2
import time
from albumentations.augmentations import transforms

import MxpiDataType_pb2 as mxpi_data
from StreamManagerApi import InProtobufVector
from StreamManagerApi import MxProtobufIn
from StreamManagerApi import StreamManagerApi


def save_color_png(img, msk, color):
    msk = msk + 0.5
    msk = cv2.resize(msk, (img.shape[1], img.shape[0]))
    msk = np.array(msk, np.uint8)
    contours, _ = cv2.findContours(
        msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if color == 'red':
        cv2.drawContours(img, contours, -1, (0, 0, 255), 1)
        img[..., 2] = np.where(msk == 1, 255, img[..., 2])
    elif color == 'green':
        cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
        img[..., 1] = np.where(msk == 1, 255, img[..., 1])
    elif color == 'blue':
        cv2.drawContours(img, contours, -1, (255, 0, 0), 1)
        img[..., 0] = np.where(msk == 1, 255, img[..., 0])

    return img


class SDKInferWrapper:
    def __init__(self):
        self._stream_name = None
        self._stream_mgr_api = StreamManagerApi()

        if self._stream_mgr_api.InitManager() != 0:
            raise RuntimeError("Failed to init stream manager.")

    def load_pipeline(self, pipeline_path):
        with open(pipeline_path, 'r') as f:
            pipeline = json.load(f)

        self._stream_name = list(pipeline.keys())[0].encode()
        if self._stream_mgr_api.CreateMultipleStreams(
                json.dumps(pipeline).encode()) != 0:
            raise RuntimeError("Failed to create stream.")

    def do_infer(self, image):
        tensor_pkg_list = mxpi_data.MxpiTensorPackageList()
        tensor_pkg = tensor_pkg_list.tensorPackageVec.add()
        tensor_vec = tensor_pkg.tensorVec.add()
        tensor_vec.deviceId = 0
        tensor_vec.memType = 0

        for dim in [1, *image.shape]:
            tensor_vec.tensorShape.append(dim)

        input_data = image.tobytes()
        tensor_vec.dataStr = input_data
        tensor_vec.tensorDataSize = len(input_data)

        protobuf_vec = InProtobufVector()
        protobuf = MxProtobufIn()
        protobuf.key = b'appsrc0'
        protobuf.type = b'MxTools.MxpiTensorPackageList'
        protobuf.protobuf = tensor_pkg_list.SerializeToString()
        protobuf_vec.push_back(protobuf)

        unique_id = self._stream_mgr_api.SendProtobuf(
            self._stream_name, 0, protobuf_vec)

        if unique_id < 0:
            raise RuntimeError("Failed to send data to stream.")

        infer_result = self._stream_mgr_api.GetResult(
            self._stream_name, unique_id)

        if infer_result.errorCode != 0:
            raise RuntimeError(
                f"GetResult error. errorCode={infer_result.errorCode}, "
                f"errorMsg={infer_result.data.decode()}")
        return infer_result


def _parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video_dir", type=str,
                        default="../../sample_data/video.mp4", help="path of dataset directory")
    parser.add_argument("--pipeline", type=str,
                        default="../config/nested_unet.pipeline", help="path of pipeline file")
    parser.add_argument("--output_dir", type=str,
                        default="./infer_result", help="path of output directory")
    return parser.parse_args()


def _parse_output_data(output_data):
    infer_result_data = json.loads(output_data.data.decode())
    content = json.loads(infer_result_data['metaData'][0]['content'])
    tensor_vec = content['tensorPackageVec'][0]['tensorVec'][0]
    data_str = tensor_vec['dataStr']
    tensor_shape = tensor_vec['tensorShape']
    infer_array = np.frombuffer(base64.b64decode(data_str), dtype=np.float32)
    return infer_array.reshape(tensor_shape)


def sigmoid(x):
    y = x.copy()
    y[x >= 0] = 1.0 / (1 + np.exp(-x[x >= 0]))
    y[x < 0] = np.exp(x[x < 0]) / (1 + np.exp(x[x < 0]))
    return y


def iou_score(output, target):
    smooth = 1e-5

    output_ = output > 0.5
    target_ = target > 0.5
    intersection = (output_ & target_).sum()
    union = (output_ | target_).sum()

    return (intersection + smooth) / (union + smooth)


def check_dir(dir):
    if not os.path.exists(dir):
        os.makedirs(dir, exist_ok=True)


def main():
    args = _parser_args()
    sdk_infer = SDKInferWrapper()
    sdk_infer.load_pipeline(args.pipeline)

    f = open('../../config.yml', encoding='utf-8')
    config = yaml.load(f.read(), Loader=yaml.FullLoader)
    num_class = config['num_classes']

    cap = cv2.VideoCapture(args.video_dir)
    video_name = os.path.basename(args.video_dir)
    video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)

    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    outfile = os.path.join(args.output_dir, video_name)
    writer = cv2.VideoWriter(outfile, fourcc, fps, (video_width, video_height))

    print("Video WRITER:", outfile)

    pre_t = []
    infer_t = []
    post_t = []
    total_t = []
    while True:
        t0 = time.time()
        success, img_bgr = cap.read()
        if not success:  # end
            break

        # preprocess
        image = cv2.resize(img_bgr, (96, 96))
        nor = transforms.Normalize()
        image = nor.apply(image)
        image = image.astype('float32') / 255
        image = image.transpose(2, 0, 1)

        # infer
        t1 = time.time()
        output_data = sdk_infer.do_infer(image)
        output_tensor = _parse_output_data(output_data)

        # postprocess
        t2 = time.time()
        os.makedirs(args.output_dir, exist_ok=True)
        color_list = ['red', 'green', 'blue']
        for i in range(num_class):
            tensor = sigmoid(output_tensor[0][i])

            # get color msk
            img_bgr = save_color_png(img_bgr, tensor, color_list[i])

        # save prediction
        writer.write(img_bgr)
        t3 = time.time()

        pre_t.append(t1-t0)
        infer_t.append(t2-t1)
        post_t.append(t3-t2)
        total_t.append(t3-t0)

    writer.release()

    print('=='*30)
    print('pre_t\t{} ms'.format(np.mean(np.array(pre_t))*1000))
    print('infer_t\t{} ms'.format(np.mean(np.array(infer_t))*1000))
    print('post_t\t{} ms'.format(np.mean(np.array(post_t))*1000))
    print('total_t\t{} ms'.format(np.mean(np.array(total_t))*1000))


if __name__ == "__main__":
    main()
