# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os
import yaml
import shutil
import argparse
import tarfile
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
TRAIN_DIR = os.path.abspath(__file__)
sys.path.append(BASE_DIR)
sys.path.append(TRAIN_DIR)

import torch

from train.archs import *
from src.models_adaption.config.config import ModelConfig


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--train_out', default='C:/Users/TDTECH/Desktop/run_unetplus_result/train_out')
    parser.add_argument(
        '--trans_out', default='C:/Users/TDTECH/Desktop/run_unetplus_result/trans_out')
    parser.add_argument(
        '--out_dir', default='C:/Users/TDTECH/Desktop/run_unetplus_result/convert_out', help='package out path')
    args = parser.parse_args()

    return args


def compress_file(tarfilename, dirname):
    """
    tarfilename:str 压缩包名字;
    dirname:str 要打包的目录
    """
    if os.path.isdir(os.path.abspath(dirname)):
        with tarfile.open(tarfilename, 'w') as tar:
            for root, dirs, files in os.walk(dirname):
                for single_file in files:
                    # if single_file != tarfilename:
                    filepath = os.path.join(root, single_file)
                    tar.add(filepath)
    else:
        with tarfile.open(tarfilename, 'w') as tar:
            tar.add(dirname)


def check_dir(dire):
    if not os.path.exists(dire):
        os.makedirs(dire)


def main(args):
    # get tmp dir to zip
    tmp_dir = os.path.join(args.out_dir, 'model_convert_tmp')
    if os.path.exists(tmp_dir):
        shutil.rmtree(tmp_dir)
    os.makedirs(tmp_dir, exist_ok=True)

    # get onnx
    f = open(os.path.join(args.train_out, 'config.yml'), encoding='utf-8')
    config = yaml.load(f.read(), Loader=yaml.FullLoader)
    model = NestedUNet(
        num_classes=config['num_classes'], input_channels=3, deep_supervision=False)
    checkpoint = torch.load(os.path.join(
        args.train_out, 'model.pth'), map_location="cpu")
    model.load_state_dict(checkpoint)
    model.eval()
    input_names = ["actual_input_1"]
    output_names = ["output1"]
    dynamic_axes = {'actual_input_1': {0: '-1'}, 'output1': {0: '-1'}}
    dummy_input = torch.randn(1, 3, 96, 96)
    torch.onnx.export(model, dummy_input, os.path.join(tmp_dir, 'model.onnx'), input_names=input_names,
                    dynamic_axes=dynamic_axes, output_names=output_names, opset_version=11)


    # copy test dadtaset and video
    shutil.copytree(os.path.join(args.trans_out, 'val'),
                    os.path.join(tmp_dir, 'sample_data'))

    # copy infer code
    shutil.copy(os.path.join(args.train_out, 'config.yml'), tmp_dir)
    shutil.copytree('infer', os.path.join(tmp_dir, 'infer'))

    # get zip
    tar_filename = 'edge_infer.tar'
    shutil.copytree(tmp_dir, 'edge_infer')
    compress_file(os.path.join(args.out_dir, tar_filename), 'edge_infer')

    # delete copied files
    shutil.rmtree(tmp_dir)
    shutil.rmtree('edge_infer')

    print("process_value=100")
    return f"INFO: {os.path.join(args.out_dir, tar_filename)}".replace('\\', '/')

if __name__ == '__main__':
    args = parse_args()
    
    yaml_file = '../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('segmentation')

    args.trans_out = yaml_data['trans_output']
    args.train_out = yaml_data['train_output']
    args.out_dir = yaml_data['output_path']

    try:
        ret = main(args)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")

