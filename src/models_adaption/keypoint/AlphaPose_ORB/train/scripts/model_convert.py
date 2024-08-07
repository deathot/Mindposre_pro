from subprocess import run
import argparse
import os
import shutil
import tarfile
import torch
import urllib.request
import sys
import json

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig


def parse_args():
    parser = argparse.ArgumentParser("Convert '.pth' model into '.onnx', output tar compress package.")
    parser.add_argument("--cfg", type=str, help="experiment configure file name.",
                        default=r"../configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml")
    parser.add_argument("--weights", type=str, help="Input train output '.pth' abs file path.",
                        default=r"./final_DPG.pth")
    parser.add_argument("--output_path", type=str, help="Output tar package file path.",
                        default=r"../")
    parser.add_argument("--data", type=str, help="Infer data path.",
                        default=r"./data")
    return parser.parse_args()


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


def main(cfg, weights, data_path, output_path):
    if not os.path.isfile(weights):
        return 'ERROR: not found pth weights.'
    if not os.path.isdir(data_path):
        return 'ERROR: data input is not a exist dir.'
    cur_path = os.getcwd()
    # conver pth into onnx
    model_path, _ = os.path.split(weights)
    output_onnx = os.path.join(model_path, "fast_res50_256x192_bs1.onnx")
    # download yolov3_tf.pb
    if not os.path.exists(os.path.join(model_path, "yolov3_tf.pb")):
        with open("../../../../../version.json", "r") as download_urls:
            try:
                url = json.load(download_urls)["model_urls"]["yolov3_tf"]
                urllib.request.urlretrieve(url, model_path + '\\yolov3_tf.pb')
            except Exception as e:
                print(e)
    output_detector = os.path.join(model_path, "yolov3_tf.pb")
    cmd_1 = f'python .\scripts\pthtar2onnx.py --cfg={cfg} --input_pth={weights} --output_onnx={output_onnx}'
    run(cmd_1, shell=False)
    # make tar package.
    shutil.copy(output_onnx, '../infer/models')
    shutil.copy(output_detector, '../infer/models')
    shutil.copytree(data_path, '../infer/data')
    tar_filename = f'edge_infer.tar'

    full_path_out = os.path.join(cur_path, '../')
    os.chdir(full_path_out)
    compress_file(f'{output_path}/{tar_filename}', 'infer')
    os.remove(output_onnx)
    os.remove(output_detector)
    os.remove(f'infer/models/fast_res50_256x192_bs1.onnx')
    os.remove(f'infer/models/yolov3_tf.pb')
    shutil.rmtree('infer/data')
    temp_path = os.path.join(torch.hub.get_dir(), "checkpoints")
    if os.path.exists(temp_path):
        shutil.rmtree(temp_path)
    print("process_value=100")
    return f"INFO: {output_path}/{tar_filename}".replace('\\', '/')


if __name__ == '__main__':
    args = parse_args()
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('keypoint')
    args.cfg = os.path.join(yaml_data['trans_output'], 'data.yaml')
    args.weights = os.path.join(yaml_data['train_output'], 'data.yaml', 'final_DPG.pth')
    args.data = os.path.join(yaml_data['trans_output'])
    args.output_path = yaml_data['output_path']
    try:
        ret = main(args.cfg, args.weights, args.data, args.output_path)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
