from subprocess import run
import argparse
import os
import shutil
import tarfile
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig


def parse_args():
    parser = argparse.ArgumentParser("Convert '.pt' model into '.onnx', output tar compress package.")
    parser.add_argument("--weights", required=True, type=str, help="Input train output '.pt' abs file path.",
                        default=r"./best.pt")
    parser.add_argument("--output_path", required=True, type=str, help="Output tar package file path.",
                        default=r"../")
    parser.add_argument("--data", required=True, type=str, help="Infer data path.",
                        default=r"./data")
    parser.add_argument("--num_classes", required=True, type=int, help="num_classes.")

    parser.set_defaults()
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


def main(weights, data_path, output_path, num_classes):
    if not os.path.isfile(weights):
        return 'ERROR: not found pth weights.'
    if not os.path.isdir(data_path):
        return 'ERROR: data input is not a exist dir.'
    if num_classes <= 0:
        return 'ERROR: num_classes should bigger than 0.'
    cur_path = os.getcwd()
    # conver pth into onnx
    model_path, file_name = os.path.split(weights)
    model_name = file_name.split('.')[0]
    cmd_1 = f'python MobileNetV3_pth2onnx.py {weights} {model_path}/{model_name}.onnx {num_classes}'
    run(cmd_1, shell=False)
    # make tar package.
    shutil.copy(f'{model_path}/class_indices.json', '../inference/edge_infer')
    shutil.copy(f'{model_path}/{model_name}.onnx', '../inference/edge_infer')
    shutil.copytree(data_path, '../inference/edge_infer/data')
    tar_filename = f'edge_infer.tar'

    full_path_out = os.path.join(cur_path, '../inference')
    os.chdir(full_path_out)
    compress_file(f'{output_path}/{tar_filename}', 'edge_infer')
    os.remove(f'edge_infer/{model_name}.onnx')
    os.remove(f'edge_infer/class_indices.json')
    shutil.rmtree('../inference/edge_infer/data')
    print("process_value=100")
    return f"INFO: {output_path}/{tar_filename}".replace('\\', '/')


if __name__ == '__main__':
    # args = parse_args()
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('classification')
    weights = os.path.join(yaml_data['train_output'], 'MobileNetV3_large.pth')
    data = os.path.join(yaml_data['trans_output'], 'test')
    output_path = yaml_data['output_path']
    num_classes = yaml_data['class_num']
    try:
        ret = main(weights, data, output_path, num_classes)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
