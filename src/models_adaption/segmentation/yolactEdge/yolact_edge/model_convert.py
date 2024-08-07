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
    parser.add_argument('--class_names', default="('bear','person','car')",type=str)

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


def main(weights, class_names, data_path, output_path):

    if not os.path.isfile(weights):
        return 'ERROR: not found pth weights.'
    if not os.path.isdir(data_path):
        return 'ERROR: data input is not a exist dir.'
    if len(class_names)<1:
        return 'ERROR: the length class_names should more than 1.'

    # conver pth into onnx
    model_path, file_name = os.path.split(weights)
    model_name = file_name.split('.')[0]
    cmd_1 = f'python pth2onnx.py --trained_model {weights} --onnx_file {model_path}/{model_name}.onnx --class_names {class_names}'
    run(cmd_1, shell=False)

    # 复制文件
    shutil.copy(f'{model_path}/{model_name}.onnx', './')
    shutil.copy(f'{weights}', './')
    shutil.copytree(data_path, './data')
    tar_filename = f'yolact_infer_200DK.tar'

    # 改变到上一级文件夹
    cur_path = os.getcwd()
    full_path_out = os.path.join(cur_path, '../')
    os.chdir(full_path_out)

    # 压缩文件
    compress_file(f'{output_path}/{tar_filename}', 'yolact_edge')

    # 删除生成的文件，释放空间
    os.remove(f'yolact_edge/{model_name}.onnx')
    os.remove(f'yolact_edge/{model_name}.pth')
    shutil.rmtree('yolact_edge/data')
    print("process_value=100")
    return "INFO:Generate edge package yolact_infer_200DK.tar success."


if __name__ == '__main__':
    # args = parse_args()
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('segmentation')
    weights = yaml_data['train_output']
    temp = ','.join(yaml_data['class_names'])
    class_names = f'"{temp}"'
    data = os.path.join(yaml_data['trans_output'], 'valid')
    output_path = yaml_data['output_path']
    try:
        ret = main(weights, class_names, data, output_path)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
