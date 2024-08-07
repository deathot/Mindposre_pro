from subprocess import run
import argparse
import shutil
import tarfile
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(BASE_DIR)

RETURN_SUCCESS = 0


def parse_args():
    parser = argparse.ArgumentParser("Convert '.pt' model into '.onnx'.")
    parser.add_argument("--weights", required=True, type=str, help="Input train output '.pt' abs file path.",
                        default=r"./best.pt")
    parser.add_argument("--names", required=True, type=str, help="Infer names.txt path.",
                        default=r"./data")
    parser.add_argument("--output_path", required=True, type=str, help="Output tar package file path.",
                        default=r"./")
    parser.add_argument("--data", required=True, type=str, help="Infer data path.",
                        default=r"./data")

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
                    filepath = os.path.join(root, single_file)
                    tar.add(filepath)
    else:
        with tarfile.open(tarfilename, 'w') as tar:
            tar.add(dirname)


def main(weights, data_path, output_path, names):
    cur_path = os.getcwd()
    if not os.path.isfile(weights):
        return f"ERROR: not found weights file from input weights path."
    if not os.path.isfile(names):
        return f"ERROR: not found names.txt."
    if not os.path.isdir(data_path):
        return "ERROR: data   is not a file folder."
    if len(os.listdir(data_path)) == 0:
        return "ERROR: data path is empty."
    if os.path.exists(output_path):
        shutil.rmtree(output_path)
    os.mkdir(output_path)
    # conver pt into onnx
    model_path, file_name = os.path.split(weights)
    model_name = file_name.split('.')[0]
    cmd_1 = f'python export.py --weights {weights}'
    ret = run(cmd_1, shell=False)
    # onnx simplify
    os.chdir(model_path)
    cmd_2 = f'python -m onnxsim {model_path}\\{model_name}.onnx {model_name}_sim.onnx --skip-optimization --no-large-tensor'
    ret.returncode = run(cmd_2, shell=False)
    os.chdir(cur_path)
    # modify slice.
    cmd_3 = f'python modify_yolov5.py {model_path}\\{model_name}_sim.onnx'
    ret = run(cmd_3, shell=False)
    if ret.returncode != RETURN_SUCCESS:
        return ret
    # delete temp file
    os.remove(f'{model_path}\\{model_name}_sim.onnx')
    os.remove(f'{model_path}\\{model_name}.onnx')

    # make tar package.
    shutil.copy(f'{model_path}\\{model_name}_sim_t.onnx', '../../inference/edge_infer')
    shutil.copy(names, '../../inference/edge_infer')
    if os.path.exists(os.path.join(cur_path, '../../inference/edge_infer/data')):
        shutil.rmtree(os.path.join(cur_path, '../../inference/edge_infer/data'))
    shutil.copytree(data_path, '../../inference/edge_infer/data')

    full_path_out = os.path.join(cur_path, '../../inference')
    os.chdir(full_path_out)
    tar_filename = f'edge_infer.tar'
    compress_file(f'{output_path}/{tar_filename}', 'edge_infer')
    os.remove(f'edge_infer/{model_name}_sim_t.onnx')
    os.remove(f'edge_infer/names.txt')
    shutil.rmtree(f'edge_infer/data')
    print(f"process_value=100")
    return f"INFO:Generate edge package edge_infer.tar success."


if __name__ == '__main__':
    args = parse_args()
    try:
        ret = main(args.weights, args.data, args.output_path, args.names)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
