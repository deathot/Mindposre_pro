import os
import tarfile
import shutil
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig


def make_targz(output_filename, source_dir):
    with tarfile.open(output_filename, "w:gz") as tar:
        tar.add(source_dir, arcname=os.path.basename(source_dir))


def prepare_calibration_data(infer_proj_path, cali_num=16):
    test_img_dir = os.path.join(infer_proj_path, 'test/images')
    if not os.path.exists(test_img_dir) or not os.listdir(test_img_dir):
        print('[WARNING] Test images do not exist! Skipping calibration data preparation...')
        return
    cali_data_txt = os.path.join(infer_proj_path, 'common/quantize/img_info_amct.txt')
    sample_images = os.listdir(test_img_dir)[:cali_num]
    with open(cali_data_txt, 'w') as f:
        for i, img_name in enumerate(sample_images):
            row_data = f'{i + 1} test/images/{img_name}'
            f.write(row_data + '\n')


if __name__ == "__main__":
    yam_file = '../../../../config/config.yaml'
    config = ModelConfig(yam_file)
    yaml_data = config.get_yaml_data().get('detection')
    output_path = yaml_data['output_path']
    weights = yaml_data['weights']
    onnx_model_path = os.path.join(yaml_data['train_output'], 'weights', 'best.onnx')
    pt_model_path = os.path.join(yaml_data['train_output'], 'weights', 'best.pt')

    infer_project_path = os.path.join(output_path, 'infer_project')
    if os.path.exists(infer_project_path):
        shutil.rmtree(infer_project_path)

    infer_file_path = '../infer'

    shutil.copytree(infer_file_path, infer_project_path)

    config_yaml_path = '../../../../config/config.yaml'
    config_yaml_dest_path = os.path.join(infer_project_path, 'config.yaml')
    shutil.copy(config_yaml_path, config_yaml_dest_path)

    data_yaml_src_path = os.path.join(yaml_data['trans_output'], 'data.yaml')
    data_yaml_dest_path = os.path.join(output_path, 'infer_project', 'data.yaml')
    shutil.copy(data_yaml_src_path, data_yaml_dest_path)

    onnx_model_dest_path = os.path.join(output_path, 'infer_project', f'{weights}.onnx')
    pt_model_dest_path = os.path.join(output_path, 'infer_project', f'{weights}.pt')

    shutil.copy(onnx_model_path, onnx_model_dest_path)
    shutil.copy(pt_model_path, pt_model_dest_path)

    test_dataset_src_path = os.path.join(yaml_data['trans_output'], 'test')
    test_dataset_dest_path = os.path.join(infer_project_path, 'test')
    if os.path.exists(test_dataset_src_path):
        shutil.copytree(test_dataset_src_path, test_dataset_dest_path)
        prepare_calibration_data(infer_project_path)

    package_file_path = os.path.join(output_path, 'infer_project.tar.gz')
    make_targz(package_file_path, infer_project_path)

    if os.path.exists(infer_project_path):
        shutil.rmtree(infer_project_path)
    print(f"process_value=100")
    print(f"INFO: {package_file_path}".replace('\\', '/'))
