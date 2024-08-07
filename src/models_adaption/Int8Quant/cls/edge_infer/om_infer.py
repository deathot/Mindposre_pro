import os
import numpy as np
# import cv2
import argparse
from acl_resource import AclResource
from acl_model import Model
import time
import json
from PIL import Image
from torchvision import transforms

FILE = os.path.realpath(__file__)
infer_folder = os.path.dirname(FILE)
INPUT_DIR = os.path.join(infer_folder, "data")

IMG_WIDTH = 224
IMG_HEIGHT = 224


def parse_args():
    parser = argparse.ArgumentParser("Infer mobilenetv3.")
    parser.add_argument("--infer_quant", required=False, default=False, type=bool)

    parser.set_defaults()
    return parser.parse_args()


def main(model_path, output_path, label_path):
    # check
    if os.path.exists(f'{output_path}/cls_output.txt'):
        os.remove(f'{output_path}/cls_output.txt')
    
    if not os.path.exists(model_path):
        return ('ERROR: not found om model.')
    if os.path.isfile(f'{output_path}/cls_output.txt'):
        return 'ERROR: output_path already exist.'
    if not os.path.isfile(label_path):
        return ('ERROR: label_path not found, please check.')
    label_dict = {}
    with open(label_path, 'r') as f:
        label_dict = json.load(f)
    acl_resource = AclResource()
    acl_resource.init()
    model = Model(acl_resource, model_path)
    ac = 0
    all = 0
    total_time = 0.0 # Infer only time
    with open(f'{output_path}/cls_output.txt', 'w') as f:
        for pic_dir in os.listdir(INPUT_DIR):
            dir_data = os.listdir(os.path.join(INPUT_DIR, pic_dir))
            print(dir_data)
            dir_data = sorted(dir_data, key=lambda x: int(x.split(".")[1]))
            f.write(f"\n")
            # for pic in os.listdir(os.path.join(INPUT_DIR, pic_dir)):
            for pic in dir_data:
                pic_path = os.path.join(INPUT_DIR, pic_dir, pic)
                pic_name = pic.split('.')[0]
                
                t1 = time.time()
                img_origin = Image.open(pic_path).convert('RGB') # IO takes time a lot
                normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
                trans_list = transforms.Compose([transforms.Resize(256),
                                        transforms.CenterCrop(224),
                                        transforms.ToTensor(),
                                        normalize])
                img = trans_list(img_origin)
                img = img.unsqueeze(0).numpy()

                t2 = time.time()
                output = model.execute([img])
                print("output:", output)
                t3 = time.time()
                
                output0 = output[0]
                result0_array = np.array(output0)
                print("result0_array is: ", result0_array)
                max_idx = result0_array.argmax(axis=1)[0]
                infer_result = label_dict.get(str(max_idx))
                t4 = time.time()

                print("class result : ", infer_result)
                print("pic name: ", pic_name)
                print("pre cost:{:.1f}ms".format((t2 - t1) * 1000))
                print("forward cost:{:.1f}ms".format((t3 - t2) * 1000))
                print("post cost:{:.1f}ms".format((t4 - t3) * 1000))
                print("total cost:{:.1f}ms".format((t4 - t1) * 1000))
                print("FPS:{:.1f}".format(1 / (t4 - t1)))
                # record in txt
                print(f"image name :{pic_path}, infer result: {infer_result}")
                f.write(f"image name :{pic_path}, infer result: {infer_result}\n")
                if pic_dir == infer_result:
                    ac += 1
                all += 1
                print("all is:", all) 
                
                if all != 1:
                    total_time += ((t3 - t2) * 1000)
                    print("total_time is:", total_time, "ms")
                print()
                
        print(f"Execute end. Acc: {ac/all}")
        f.write(f"Execute end. Acc: {ac/all}\n")
        print(f"Total Time Spend: {total_time / (all-1)}ms")
        f.write(f"On Avg, each picture spend: {total_time / (all-1)}ms")
    return "INFO: convert model success!"


if __name__ == '__main__':
    args = parse_args()
    infer_quant = args.infer_quant
    if infer_quant:
        model_path = os.path.join(infer_folder, "mobilenetv3_quant.om")
    else:
        model_path = os.path.join(infer_folder, "mobilenetv3_100_bs1.om")
    label_path = os.path.join(infer_folder, "class_indices.json")
    output_path = os.path.join(infer_folder, "out")
    try:
        ret = main(model_path, output_path, label_path)
        print(ret)
    except Exception as e:
        os.remove(f'{output_path}/cls_output.txt')
        print(e)
        raise Exception(e)
