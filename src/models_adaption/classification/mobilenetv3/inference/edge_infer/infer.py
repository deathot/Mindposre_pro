import os
import numpy as np
import cv2
import argparse
from acl_resource import AclResource
from acl_model import Model
import time
import json
from PIL import Image

INPUT_DIR = './data/'
OUTPUT_DIR = './out_res/'

IMG_WIDTH = 224
IMG_HEIGHT = 224


def parse_args():
    parser = argparse.ArgumentParser("Infer mobilenetv3.")
    parser.add_argument("--model", required=True, type=str, help="Input om model path.",
                        default=r".om")
    parser.add_argument("--label_path", required=True, type=str, help="class_indices.json file path.",
                        default="./label_path")
    parser.add_argument("--output_path", required=False, type=str, help="cls_output.txt path.",
                        default="./out")

    parser.set_defaults()
    return parser.parse_args()


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114)):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    dw, dh = 0.0, 0.0
    new_unpad = (new_shape[1], new_shape[0])
    ratio = (new_shape[1] / shape[1], new_shape[0] / shape[0])  # width, height ratios

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    return img, ratio


def main(model_path, output_path, label_path):
    # check
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
    with open(f'{output_path}/cls_output.txt', 'w') as f:
        for pic_dir in os.listdir(INPUT_DIR):
            for pic in os.listdir(os.path.join(INPUT_DIR, pic_dir)):
                pic_path = os.path.join(INPUT_DIR, pic_dir, pic)
                pic_name = pic.split('.')[0]
                t1 = time.time()
                img_origin = Image.open(pic_path).convert('RGB')
                from torchvision import transforms
                normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
                trans_list = transforms.Compose([transforms.Resize(256),
                                        transforms.CenterCrop(224),
                                        transforms.ToTensor(),
                                        normalize])
                img = trans_list(img_origin)
                img = img.unsqueeze(0).numpy()

                t2 = time.time()

                output = model.execute([img])
                t3 = time.time()
                output0 = output[0]
                result0_array = np.array(output0)
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
    print(f"Execute end. Acc: {ac/all}")
    return "INFO: convert model success!"


if __name__ == '__main__':
    args = parse_args()
    model_path = args.model
    label_path = args.label_path
    output_path = args.output_path
    try:
        ret = main(model_path, output_path, label_path)
        print(ret)
    except Exception as e:
        os.remove(f'{output_path}/cls_output.txt')
        print(e)
        raise Exception(e)
