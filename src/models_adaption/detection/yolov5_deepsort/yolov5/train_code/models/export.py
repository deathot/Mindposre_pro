"""Exports a YOLOv5 *.pt model to ONNX and TorchScript formats

Usage:
    $ export PYTHONPATH="$PWD" && python models/export.py --weights ./weights/yolov5s.pt --img 640 --batch 1
"""

import argparse
import os
import sys
BASE_DIR1 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE_DIR2 = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(BASE_DIR1)
sys.path.append(BASE_DIR2)
from models.common import *
from yolov5.train_code.utils.torch_utils import *

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='./yolov5s.pt', help='weights path')
    parser.add_argument('--img_size', nargs='+', type=int, default=[640, 640], help='image size')
    parser.add_argument('--batch_size', type=int, default=1, help='batch size')
    opt = parser.parse_args()
    # params check
    if not os.path.isfile(opt.weights):
        return 'ERROR: weights not exist.'
    if opt.batch_size <= 0:
        return 'ERROR: batch size should bigger than 0.'

    opt.img_size *= 2 if len(opt.img_size) == 1 else 1  # expand
    print(opt)

    # Input
    img = torch.zeros((opt.batch_size, 3, *opt.img_size))  # image size(1,3,320,192) iDetection

    # Load PyTorch model
    model = torch.load(opt.weights, map_location=torch.device('cpu'))['model'].float()

    model.eval()
    try:
        model.model[-1].export = True  # set Detect() layer export=True
    except AttributeError:
        pass
    y = model(img)  # dry run

    # ONNX export
    try:
        import onnx

        print('\nStarting ONNX export with onnx %s...' % onnx.__version__)
        f = opt.weights.replace('.pt', '.onnx')  # filename
        # model.fuse()  # only for ONNX
        torch.onnx.export(model, img, f, verbose=False, opset_version=11, input_names=['images'],
                          do_constant_folding=True,
                          output_names=['classes', 'boxes'] if y is None else ['output'])

        # Checks
        onnx_model = onnx.load(f)  # load onnx model
        onnx.checker.check_model(onnx_model)  # check onnx model
        print(onnx.helper.printable_graph(onnx_model.graph))  # print a human readable model
        print('ONNX export success, saved as %s' % f)
    except Exception as e:
        print('ONNX export failure: %s' % e)

    # Finish
    print('\nExport complete. Visualize with https://github.com/lutzroeder/netron.')


if __name__ == '__main__':
    main()

