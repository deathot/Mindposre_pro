import sys
import torch
import numpy as np
from utils.BaseDetector import baseDet
from utils.general import non_max_suppression, scale_coords, letterbox
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yolov5.train_code.utils.torch_utils import select_device
from yolov5.train_code.models.experimental import attempt_load


class Detector(baseDet):

    def __init__(self, weights, object_list):
        super(Detector, self).__init__()
        self.weights = weights
        self.object_list = object_list
        self.init_model()
        self.build_config()

    def init_model(self):
        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.device = select_device(self.device)
        model = attempt_load(self.weights, map_location=self.device)
        model.to(self.device).eval()
        model.float()
        # torch.save(model, 'test.pt')
        self.m = model
        self.names = model.module.names if hasattr(
            model, 'module') else model.names

    def preprocess(self, img):

        img0 = img.copy()
        img = letterbox(img, new_shape=self.img_size)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.float()
        img /= 255.0  # 图像归一化
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        return img0, img

    def detect(self, im):

        im0, img = self.preprocess(im)

        pred = self.m(img, augment=False)[0]
        pred = pred.float()
        pred = non_max_suppression(pred, self.threshold, 0.25, names=self.names)

        pred_boxes = []
        for det in pred:

            if det is not None and len(det):
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()

                for *x, conf, cls_id in det:
                    if len(self.names) == 1:
                        lbl = self.names[int(0)]
                    else:
                        lbl = self.names[int(cls_id)]
                    if not lbl in self.object_list:
                        continue
                    x1, y1 = int(x[0]), int(x[1])
                    x2, y2 = int(x[2]), int(x[3])
                    pred_boxes.append(
                        (x1, y1, x2, y2, lbl, conf))

        return im, pred_boxes

