from ais_bench.infer.interface import InferSession

import cv2
import time
import numpy as np

def letterbox(im, new_shape=(320, 320), color=(114, 114, 114), auto=True, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, r, (dw, dh)

class YoloV7:
    def __init__(self,model_path):
        self.sess = None
        self.model_path = model_path

    def infer(self, in_queue,out_queue):
        self.sess = InferSession(0,self.model_path)
        while True:
            while in_queue.empty():
                continue
            frame = in_queue.get()
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            image = img.copy()
            image, ratio, dwdh = letterbox(image, auto=False)
            image = image.transpose((2, 0, 1))
            image = np.expand_dims(image, 0)
            image = np.ascontiguousarray(image)
            im = image.astype(np.float32)

            out = self.sess.infer([im])

            send_item = []
            for item in out[0]:
                bs, x0, y0, x1, y1, cls_id, score = item.tolist()
                if score < 0.6:
                    continue
                box = np.array([x0, y0, x1, y1])
                box -= np.array(dwdh * 2)
                box /= ratio
                box = box.round().astype(np.int32).tolist()
                cls_id = int(cls_id)
                score = float(score)
                send_item.append(box+[cls_id,score])
            out_queue.put(send_item)


