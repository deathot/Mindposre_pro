#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import cv2
import numpy as np
import torch
import torchvision

from src.utils.logger import logger_instance as log


def preprocess_image_yolov5(image, cfg, bgr2rgb=True):
    """图片预处理"""
    img, scale_ratio, pad_size = letterbox(image, new_shape=cfg['input_shape'])
    if bgr2rgb:
        img = img[:, :, ::-1]
    img = img.transpose(2, 0, 1)  # HWC2CHW
    img = np.ascontiguousarray(img, dtype=np.float32)
    return img, scale_ratio, pad_size


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=False, scaleFill=False, scaleup=True):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 64), np.mod(dh, 64)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)


def xyxy2xywh(x):
    # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y[:, 0] = (x[:, 0] + x[:, 2]) / 2  # x center
    y[:, 1] = (x[:, 1] + x[:, 3]) / 2  # y center
    y[:, 2] = x[:, 2] - x[:, 0]  # width
    y[:, 3] = x[:, 3] - x[:, 1]  # height
    return y


def non_max_suppression(
        prediction,
        conf_thres=0.25,
        iou_thres=0.45,
        classes=None,
        agnostic=False,
        multi_label=False,
        labels=(),
        max_det=300,
        nm=0,  # number of masks
):
    """Non-Maximum Suppression (NMS) on inference results to reject overlapping detections

    Returns:
         list of detections, on (n,6) tensor per image [xyxy, conf, cls]
    """

    if isinstance(prediction, (list, tuple)):  # YOLOv5 model in validation model, output = (inference_out, loss_out)
        prediction = prediction[0]  # select only inference output

    device = prediction.device
    mps = 'mps' in device.type  # Apple MPS
    if mps:  # MPS not fully supported yet, convert tensors to CPU before NMS
        prediction = prediction.cpu()
    bs = prediction.shape[0]  # batch size
    nc = prediction.shape[2] - nm - 5  # number of classes
    xc = prediction[..., 4] > conf_thres  # candidates

    # Checks
    assert 0 <= conf_thres <= 1, f'Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0'
    assert 0 <= iou_thres <= 1, f'Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0'

    # Settings
    # min_wh = 2  # (pixels) minimum box width and height
    max_wh = 7680  # (pixels) maximum box width and height
    max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
    time_limit = 0.5 + 0.05 * bs  # seconds to quit after
    redundant = True  # require redundant detections
    multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
    merge = False  # use merge-NMS

    t = time.time()
    mi = 5 + nc  # mask start index
    output = [torch.zeros((0, 6 + nm), device=prediction.device)] * bs
    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
        x = x[xc[xi]]  # confidence

        # Cat apriori labels if autolabelling
        if labels and len(labels[xi]):
            lb = labels[xi]
            v = torch.zeros((len(lb), nc + nm + 5), device=x.device)
            v[:, :4] = lb[:, 1:5]  # box
            v[:, 4] = 1.0  # conf
            v[range(len(lb)), lb[:, 0].long() + 5] = 1.0  # cls
            x = torch.cat((x, v), 0)

        # If none remain process next image
        if not x.shape[0]:
            continue

        # Compute conf
        x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box/Mask
        box = xywh2xyxy(x[:, :4])  # center_x, center_y, width, height) to (x1, y1, x2, y2)
        mask = x[:, mi:]  # zero columns if no masks

        # Detections matrix nx6 (xyxy, conf, cls)
        if multi_label:
            i, j = (x[:, 5:mi] > conf_thres).nonzero(as_tuple=False).T
            x = torch.cat((box[i], x[i, 5 + j, None], j[:, None].float(), mask[i]), 1)
        else:  # best class only
            conf, j = x[:, 5:mi].max(1, keepdim=True)
            x = torch.cat((box, conf, j.float(), mask), 1)[conf.view(-1) > conf_thres]

        # Filter by class
        if classes is not None:
            x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

        # Apply finite constraint
        # if not torch.isfinite(x).all():
        #     x = x[torch.isfinite(x).all(1)]

        # Check shape
        n = x.shape[0]  # number of boxes
        if not n:  # no boxes
            continue
        elif n > max_nms:  # excess boxes
            x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence
        else:
            x = x[x[:, 4].argsort(descending=True)]  # sort by confidence

        # Batched NMS
        c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
        boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
        i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
        if i.shape[0] > max_det:  # limit detections
            i = i[:max_det]
        if merge and (1 < n < 3E3):  # Merge NMS (boxes merged using weighted mean)
            # update boxes as boxes(i,4) = weights(i,n) * boxes(n,4)
            iou = box_iou(boxes[i], boxes) > iou_thres  # iou matrix
            weights = iou * scores[None]  # box weights
            x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)  # merged boxes
            if redundant:
                i = i[iou.sum(1) > 1]  # require redundancy

        output[xi] = x[i]
        if mps:
            output[xi] = output[xi].to(device)
        if (time.time() - t) > time_limit:
            log.warning(f'WARNING ⚠️ NMS time limit {time_limit:.3f}s exceeded')
            break  # time limit exceeded

    return output


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0]  # x padding
    coords[:, [1, 3]] -= pad[1]  # y padding
    coords[:, :4] /= gain
    clip_coords(coords, img0_shape)
    return coords


def clip_coords(boxes, shape):
    # Clip bounding xyxy bounding boxes to image shape (height, width)
    if isinstance(boxes, torch.Tensor):  # faster individually
        boxes[:, 0].clamp_(0, shape[1])  # x1
        boxes[:, 1].clamp_(0, shape[0])  # y1
        boxes[:, 2].clamp_(0, shape[1])  # x2
        boxes[:, 3].clamp_(0, shape[0])  # y2
    else:  # np.array (faster grouped)
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, shape[1])  # x1, x2
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, shape[0])  # y1, y2


def nms(box_out, conf_thres=0.4, iou_thres=0.5):
    try:
        boxout = non_max_suppression(box_out, conf_thres=conf_thres, iou_thres=iou_thres, multi_label=True)
    except:
        boxout = non_max_suppression(box_out, conf_thres=conf_thres, iou_thres=iou_thres)
    return boxout


def xyxy_to_xywh(xyxy):
    center_x = (xyxy[0] + xyxy[2]) / 2
    center_y = (xyxy[1] + xyxy[3]) / 2
    w = xyxy[2] - xyxy[0]
    h = xyxy[3] - xyxy[1]
    return (center_x, center_y, w, h)

def plot_one_box(xyxy, img, color=(0, 200, 0), target=False):
    xy1 = (int(xyxy[0]), int(xyxy[1]))
    xy2 = (int(xyxy[2]), int(xyxy[3]))
    if target:
        color = (0, 0, 255)
    cv2.rectangle(img, xy1, xy2, color, 1, cv2.LINE_AA)  # filled


def updata_trace_list(box_center, trace_list, max_list_len=50):
    if len(trace_list) <= max_list_len:
        trace_list.append(box_center)
    else:
        trace_list.pop(0)
        trace_list.append(box_center)
    return trace_list


def draw_trace(img, trace_list):
    """
    更新trace_list,绘制trace
    :param trace_list:
    :param max_list_len:
    :return:
    """
    for i, item in enumerate(trace_list):

        if i < 1:
            continue
        cv2.line(img,
                 (trace_list[i][0], trace_list[i][1]), (trace_list[i - 1][0], trace_list[i - 1][1]),
                 (255, 255, 0), 3)


def cal_iou(box1, box2):
    """

    :param box1: xyxy 左上右下
    :param box2: xyxy
    :return:
    """
    x1min, y1min, x1max, y1max = box1[0], box1[1], box1[2], box1[3]
    x2min, y2min, x2max, y2max = box2[0], box2[1], box2[2], box2[3]
    # 计算两个框的面积
    s1 = (y1max - y1min + 1.) * (x1max - x1min + 1.)
    s2 = (y2max - y2min + 1.) * (x2max - x2min + 1.)

    # 计算相交部分的坐标
    xmin = max(x1min, x2min)
    ymin = max(y1min, y2min)
    xmax = min(x1max, x2max)
    ymax = min(y1max, y2max)

    inter_h = max(ymax - ymin + 1, 0)
    inter_w = max(xmax - xmin + 1, 0)

    intersection = inter_h * inter_w
    union = s1 + s2 - intersection

    # 计算iou
    iou = intersection / union
    return iou

def cal_inter_small(box1, box2):
    x1min, y1min, x1max, y1max = box1[0], box1[1], box1[2], box1[3]
    x2min, y2min, x2max, y2max = box2[0], box2[1], box2[2], box2[3]
    # 计算两个框的面积
    s1 = (y1max - y1min + 1.) * (x1max - x1min + 1.)
    s2 = (y2max - y2min + 1.) * (x2max - x2min + 1.)

    # 计算相交部分的坐标
    xmin = max(x1min, x2min)
    ymin = max(y1min, y2min)
    xmax = min(x1max, x2max)
    ymax = min(y1max, y2max)

    inter_h = max(ymax - ymin + 1, 0)
    inter_w = max(xmax - xmin + 1, 0)

    intersection = inter_h * inter_w

    iou = intersection / min(s1,s2)
    return iou


def cal_distance(box1, box2):
    """
    计算两个box中心点的距离
    :param box1: xyxy 左上右下
    :param box2: xyxy
    :return:
    """
    center1 = ((box1[0] + box1[2]) // 2, (box1[1] + box1[3]) // 2)
    center2 = ((box2[0] + box2[2]) // 2, (box2[1] + box2[3]) // 2)
    dis = ((center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2) ** 0.5

    return dis


def xywh_to_xyxy(xywh):
    x1 = xywh[0] - xywh[2]//2
    y1 = xywh[1] - xywh[3]//2
    x2 = xywh[0] + xywh[2] // 2
    y2 = xywh[1] + xywh[3] // 2

    return [x1, y1, x2, y2]
