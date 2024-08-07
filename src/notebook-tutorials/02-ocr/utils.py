import os
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Config:
    text_proposals_min_scores: float = 0.7
    text_proposals_nms_thresh: float = 0.1
    line_min_score: float = 0.9
    min_ratio: float = 0.5
    text_proposals_width: int = 16
    min_num_proposals: int = 2
    max_horizontal_gap: int = 60
    min_v_overlaps: float = 0.7
    min_size_sim: int = 0.7


config = Config()


def get_images_from_path(img_path):
    img_list = []
    if os.path.isfile(img_path):
        img_list.append(img_path)
    if os.path.isdir(img_path):
        for file in os.listdir(img_path):
            img_list.append(os.path.join(img_path, file))
    return img_list


def img_read(path):
    path = os.path.realpath(path)
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"cannot decode file:{path}")
    return img


def filter_proposal(proposals, scores):
    """
    Filter text proposals

    Args:
        proposals(numpy.array): Text proposals.
    Returns:
        proposals(numpy.array): Text proposals after filter.
    """
    inds = np.where(scores > config.text_proposals_min_scores)[0]
    keep_proposals = proposals[inds]
    keep_scores = scores[inds]
    sorted_inds = np.argsort(keep_scores.ravel())[::-1]
    keep_proposals, keep_scores = keep_proposals[sorted_inds], keep_scores[sorted_inds]
    nms_inds = nms(np.hstack((keep_proposals, keep_scores)), config.text_proposals_nms_thresh)
    keep_proposals, keep_scores = keep_proposals[nms_inds], keep_scores[nms_inds]
    return keep_proposals, keep_scores


def filter_boxes(boxes):
    """
    Filter text boxes

    Args:
        boxes(numpy.array): Text boxes.
    Returns:
        boxes(numpy.array): Text boxes after filter.
    """
    heights = np.zeros((len(boxes), 1), np.float)
    widths = np.zeros((len(boxes), 1), np.float)
    scores = np.zeros((len(boxes), 1), np.float)
    index = 0
    for box in boxes:
        widths[index] = abs(box[2] - box[0])
        heights[index] = abs(box[3] - box[1])
        scores[index] = abs(box[4])
        index += 1
    return np.where((widths / heights > config.min_ratio) & (scores > config.line_min_score) & \
                    (widths > (config.text_proposals_width * config.min_num_proposals)))[0]


def connect_text_lines(text_proposals, scores, size):
    """
    Connect text lines

    Args:
        text_proposals(numpy.array): Predict text proposals.
        scores(numpy.array): Bbox predicts scores.
        size(numpy.array): Image size.
    Returns:
        text_recs(numpy.array): Text boxes after connect.
    """
    graph = get_successions(text_proposals, scores, size)
    text_lines = np.zeros((len(graph), 5), np.float32)
    for index, indices in enumerate(graph):
        text_line_boxes = text_proposals[list(indices)]
        x0 = np.min(text_line_boxes[:, 0])
        x1 = np.max(text_line_boxes[:, 2])

        offset = (text_line_boxes[0, 2] - text_line_boxes[0, 0]) * 0.5

        lt_y, rt_y = fit_y(text_line_boxes[:, 0], text_line_boxes[:, 1], x0 + offset, x1 - offset)
        lb_y, rb_y = fit_y(text_line_boxes[:, 0], text_line_boxes[:, 3], x0 + offset, x1 - offset)

        # the score of a text line is the average score of the scores
        # of all text proposals contained in the text line
        score = scores[list(indices)].sum() / float(len(indices))

        text_lines[index, 0] = x0
        text_lines[index, 1] = min(lt_y, rt_y)
        text_lines[index, 2] = x1
        text_lines[index, 3] = max(lb_y, rb_y)
        text_lines[index, 4] = score

    text_lines = clip_boxes(text_lines, size)

    text_recs = np.zeros((len(text_lines), 9), np.float)
    index = 0
    for line in text_lines:
        xmin, ymin, xmax, ymax = line[0], line[1], line[2], line[3]
        text_recs[index, 0] = xmin
        text_recs[index, 1] = ymin
        text_recs[index, 2] = xmax
        text_recs[index, 3] = ymax
        text_recs[index, 4] = line[4]
        index = index + 1
    return text_recs


def get_successions(text_proposals, scores, im_size):
    """
    Get successions text boxes.

    Args:
        text_proposals(numpy.array): Predict text proposals.
        scores(numpy.array): Bbox predicts scores.
        size(numpy.array): Image size.
    Returns:
        sub_graph(list): Proposals graph.
    """
    bboxes_table = [[] for _ in range(int(im_size[1]))]
    for index, box in enumerate(text_proposals):
        bboxes_table[int(box[0])].append(index)
    graph = np.zeros((text_proposals.shape[0], text_proposals.shape[0]), np.bool)
    for index, box in enumerate(text_proposals):
        successions_left = []
        for left in range(int(box[0]) + 1, min(int(box[0]) + config.max_horizontal_gap + 1, im_size[1])):
            adj_box_indices = bboxes_table[left]
            for adj_box_index in adj_box_indices:
                if meet_v_iou(text_proposals, adj_box_index, index):
                    successions_left.append(adj_box_index)
            if successions_left:
                break
        if not successions_left:
            continue
        succession_index = successions_left[np.argmax(scores[successions_left])]
        box_right = text_proposals[succession_index]
        succession_right = []
        for right in range(int(box_right[0]) - 1, max(int(box_right[0] - config.max_horizontal_gap), 0) - 1, -1):
            adj_box_indices = bboxes_table[right]
            for adj_box_index in adj_box_indices:
                if meet_v_iou(text_proposals, adj_box_index, index):
                    succession_right.append(adj_box_index)
            if succession_right:
                break
        if scores[index] >= np.max(scores[succession_right]):
            graph[index, succession_index] = True
    sub_graph = get_sub_graph(graph)
    return sub_graph


def get_sub_graph(graph):
    """
    Get successions text boxes.

    Args:
        graph(numpy.array): proposal graph
    Returns:
        sub_graph(list): Proposals graph after connect.
    """
    sub_graphs = []
    for index in range(graph.shape[0]):
        if not graph[:, index].any() and graph[index, :].any():
            v = index
            sub_graphs.append([v])
            while graph[v, :].any():
                v = np.where(graph[v, :])[0][0]
                sub_graphs[-1].append(v)
    return sub_graphs


def meet_v_iou(text_proposals, index1, index2):
    """
    Calculate vertical iou.

    Args:
        text_proposals(numpy.array): tex proposals
        index1(int): text_proposal index
        tindex2(int): text proposal index
    Returns:
        sub_graph(list): Proposals graph after connect.
    """
    heights = text_proposals[:, 3] - text_proposals[:, 1] + 1
    return overlaps_v(text_proposals, index1, index2) >= config.min_v_overlaps and \
        size_similarity(heights, index1, index2) >= config.min_size_sim


def threshold(coords, min_, max_):
    return np.maximum(np.minimum(coords, max_), min_)


def clip_boxes(boxes, im_shape):
    """
    Clip boxes to image boundaries.

    Args:
        boxes(numpy.array):bounding box.
        im_shape(numpy.array): image shape.

    Return:
        boxes(numpy.array):boundding box after clip.
    """
    boxes[:, 0::2] = threshold(boxes[:, 0::2], 0, im_shape[1] - 1)
    boxes[:, 1::2] = threshold(boxes[:, 1::2], 0, im_shape[0] - 1)
    return boxes


def overlaps_v(text_proposals, index1, index2):
    """
    Calculate vertical overlap ratio.

    Args:
        text_proposals(numpy.array): Text proposlas.
        index1(int): First text proposal.
        index2(int): Second text proposal.

    Return:
        overlap(float32): vertical overlap.
    """
    h1 = text_proposals[index1][3] - text_proposals[index1][1] + 1
    h2 = text_proposals[index2][3] - text_proposals[index2][1] + 1
    y0 = max(text_proposals[index2][1], text_proposals[index1][1])
    y1 = min(text_proposals[index2][3], text_proposals[index1][3])
    return max(0, y1 - y0 + 1) / min(h1, h2)


def size_similarity(heights, index1, index2):
    """
    Calculate vertical size similarity ratio.

    Args:
        heights(numpy.array): Text proposlas heights.
        index1(int): First text proposal.
        index2(int): Second text proposal.

    Return:
        overlap(float32): vertical overlap.
    """
    h1 = heights[index1]
    h2 = heights[index2]
    return min(h1, h2) / max(h1, h2)


def fit_y(X, Y, x1, x2):
    if np.sum(X == X[0]) == len(X):
        return Y[0], Y[0]
    p = np.poly1d(np.polyfit(X, Y, 1))
    return p(x1), p(x2)


def nms(bboxs, thresh):
    """
    Args:
        text_proposals(numpy.array): tex proposals
        index1(int): text_proposal index
        tindex2(int): text proposal index
    """
    x1, y1, x2, y2, scores = np.split(bboxs, 5, axis=1)
    x1 = bboxs[:, 0]
    y1 = bboxs[:, 1]
    x2 = bboxs[:, 2]
    y2 = bboxs[:, 3]
    scores = bboxs[:, 4]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    num_dets = bboxs.shape[0]
    suppressed = np.zeros(num_dets, dtype=np.int32)
    keep = []
    for _i in range(num_dets):
        i = order[_i]
        if suppressed[i] == 1:
            continue
        keep.append(i)
        x1_i = x1[i]
        y1_i = y1[i]
        x2_i = x2[i]
        y2_i = y2[i]
        area_i = areas[i]
        for _j in range(_i + 1, num_dets):
            j = order[_j]
            if suppressed[j] == 1:
                continue
            x1_j = max(x1_i, x1[j])
            y1_j = max(y1_i, y1[j])
            x2_j = min(x2_i, x2[j])
            y2_j = min(y2_i, y2[j])
            w = max(0.0, x2_j - x1_j + 1)
            h = max(0.0, y2_j - y1_j + 1)
            inter = w * h
            overlap = inter / (area_i + areas[j] - inter)
            if overlap >= thresh:
                suppressed[j] = 1
    return keep


def detect(text_proposals, scores, shape):
    keep_proposals, keep_scores = filter_proposal(text_proposals, scores)
    connect_boxes = connect_text_lines(keep_proposals, keep_scores, shape)
    boxes = connect_boxes[filter_boxes(connect_boxes)]
    return boxes


dtype_map = {
    0: np.float32,
    1: np.float16,
    12: np.bool8
}
