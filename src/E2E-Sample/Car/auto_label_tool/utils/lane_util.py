import cv2
import numpy as np
import logging
import os

def detect_edges(frame):
    """
    寻找色域 + 腐蚀膨胀
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 读取色域范围
    base_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    hsv_data_dir = os.path.join(base_dir, "hsv-data")
    hsv_data_path = os.path.join(hsv_data_dir, "hsv.txt")
    
    with open(hsv_data_path, 'r') as f:
        content = f.read()
        
    low_h, high_h, low_s, high_s, low_v, high_v = list(map(int, content.split("\n")))
    
    # 设置色域范围
    lower_color = np.array([low_h, low_s, low_v]) 
    upper_color = np.array([high_h, high_s, high_v])
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    # 腐蚀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    erode = cv2.erode(mask, kernel, iterations=5)

    # 膨胀
    dilate = cv2.dilate(erode, kernel, iterations=2)

    # 闭运算
    close = cv2.morphologyEx(dilate, cv2.MORPH_CLOSE, kernel, iterations=15)

    return close


def region_of_interest(img):
    height, width = img.shape
    mask = np.zeros_like(img)

    polygon = np.array([[
        (height // 18, height // 2), 
        (width, height // 2), 
        (width, height),
        (height // 18, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
    
    
def detect_line_segments(cropped_edges):
    rho = 1  
    angle = np.pi / 180  
    min_threshold = 10  
    minLineLength = 10 
    maxLineGap = 6 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength,
                                    maxLineGap)

    return line_segments 


def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/2
    left_region_boundary = width * (1 - boundary)  
    right_region_boundary = width * boundary 

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment: 
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(make_points(frame, left_fit_average))

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height 
    y2 = int(y1 * 1 / 2)  

    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=5): # revise
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image