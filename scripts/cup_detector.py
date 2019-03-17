#!/usr/bin/env python
import cv2
import numpy as np
from threading import Lock
import pyrealsense2 as rs
from helper import *

ISCV3 = cv2.__version__[0]=="3"

# Ros pub/sub
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
# import message_filters

MIN_CONTOUR_AREA = 500
NUM_RECT_POINTS=20
SHAPE_RET_THRESHOLD=12

def process_image(img):
    # binary = hue_mask(img, 30, 55, 15, 160,15, 160)
    binary = hue_mask(img, 27, 58, 10, 170,10, 170)
    kernel = np.ones((5,5),np.uint8)
    binary = cv2.erode(binary,kernel,iterations = 7)
    binary = cv2.dilate(binary,kernel,iterations = 7)
    _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return binary, contours

def find_cups(img):
    binary, contours = process_image(img)
    result = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(result,contours,-1,(255,0,0),2)
    rectangles = []
    failed_rects = []
    for contour in contours:
        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            continue
        rect = cv2.minAreaRect(contour)
        if ISCV3:
            rect_cnt = np.int0(cv2.boxPoints(rect))
        else:
            rect_cnt =np.int0(cv2.cv.BoxPoints(rect))

        approx = cv2.approxPolyDP(contour, .03 * cv2.arcLength(contour, True), True)
        if len(approx) != 4:
            failed_rects.append(approx)
            continue
        if cv2.contourArea(approx) < 6000:
            failed_rects.append(approx)
            continue
        rectangles.append(approx)

    cv2.drawContours(result,rectangles,-1,(0,255,0),2)
    cv2.drawContours(result,failed_rects,-1,(0,0,255),2)
    return result, rectangles

def process_images(depth_image, color_image):
    """
    Process the latest images, waiting for new ones to become available if necessary.
    Publishes the new estimate of the global position.
    """
    # Finding the cup
    result_img, rectangles = find_cups(color_image)
    detections ={}
    detections["cup"] = []
    for rect in rectangles:
        cx, cy = get_contour_centroid(rect)
        detections["cup"].append((cx,cy))


    return detections, result_img
