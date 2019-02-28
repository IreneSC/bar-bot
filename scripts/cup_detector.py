#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
def hue_mask(img, minHue, maxHue, minSaturation, maxSaturation, minValue, maxValue):
    """Create a binary image based on the desired HSV range

    Parameters
    ----------
    img : OpenCV image
        image to mask
    minHue : int in range [0,179]
        the lower value of the desired hue range
    maxHue : int in range [0,179]
        the upper value of the desired hue range (may be less than minHue, since
        hue wraps around at red)
    minSaturation : int in range [0,255]
        the lower value of the desired saturation range
    maxSaturation :  int in range [0,255]
        the upper value of the desired saturation range
    minValue : int in range [0,255]
        the lower value of the desired value range
    maxValue : int in range [0,255]
        the upper value of the desired value range

    Returns
    -------
    binary OpenCV image
        a mask of the pixels that are in the range

    """

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    (h,s,v) = cv2.split(hsv)
    ret, h_minthresh = cv2.threshold(h, minHue, 255, cv2.THRESH_BINARY)
    ret, h_maxthresh = cv2.threshold(h, maxHue, 255, cv2.THRESH_BINARY_INV)
    ret, s_minthresh = cv2.threshold(s, minSaturation, 255, cv2.THRESH_BINARY)
    ret, s_maxthresh = cv2.threshold(s, maxSaturation, 255, cv2.THRESH_BINARY_INV)
    ret, v_minthresh = cv2.threshold(v, minValue, 255, cv2.THRESH_BINARY)
    ret, v_maxthresh = cv2.threshold(v, maxValue, 255, cv2.THRESH_BINARY_INV)
    #account for hue values wrapping around at red
    if minHue < maxHue:
        h_thresh = cv2.bitwise_and(h_minthresh, h_maxthresh)
    else:
        h_thresh = cv2.bitwise_or(h_minthresh, h_maxthresh)
    s_thresh = cv2.bitwise_and(s_minthresh, s_maxthresh)
    v_thresh = cv2.bitwise_and(v_minthresh, v_maxthresh)
    result = cv2.bitwise_and(h_thresh, s_thresh)
    result = cv2.bitwise_and(result, v_thresh)
    detector =  cv2.cvtColor(result, cv2.COLOR_GRAY2BGR) #display image result
    return result

def process_image(img):
    binary = hue_mask(img, 35, 40, 35, 165, 15, 165)
    kernel = np.ones((3,3),np.uint8)
    binary = cv2.dilate(binary,kernel,iterations = 3)
    binary = cv2.erode(binary,kernel,iterations = 3)
    binary = cv2.dilate(binary,kernel,iterations = 3)
    _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return binary, contours


def find_max_contour(contours):
    if len(contours)==0:
        return []
    max_contour_area = cv2.contourArea(contours[0])
    max_contour = contours[0]
    max_index = 0
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area>max_contour_area:
            max_contour=cnt
            max_contour_area=area
            max_index = i
    epsilon = 0.02*cv2.arcLength(max_contour,True)
    max_contour = cv2.approxPolyDP(max_contour,epsilon,True)
    area = cv2.contourArea(max_contour)
    if area < 600:
        print("too smol :(((")
        return []
    print("big boi")
    return max_contour

def img_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    binary, contours = process_image(cv_image)
    max_contour = find_max_contour(contours);
    result = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    if not(max_contour ==[]):
        cv2.drawContours(result, [max_contour], -1, (0,255,0), 2)
    cv2.drawContours(result, contours, -1, (255,0,0), 2)
    cv2.imshow("Image window", cv_image)
    cv2.imshow("Processed Image Window", result)
    cv2.waitKey(3)


def main():
    try:
        rospy.init_node("cup_boi")
        image_sub = rospy.Subscriber("/cam/image_raw", Image, img_callback)
        while not rospy.is_shutdown():
            rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
