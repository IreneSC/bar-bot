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
        local = pixel_to_local3d(cx, cy, aligned_depth_frame, depth_intrin)
        world = local3d_to_global(local)
        merged = False
        for center in centers:
            if np.linalg.norm(center-world) <= merging_dist:
                center = (center+world)/2
                merged = True
                break
        if not merged:
            centers.append(world)

    #assign contours to detections
    for center in centers:
        assigned_det = -1
        closest_dist = float('inf')
        for i in range(len(detections)):
            score, det = detections[i]
            avg = np.array([sum(x) for x in zip(*det)])/len(det)
            dist = np.linalg.norm(avg-center)
            if  dist <= dist_error and dist <= closest_dist :
                assigned_det = i
                closest_dist = dist
        if assigned_det == -1:
            detections.append((1.5, [center]))
        else:
            score, det = detections[assigned_det]
            det.append(center)
            detections[assigned_det] = (score+1.5, det)
            if len(det) >5:
                det.pop(0)

    #update detection scores/decide which to return
    for i in range(len(detections)-1,-1,-1):
        score, det = detections[i]
        detections[i] = (score - 0.5, det)
        if score < 0:
            detections.pop(i)
            continue
        if score >=min_score:
            center = det[len(det)-1]
            p = Pose()
            p.position.x = center[0]
            p.position.y = center[1]
            p.position.z = center[2]
            stamped_positions.poses.append(p)
        if score >max_score:
            detections[i] = max_score, det

    # print("num poses:", len(stamped_positions.poses))
    position_pub.publish(stamped_positions)



    # Publish the feedback image
    global image_pub
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(result_img,"bgr8"))
    except CvBridgeError as e:
        print(e)

    return detections

def get_contour_centroid(contour):
    """
    Returns the center of the contour input. contour must not be empty
    """
    M = cv2.moments(contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)


def pixel_to_local3d(px, py, depth_frame, depth_intrin):
# def pixel_to_local3d(px, py, r, proj_mat):
    """
    Params:
        px, py:         raw pixel coords (ints)
        depth_frame:    the realsense depth frame.
        # r:              distance in straight line to px, py
        # proj_mat:       3x3 numpy array of camera matrix

    Return:
        3x1 np matrix: x, y, z (camera local 3d coordinates)
    """
    depth = depth_frame.get_distance(px, py)
    depth_point = rs.rs2_deproject_pixel_to_point(
                                depth_intrin, [px, py], depth)
    return depth_point
    # x = (px - proj_mat[0,2]) / proj_mat[0, 0]
    # y = (py - proj_mat[1,2]) / proj_mat[1, 1]
    # z = r * sqrt(1/(x ** 2 + y ** 2 + 1))
    # return np.array([x, y, z])


def rotate_z(theta):
    """
    Return:
        Rotation of an angle `theta` about the z axis
    """

    return np.array([ [math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta),  math.cos(theta), 0],
                      [0,           0,          1]])


def local3d_to_global(local):
    """
    Params:
        local:      local coords returned by pixel_to_local3d
        theta1:     Rotation of first angle - the yaw of the arm

    Return:
        3x1 np matrix: global x, y, z
    """
    global theta1
    # 3x1 np matrix of offset of camera to global frame
    cam_offset = [.07, .08, 0.07]

    # Global x direction is local z (i.e. planar distance from camera)
    # Global y is local x (left/right in camera frame)
    # Global z is local y (up/down in camera frame)
    permuted = np.array( [local[0], local[2], -local[1]] )

    # TODO: plus or minus cam offset? Depends on how defined
    ret = np.dot(rotate_z(theta1), (permuted + cam_offset))
    # Minimum height of .01
    ret[2] = max(0.01, ret[2])
    return ret
    # return permuted


def init_realsense():
    # RealSense Setup

    # Create a config and configure the pipeline to stream
    #  the same resolutions of color and depth streams
    # TODO: bump up resolution?
    print("start init")
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    global align
    align = rs.align(align_to)
    print("done init")

def joint_state_feedback_callback(joint_state):
    global theta1
    theta1 = joint_state.position[0]
    # print(theta1)

def find_max_contour(contours):
    """
    Return the largest contour in the vector of contours by area, as long
    as that largest contour has a sufficiently large area.
    Return [] if no contour is sufficiently large.
    """
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
        return []
    return max_contour

def main():
    try:
        rospy.init_node("cup_boi")

        # Subscribers
        # bgr_sub = rospy.Subscriber("/camera/color/image_raw", Image, bgr_img_callback)
        # depth_sub = rospy.Subscriber("TODO", Image, bgr_img_callback)

        # # Synchronize the subscribers
        # time_sync = message_filters.TimeSynchronizer([bgr_sub, depth_sub], 1)
        # ts.registerCallback(image_callback)

        # Publishers
        global image_pub
        image_pub = rospy.Publisher(
                rospy.get_param("processed_image_topic"), Image, queue_size=10)

        global position_pub
        position_pub = rospy.Publisher("/bar_bot/cup_multidetections", PoseArray, queue_size=10)
        global joint_state_feedback_sub
        joint_state_feedback_sub = rospy.Subscriber("/hebiros/all/feedback/joint_state", JointState, joint_state_feedback_callback)

        init_realsense()

        detections =[]
        while not rospy.is_shutdown():
            detections = process_images(detections)
            # print(detections)
            # rospy.spinOnce()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
>>>>>>> Working demo, except for collision detections are poor


    return detections, result_img
