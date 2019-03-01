#!/usr/bin/env python
import sys
import math
import rospy
import cv2
import numpy as np
from threading import Lock
import pyrealsense2 as rs

# Ros pub/sub
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
# import message_filters

# Pub/sub global variables
bridge = CvBridge()
image_pub = None
position_pub = None
joint_state_feedback_sub = None
# Create a pipeline
pipeline = rs.pipeline()
align = None  # Used to align the depth and color images

# the current yaw of the arm in radians
theta1 = 0

# bgr and depth camera global variables
# image_ready = False
# image_bgr = None
# image_depth = None
# image_lock = Lock()

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
    binary = hue_mask(img, 30, 55, 20, 170, 20, 175)
    kernel = np.ones((3,3),np.uint8)
    binary = cv2.erode(binary,kernel,iterations = 5)
    binary = cv2.dilate(binary,kernel,iterations = 5)
    _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return binary, contours


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


# def image_callback(bgr, depth):
#     """
#     Takes the associated bgr and depth image, and updates the global variables
#     containing them so that we can process them.
#     """
#     with image_lock:
#         global image_bgr
#         global image_depth
#         global image_ready
#         image_bgr = bgr
#         image_depth = depth
#         image_ready = True


def process_images():
    """
    Process the latest images, waiting for new ones to become available if necessary.
    Publishes the new estimate of the global position.
    """

    # try:
    #     with image_lock:
    #         global image_ready
    #         if image_ready:
    #             cv_bgr  = bridge.imgmsg_to_cv2(image_bgr, "bgr8")
    #             cv_depth  = bridge.imgmsg_to_cv2(image_depth, "mono16")
    #             image_ready = False  # Don't process the same images twice.
    # except CvBridgeError as e:
    #     print(e)

    # Get frameset of color and depth
    # print("waiting for frames")
    frames = pipeline.wait_for_frames()
    # print("got frames")
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a depth image
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        print ("one was none")
        return

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Finding the cup
    binary, contours = process_image(color_image)
    max_contour = find_max_contour(contours);

    # Display the result
    result = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    # Draw the cup
    if not max_contour ==[]:
        cv2.drawContours(result, [max_contour], -1, (0,255,0), 2)

    # Draw all other contours
    cv2.drawContours(result, contours, -1, (255,0,0), 2)

    # Publish the feedback image
    global image_pub
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(result,"bgr8"))
    except CvBridgeError as e:
        print(e)

    # Find the global coords of the cup
    if not max_contour == []:
        cx, cy = get_contour_centroid(max_contour)
        local = pixel_to_local3d(cx, cy, aligned_depth_frame, depth_intrin)
        world = local3d_to_global(local)

        # Publish the coordinates
        stamped_pos = PointStamped()
        stamped_pos.point.x = world[0]
        stamped_pos.point.y = world[1]
        stamped_pos.point.z = world[2]
        stamped_pos.header.stamp = rospy.Time.now()
        position_pub.publish(stamped_pos)

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
    cam_offset = [.1, .08, 0.07]

    # Global x direction is local z (i.e. planar distance from camera)
    # Global y is local x (left/right in camera frame)
    # Global z is local y (up/down in camera frame)
    permuted = np.array( [local[0], local[2], -local[1]] )

    # TODO: plus or minus cam offset? Depends on how defined
    ret = np.dot(rotate_z(theta1), (permuted + cam_offset))
    # Minimum height of .05
    ret[2] = max(0.15, ret[2])
    return ret
    # return permuted


def init_realsense():
    # RealSense Setup

    # Create a config and configure the pipeline to stream
    #  the same resolutions of color and depth streams
    # TODO: bump up resolution?
    print("start init")
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)

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
    print(theta1)

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
        position_pub = rospy.Publisher(
                rospy.get_param("cup_detection_topic"), PointStamped, queue_size=10)
        global joint_state_feedback_sub
        joint_state_feedback_sub = rospy.Subscriber("/hebiros/all/feedback/joint_state", JointState, joint_state_feedback_callback)

        init_realsense()

        while not rospy.is_shutdown():
            process_images()
            # rospy.spinOnce()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
