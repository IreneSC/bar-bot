import cv2
import math
import numpy as np
from threading import Lock
import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()
align = None  # Used to align the depth and color images
theta1 = 0

def pixel_to_local3d(px, py, depth_frame, depth_intrin):
# def pixel_to_local3d(px, py, r, proj_mat):
    """
    Params:
        px, py:         raw pixel coords (ints)
        depth_frame:    the realsense depth frame.
        # r:              distance in straight line to px, py
        # proj_mat:       3x3 numpy array of camera matrix

    Return:
        3x1 np matrix: x, y, z (camera local 3d coordinates)detections
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
    cam_offset = [.097, .08, 0.07]

    # Global x direction is local z (i.e. planar distance from camera)
    # Global y is local x (left/right in camera frame)
    # Global z is local y (up/down in camera frame)
    permuted = np.array( [local[0], local[2], -local[1]] )

    # TODO: plus or minus cam offset? Depends on how defined
    ret = np.dot(rotate_z(theta1), (permuted + cam_offset))
    # Minimum height of .05
    ret[2] = max(0.1, ret[2])
    return ret
    # return permuted

def get_frames():
    # Align the depth frame to color frame
    frames = pipeline.wait_for_frames()
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

    return depth_image, color_image, aligned_depth_frame, depth_intrin

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
    #

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


def get_contour_centroid(contour):
    """
    Returns the center of the contour input. contour must not be empty
    """
    M = cv2.moments(contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)


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


def compare_rectangle(cnt, rect_cnt):
    square_error=0.0
    for pt in cnt[0::NUM_RECT_POINTS]:
        square_error += (cv2.pointPolygonTest(rect_cnt,tuple(pt[0]),True))**2
    for pt in rect_cnt:
        square_error += (cv2.pointPolygonTest(cnt,tuple(pt),True))**2
    return math.sqrt(square_error)

def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
    	for i in np.arange(0, 256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)
