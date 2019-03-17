#!/usr/bin/env python
import sys
import math
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from helper import *

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
from bar_bot import Detections

import cup_detector
import qr_detector

bridge = CvBridge()

def merge_and_make_global(pixels_centers, aligned_depth_frame, depth_intrin, merging_dist=0.03):
    global_centers = []
    for (cx, cy) in pixel_centers:
        local = pixel_to_local3d(cx, cy, aligned_depth_frame, depth_intrin)
        world = local3d_to_global(local)
        merged = False
        for center in global_centers:
            if np.linalg.norm(center-world) <= merging_dist:
                center = (center+world)/2
                merged = True
                break
        if not merged:
            global_centers.append(world)

def track_objects(detections, global_centers, dist_error=0.08, delete_score = 0, min_score = 5, max_score = 15):

    for center in global_centers:
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
    scored_positions = []
    for i in range(len(detections)-1,-1,-1):
        score, det = detections[i]
        detections[i] = (score - 0.5, det)
        if score < 0:
            detections.pop(i)
            continue
        if score >=min_score:
            center = det[len(det)-1]
            scored_positions.append(score, Position(center[0], center[1], center[2]))
        if score >max_score:
            detections[i] = max_score, det
    scored_positions.sort(key=lambda x: x[0])
    if len(scored_positions>0):
        return scored_positions[0][1], detections
    else:
        return None, detections


def main():
    try:
        rospy.init_node("detector_node")
        cup_image_pub = rospy.Publisher("/bar_bot/cup_image", Image, queue_size=10)
        bottle_image_pub = rospy.Publisher("/bar_bot/bottle_image", Image, queue_size=10)
        position_pub = rospy.Publisher("/bar_bot/detections", Detections, queue_size=10)
        joint_state_feedback_sub = rospy.Subscriber("/hebiros/all/feedback/joint_state", JointState, joint_state_feedback_callback)

        init_realsense()

        global_detections = {}
        while not rospy.is_shutdown():
            depth_image, color_image, aligned_depth_frame, depth_intrin = get_frames()
            d = Detections()
            d.header.stamp = rospy.Time.now()
            cup_detections, cup_result_img = cup_detector.process_images(depth_image.copy(), color_image.copy())
            bottle_detections, bottle_result_img = qr_detector.process_images(depth_image.copy(), color_image.copy())


            for key in cup_detections:
                if key not in global_detections:
                    global_detections[key] = []
                global_centers = merge_and_make_global(cup_detections[key]), aligned_depth_frame, depth_intrin, merging_dist=0.03)
                best_pos, global_detections[key] = track_objects(global_detections[key], global_centers, dist_error=0.08, delete_score = 0, min_score = 5, max_score = 15)
                if best_pos is not None:
                    d.detection_types.append(key)
                    d.detection_positions.append(best_pos)
            for key in bottle_detections:
                if key not in global_detections:
                    global_detections[key] = []
                global_centers = merge_and_make_global(bottle_detections[key]), aligned_depth_frame, depth_intrin, merging_dist=0.03)
                best_pos, global_detections[key] = track_objects(global_detections[key], global_centers, dist_error=0.08, delete_score = -2, min_score = 0.5, max_score = 15)
                if best_pos is not None:
                    d.detection_types.append(key)
                    d.detection_positions.append(best_pos)
                    
            position_pub.publish(d)
            try:
                cup_image_pub.publish(bridge.cv2_to_imgmsg(cup_result_img,"bgr8"))
                bottle_image_pub.publish(bridge.cv2_to_imgmsg(bottle_result_img,"bgr8"))
            except CvBridgeError as e:
                print(e)

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
