#!/usr/bin/env python
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from helper import *

def decode(im) :
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)

    hulls = []
    for obj in decodedObjects:
        # print('Type : ', obj.type)
        # print('Data : ', obj.data,'\n')
        points = obj.polygon
        if len(points) > 4 :
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else :
            hull = points
        hulls.append(hull)
    return decodedObjects, hulls

def process_images(depth_image, color_image):
    detections ={}
    mask = hue_mask(color_image, 150, 5, 70, 250,50, 240)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.erode(mask,kernel,iterations = 1)
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations = 4)
    color_image = cv2.bitwise_and(color_image,color_image,mask = mask)

    _, rois, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for roi_contour in rois:
        roi_mask = np.zeros_like(mask)
        cv2.drawContours(roi_mask,[roi_contour],-1,255,-1)
        locs0, locs1 = np.nonzero(roi_mask)
        margin = 10
        x_min, x_max = max(np.amin(locs0)-margin,0), min(np.amax(locs0)+margin, roi_mask.shape[0])
        y_min, y_max = max(np.amin(locs1)-margin,0), min(np.amax(locs1)+margin, roi_mask.shape[1])
        cropped = color_image[x_min: x_max, y_min: y_max]
        ret2,th2 = cv2.threshold(cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY),0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        decodedObjects, hulls = decode(th2)
        for obj, hull in zip(decodedObjects, hulls):
            n = len(hull)
            for j in range(0,n):
                cv2.line(cropped, hull[j], hull[ (j+1) % n], (255,0,0), 3)
            center = get_contour_centroid(hull)
            if obj.data in detections:
                detections[obj.data].append(center)
            else:
                detections[obj.data] = [center]
            cv2.putText(cropped, obj.data, hull[0], cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
        color_image[x_min: x_max, y_min: y_max] = cropped

    return detections, color_image


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
        image_pub = rospy.Publisher(
                rospy.get_param("processed_image_topic"), Image, queue_size=10)

        position_pub = rospy.Publisher("/bar_bot/qr_multidetections", PoseArray, queue_size=10)
        joint_state_feedback_sub = rospy.Subscriber("/hebiros/all/feedback/joint_state", JointState, joint_state_feedback_callback)

        init_realsense()

        detections ={}
        while not rospy.is_shutdown():
            detections = process_images(detections)
            #print(detections)
            # rospy.spinOnce()

    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
