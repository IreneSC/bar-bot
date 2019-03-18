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
    kernel = np.ones((15,15),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations = 3)
    color_image = cv2.bitwise_and(color_image,color_image,mask = mask)

    _, rois, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for roi_contour in rois:
        roi_mask = np.zeros_like(mask)
        cv2.drawContours(roi_mask,[roi_contour],-1,255,-1)
        locs0, locs1 = np.nonzero(roi_mask)
        margin = 10
        y_min, y_max = max(np.amin(locs0)-margin,0), min(np.amax(locs0)+margin, roi_mask.shape[0])
        x_min, x_max = max(np.amin(locs1)-margin,0), min(np.amax(locs1)+margin, roi_mask.shape[1])
        cropped = color_image[y_min: y_max, x_min: x_max]
        cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
        # cropped = adjust_gamma(cropped, gamma=0.5)
        # cropped = cropped.astype('float64')
        # cropped[cropped<int(255/2)]*=0.8
        # cropped[cropped>int(255/2)]*=1.05
        # cropped[cropped>255]=255
        # cropped = cropped.astype('uint8')
        # white_mask = cropped 255/2,255)
        try:
            white = int(np.mean(cropped[cropped>int(255/2)]))
            cropped[cropped==0] = white
        except ValueError:
            pass

        cropped = cv2.cvtColor(cropped,cv2.COLOR_GRAY2BGR)
        # Big = cv2.resize(cropped, (0,0), fx=2, fy=2)
        # ret2,th2 = cv2.threshold(cropped[:,:,1],0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # cropped = cv2.cvtColor(th2, cv2.COLOR_GRAY2BGR)
        decodedObjects, hulls = decode(cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY))

        center = None
        circclllee = (x_min, y_min)
        for obj, hull in zip(decodedObjects, hulls):
            n = len(hull)
            center_x = 0
            center_y = 0
            for j in range(0,n):
                cv2.line(cropped, hull[j], hull[ (j+1) % n], (255,0,0), 3)
                center_x +=hull[j][1]/n
                center_y +=hull[j][0]/n
            center = (center_x + x_min, center_y + y_min)
            print("center: ", center)
            if obj.data in detections:
                detections[obj.data].append(center)
            else:
                detections[obj.data] = [center]
            cv2.putText(cropped, obj.data, hull[0], cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
        color_image[y_min: y_max, x_min: x_max] = cropped

        if (center is not None):
            cv2.circle(color_image, center, 10, (255,0,255), -1)

        # cv2.circle(color_image, circclllee, 30, (255,255,0), -1)

    return detections, color_image
