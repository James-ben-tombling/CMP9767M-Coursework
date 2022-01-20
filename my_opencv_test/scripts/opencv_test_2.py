#!/usr/bin/env python

import rospy 
import imutils
from imutils import contours
from skimage import measure
from cv2 import namedWindow, cvtColor, imshow, inRange, FILLED
import cv2
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC, drawContours
from numpy import mean
import numpy as np
import matplotlib.pyplot as plt 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect",
                                          Image, self.image_callback)
        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw",
        #     Image, self.callback)

    def image_callback(self, data):
        namedWindow("Image window")
        namedWindow("masked")
        namedWindow("canny")
        namedWindow("green isolate")
        namedWindow("Grape Bunch Count")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #cv_image = cv2.imread("~/catkin_ws/src/my_opencv_test/images/grapes4.png")
        cv_image = resize(cv_image, None, fx=0.7, fy=0.7, interpolation = INTER_CUBIC)
# green mask image 
        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)
        greenmaskout_img = inRange(hsv_img, (36,25,25), (70,255,255) )
        imask = greenmaskout_img>0
        green= np.zeros_like(cv_image, np.uint8)
        green[imask] = cv_image[imask]
        #imshow("green isolate", hsv_img)
# HSV image 
        #grape= np.uint8([[[128,0,128]]])
        grape_img = cvtColor(cv_image, COLOR_BGR2HSV)
        grapemaskout_img = inRange(grape_img, (100,25,25), (190,255,255) )
        jmask = grapemaskout_img>0 
        grape= np.zeros_like(grape_img, np.uint8)
        grape[jmask] = grape_img[jmask] 


        grape_canny = Canny(grape, 10, 200)
        rect=cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        grape_dilate = cv2.dilate(grape_canny, rect,iterations = 10 )
        grape_erode = cv2.erode(grape_dilate, rect, iterations=7)
    
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(grape_erode, None, None, None, 8, cv2.CV_32S)
        areas = stats[1:,cv2.CC_STAT_AREA]
        grape_denoise = np.zeros((labels.shape), np.uint8)
        
        for i in range(0, nlabels - 1):
            if areas[i] >= 900:   #keep
                grape_denoise[labels == i + 1] = 255
        labels = measure.label(grape_denoise, neighbors=8, background=0)
        mask = np.zeros(grape_denoise.shape, dtype="uint8")
        for label in np.unique(labels):
            if label == 0:
                continue 
            labelMask = np.zeros(grape_denoise.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            if numPixels > 900:
                mask = cv2.add(mask, labelMask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	        cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = contours.sort_contours(cnts)[0]  
        for (i, c) in enumerate(cnts):
            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(cv_image, (int(cX), int(cY)), int(radius),
		        (0, 0, 255), 3)
            cv2.putText(cv_image, "{}".format(i + 1), (x, y - 15),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2) 

	    
        imshow("Grape Bunch Count" , cv_image)
# mask
        mask = inRange(cv_image, (0, 150, 150), (255, 255, 255))
        imshow("masked", grape_denoise)
# canny 
        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        img3 = Canny(gray_img, 10, 200)
        #imshow("canny", img3)

        #imshow("Image window", cv_image)


        waitKey(1)

#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

#destroyAllWindows()