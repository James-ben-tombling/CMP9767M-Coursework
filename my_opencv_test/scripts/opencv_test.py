#!/usr/bin/env python

import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange

from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey, COLOR_BGR2HSV
from cv2 import blur, Canny, resize, INTER_CUBIC
from numpy import mean
import numpy as np
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
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = resize(cv_image, None, fx=0.2, fy=0.2, interpolation = INTER_CUBIC)

        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)
        greenmaskout_img = inRange(hsv_img, (36,25,25), (70,255,255) )
        imask = greenmaskout_img>0
        green= np.zeros_like(cv_image, np.uint8)
        green[imask] = cv_image[imask]
        imshow("green isolate", green)
        
        mask = inRange(cv_image, (0, 150, 150), (255, 255, 255))
        imshow("masked", mask)
        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        img3 = Canny(gray_img, 10, 200)
        imshow("canny", img3)

        imshow("Image window", cv_image)


        waitKey(1)

#startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

#destroyAllWindows()