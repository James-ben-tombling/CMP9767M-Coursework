#!/usr/bin/env python
# made by James Tombling 
# code based on image_projection_3.py  
# https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/image_projection_3.py
# 

# Python libs
from email.header import Header

from ossaudiodev import SNDCTL_SEQ_CTRLRATE
import sys
import imutils
import numpy as np
from imutils import contours
from skimage import measure
from sklearn.cluster import DBSCAN


# OpenCV
import cv2
from cv2 import Canny, cvtColor,inRange, COLOR_BGR2HSV
# Global varibales 
grapelist = [] # grape coords 
vinelist = [] # vineyrad map list 
VineyardParticles = 15000 # number of particles in vinyard mapping 

# Ros libraries
import rospy, image_geometry, tf

# Ros Messages
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from geometry_msgs.msg import PoseStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2, Image, PointField, CameraInfo
from sensor_msgs import point_cloud2 
from std_msgs.msg import String 


#class grape_bunch:

class image_projection:
    
    camera_model = None
    image_depth_ros = None
    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    color2depth_aspect = (84.1/1920) / (70.0/512)
    # init image projection node 
    rospy.init_node("image_projection")
    #Grape_PointCloud = rospy.Publisher("/Grape_PointCloud", PointCloud2, queue_size= 2)
    
    # data headers and fields for the grape point cloud 
    fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()
    
    # data headers and fields for the grape point cloud
    cam_fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
    cam_header = Header()
    cam_header.frame_id = "map"
    cam_header.stamp = rospy.Time.now()
    
    # initialise all Subs, Pubs, TF tranforms, and bridges 
    def __init__(self):    
        
        self.bridge = CvBridge()
        self.camera = rospy.Subscriber('/camera', String, self.camera)
        self.vine_camera = rospy.Subscriber('/vine_camera', String, self.vine_camera)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_left_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)
        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)
        self.camera_pointcloud_sub = rospy.Subscriber('/thorvald_001/kinect2_left_sensor/sd/points', PointCloud2,
         self.camera_point_callback)
        rospy.Subscriber("/thorvald_001/kinect2_left_camera/hd/image_color_rect",
            Image, self.image_color_callback)
        rospy.Subscriber("/thorvald_001/kinect2_left_sensor/sd/image_depth_rect",
            Image, self.image_depth_callback)
        self.Camera_PointCloud = rospy.Publisher("/Camera_PointCloud", PointCloud2, queue_size=10)
        self.Grape_PointCloud = rospy.Publisher("/Grape_PointCloud", PointCloud2, queue_size=10)
        self.tf_listener = tf.TransformListener()

    def camera_point_callback(self, data): # set laser scan coordinates 
        self.camera_point_ros = data

    def camera_info_callback(self, data): # setting kinect2 camera info 
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def image_depth_callback(self, data): # setup fo depth camera
        self.image_depth_ros = data

    def image_color_callback(self, data): # setup fro RGB camera data 
        self.image_colour_ros = data

    def vine_camera(self,data): # triggered by publohser in controller.py
        ### print the current vinyard layout 
       
        camera_cloud_data = point_cloud2.read_points(self.camera_point_ros, skip_nans= True) # read cloud points from laser scan 
        camera_cloud_list = list(camera_cloud_data) # turn data coords to a list 
        Vine_location = PoseStamped() 
        Vine_location.header.frame_id = "thorvald_001/kinect2_left_rgb_optical_frame" # set frame from camera location
        for v in range(VineyardParticles): # step through every particle coordinate 
            ccl = camera_cloud_list[v] # 
            Vine_location.pose.orientation.w = 1.0
            Vine_location.pose.position.x = ccl[0]
            Vine_location.pose.position.y = ccl[1]
            Vine_location.pose.position.z = ccl[2]
            v_camera = self.tf_listener.transformPose('map', Vine_location) # transform x, y z coordinates 
            vinelist.append([v_camera.pose.position.x, v_camera.pose.position.y, v_camera.pose.position.z])  # append list of world coords 
        Camera_cloud = point_cloud2.create_cloud(self.cam_header, self.cam_fields, vinelist ) # make cloud 
        self.Camera_PointCloud.publish(Camera_cloud) # publish cloud 

    def camera(self, data): #  main script for identifying , counting and plotting grape location
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return
        try:
            image_color = self.bridge.imgmsg_to_cv2(self.image_colour_ros, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print (e)

        
        
        image_color = cv2.resize(image_color, (1024,848)) # resize image 
        # image detection by colour filtering and image processing  
        image_hsv = cvtColor(image_color, COLOR_BGR2HSV) # conver rgb to hsv
        image_mask = inRange(image_hsv, (100,25,25), (190,255,255) ) # filter hsv
        imask = image_mask>0 # remove anything not filtered 
        grape= np.zeros_like(image_hsv, np.uint8) # zero
        grape[imask] = image_hsv[imask] # add mask

        grape_canny = Canny(grape, 10, 200) # canny algorithm 
        rect=cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) # make a structual 
        grape_dilate = cv2.dilate(grape_canny, rect,iterations = 10 ) # dilate binary image to remove holes 
        grape_erode = cv2.erode(grape_dilate, rect, iterations=7) # erode back down to normal size 
        
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(grape_erode, None, None, None, 8, cv2.CV_32S) # get labels for points 
        areas = stats[1:,cv2.CC_STAT_AREA] # get area binary 
        grape_denoise = np.zeros((labels.shape), np.uint8) # denoise binary image 
        
        for i in range(0, nlabels - 1):  # specify the min size of binary blod to remove noise 
            if areas[i] >= 900:   
                grape_denoise[labels == i + 1] = 255
        labels = measure.label(grape_denoise, neighbors=8, background=0) # add labels 
        mask = np.zeros(grape_denoise.shape, dtype="uint8") # mask noise 
        for label in np.unique(labels): # label 
            if label == 0:
                continue 
            labelMask = np.zeros(grape_denoise.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            if numPixels > 900:
                mask = cv2.add(mask, labelMask)


        # if no grapes are detected exit the loop and rescan in nexter iteration 
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, # find contours of binary blobs
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts) # get the countours 
        if len(cnts) == 0: # if no garpes detcted restart 
            print('No grapes detected.')
            return
        cnts = contours.sort_contours(cnts)[0] # sort contours from left to right 

        #cycle through each contour location (grape bunch)
        for (i, c) in enumerate(cnts) :

            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(image_color, (int(cX), int(cY)), int(radius),
                (0, 0, 255), 3)
   
            image_coords = (y, x)
        # "map" from color to depth image
            depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
            image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
        # get the depth reading at the centroid location
            
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
            print ('bunch number' , i+1)      
            print ('image coords: ', image_coords)
            print ('depth coords: ', depth_coords)
            
            if np.isnan(depth_value):
                print('nan detected')
                # depth_value = image_depth[int(depth_coords[0] + 0.002), int(depth_coords[1]+ 0.002)]
                depth_value = 1.5
               
            
            #print ('depth value: ', depth_value )
        # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [X/camera_coords[2] for X in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [X*depth_value for X in camera_coords] # multiply the vector by depth
            #print 'camera coords: ', camera_coords

        #define a point in camera coordinates
            object_location = PoseStamped()
            object_location.header.frame_id = "thorvald_001/kinect2_left_rgb_optical_frame"
            object_location.pose.orientation.w = 1.0
            object_location.pose.position.x = camera_coords[0]
            object_location.pose.position.y = camera_coords[1]
            object_location.pose.position.z = camera_coords[2]
        # publish so we can see that in rviz
            self.object_location_pub.publish(object_location)
        

            # print out the coordinates in the map frame
            p_camera = self.tf_listener.transformPose('map', object_location)
            print ('map coords: ', p_camera.pose.position)
            

            cv2.putText(image_color,  "{}.({}, {}, {})".format(i + 1,round(p_camera.pose.position.x, 3),round(p_camera.pose.position.y,3),round(p_camera.pose.position.z,3)), (x, y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
            grapelist.append([p_camera.pose.position.x, p_camera.pose.position.y, p_camera.pose.position.z])
            
            
            ## filter results with dbscan ## 
    
            epsilon = 0.03 # 0.020
            min_samples = 10 # 10
            
            db = DBSCAN(eps=epsilon, min_samples=min_samples).fit(grapelist)
            labels = db.labels_
            # cluster_centroids =  np.mean(math.dist[np.unique(labels), :])
            
            no_clusters = len(np.unique(labels))

           
            print('number of predicted bunches:', no_clusters)
            

  
        Grape_cloud = point_cloud2.create_cloud(self.header, self.fields, grapelist)
        self.Grape_PointCloud.publish(Grape_cloud)
       
        if self.visualisation:


            #resize and adjust for visualisation
            image_color = cv2.resize(image_color, (0,0), fx=1, fy=1)
            image_depth *= 1.0/10.0 # scale for visualisation (max range 10.0 m)
        
            cv2.imshow("image depth", image_depth)
            cv2.imshow("Grape Bunch Count", image_color)
            cv2.waitKey(1)
        # self.Grape_PointCloud.publish(point_cloud)
        #        #time.sleep(1)


           
def main(args):
    '''Initializes and cleanup ros node'''
    #rospy.init_node('image_projection', anonymous=True)
    
    ic = image_projection()
    try:
        
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
