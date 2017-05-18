#!/usr/bin/env python

from __future__ import print_function
import rospy, tf
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
import cv2, cv_bridge
from image_geometry import PinholeCameraModel
import numpy as np
import csv, sys, os
from camera_info import *
from utils import *

class Projection:

    def __init__(self, md_path, calib_file):
        self.reset()
        self.current_time = rospy.Time()
        self.metadata = load_metadata(md_path)
        self.calib_file = calib_file

    def reset(self):
        self.last_cap_r = None
        self.last_cap_f = None
        self.last_cap_yaw = None
        self.obs_centroid = None

    def track_obstacle(self) :
        obj_topics = {
            'cap_r': '/objects/capture_vehicle/rear/gps/rtkfix',
            'cap_f': '/objects/capture_vehicle/front/gps/rtkfix',
            'obs_r': '/objects/obs1/rear/gps/rtkfix'
        }

        for obj in obj_topics:
            rospy.Subscriber(obj_topics[obj],
                         Odometry,
                         self.handle_msg,
                         obj)
            

    def handle_msg(self, msg, who):
        assert isinstance(msg, Odometry)
    
        now = rospy.get_rostime()
        if now < self.current_time :
            self.reset()
        self.current_time = now
    
        if who == 'cap_r':
            self.last_cap_r = rtk_position_to_numpy(msg)
        elif who == 'cap_f' and self.last_cap_r is not None:
            cap_f = rtk_position_to_numpy(msg)
            cap_r = self.last_cap_r
    
            self.last_cap_f = cap_f
            self.last_cap_yaw = get_yaw(cap_f, cap_r)
        elif who == 'obs_r' and self.last_cap_f is not None and self.last_cap_yaw is not None:
            md = None
            for obs in self.metadata:
                if obs['obstacle_name'] == 'obs1':
                    md = obs
            assert md, 'obs1 metadata not found'
    
            # find obstacle rear RTK to centroid vector
            lrg_to_gps = [md['gps_l'], -md['gps_w'], md['gps_h']]
            lrg_to_centroid = [md['l'] / 2., -md['w'] / 2., md['h'] / 2.]
            obs_r_to_centroid = np.subtract(lrg_to_centroid, lrg_to_gps)
    
            # in the fixed GPS frame 
            cap_f = self.last_cap_f
            obs_r = rtk_position_to_numpy(msg)
            
            # in the capture vehicle velodyne frame
            cap_to_obs = np.dot(rotMatZ(-self.last_cap_yaw), obs_r - cap_f)
            cap_to_obs_centroid = cap_to_obs + obs_r_to_centroid
            velo_to_front = np.array([-1.0922, 0, -0.0508])
            cap_to_obs_centroid += velo_to_front
            self.obs_centroid = cap_to_obs_centroid


    def add_bbox(self):
        inputName = '/image_raw'
        rospy.Subscriber(inputName, Image, self.handle_img_msg, queue_size=1)
         

    def handle_img_msg(self, img_msg):
        img = None
        bridge = cv_bridge.CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr( 'image message to cv conversion failed :' )
            rospy.logerr( e )
            print( e )
            return
    
        tx, ty, tz, yaw, pitch, roll = [0.00749025, -0.40459941, -0.51372948, 
                                        -1.66780896, -1.59875352, -3.05415572]
        translation = [tx, ty, tz, 1]
        rotationMatrix = tf.transformations.euler_matrix(roll, pitch, yaw)
        rotationMatrix[:, 3] = translation
        md = self.metadata
        md = None
        for obs in self.metadata:
            if obs['obstacle_name'] == 'obs1':
                md = obs
        assert md, 'obs1 metadata not found'
        dims = np.array([md['l'], md['w'], md['h']])
        outputName = '/image_bbox'
        imgOutput = rospy.Publisher(outputName, Image, queue_size=1)
        obs_centroid = self.obs_centroid
   
        if self.obs_centroid is None:
            rospy.loginfo("Couldn't find obstacle centroid")
            imgOutput.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
            return
        
        # print centroid info
        rospy.loginfo(str(obs_centroid))

        # case when obstacle is not in camera frame
        if obs_centroid[0]<2.5 :
            imgOutput.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
            return
    
        # get bbox 
        corners = [obs_centroid  +0.5*np.array([i,j,k])*dims for i in [-1,1] 
                    for j in [-1,1] for k in [-1,1]]
        projected_pts = []
        cameraModel = PinholeCameraModel()
        cam_info = load_cam_info(self.calib_file)
        cameraModel.fromCameraInfo(cam_info)
        for pt in corners:
            rotated_pt = rotationMatrix.dot(list(pt)+[1])
            projected_pts.append(cameraModel.project3dToPixel(rotated_pt))
        projected_pts = np.array(projected_pts)
        center = np.mean(projected_pts, axis=0)
        out_img = drawBbox(img, projected_pts)
        imgOutput.publish(bridge.cv2_to_imgmsg(out_img, 'bgr8'))


if __name__ == "__main__" :
    argv = rospy.myargv()
    rospy.init_node('projection')
    assert len(argv) == 3, 'usage: \n{} <bag_file> <calib_file>'.format(argv[0])
    
    bag_dir = os.path.dirname(argv[1])
    md_path = os.path.join(bag_dir, 'metadata.csv')
    calib_file = argv[2]
    assert os.path.isfile(md_path), 'Metadata file %s does not exist' % md_path
    assert os.path.isfile(calib_file), 'Calibration file %s does not exist' % calib_file

    try :
        p = Projection(md_path, calib_file)    
        p.track_obstacle()
        p.add_bbox()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

