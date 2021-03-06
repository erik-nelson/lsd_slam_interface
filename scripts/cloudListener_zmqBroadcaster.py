#! /usr/bin/python
import numpy as np
import os

import rospy

from sensor_msgs.msg import PointCloud2
from python_msg_conversions import pointclouds

import sys
from ZmqClass import ZmqEmitter

DS = 1  # Downsample factor for pt cloud

class PointCloudListener():
    '''Creates an object that listens on the rgbdslam/batchclouds topic for 
     point clouds.'''

    def __init__(self):
        self.cloud_arr = []
        self.emitter = ZmqEmitter("5558", topic="pointCloud")
        self.ctr = 0  # For down sampling
        print 'Initializing point cloud listener...'
        rospy.init_node('ptcloud_listener', anonymous=True)
        print 'Done.'
        print 'Subscribing to lsd_slam_interface/point_cloud...'
        rospy.Subscriber("lsd_slam_interface/point_cloud", PointCloud2,
                         self.callback)
        print 'Done.'
        rospy.spin()

    def callback(self, cloud_msg):
        '''Function that the subscriber calls when it gets a new message'''
        # Get and format the point cloud
        self.cloud_arr = pointclouds.pointcloud2_to_array(cloud_msg,
                                                          split_rgb=True)
        self.cloud_arr = self.cloud_arr[np.isfinite(self.cloud_arr['z'])]
        self.ctr += 1
        # Send the pcld out
        print 'Sending ptcloud out on network...'
        tic = rospy.get_time()
        self.emitter.send_zipped_pickle(self.cloud_arr[::1])
        toc = rospy.get_time()
        print 'Ptcld sent. Elapsed time = %s' % (toc - tic)

    def getAggregateCloudSent(self, msg):
        print 'Message = %s' % msg
        rospy.wait_for_service('/rgbdslam/ros_ui')
        self.ros_ui('quick_save')

    def saveCloud(self, msg):
        print 'Saving cloud...'
        np.save('/home/grim4/Desktop/ptcloud.npy', self.cloud_arr)
        print 'Done.'


if __name__ == '__main__':
    l = PointCloudListener()
    # rospy.on_shutdown( l.saveCloud )
