#! /usr/bin/python
import numpy as np
import os
#import roslib
import rospy
import math
import tf
from nav_msgs.msg import Path
# from sensor_msgs.msg import PointCloud2

# For zmq
import sys
from ZmqClass import ZmqEmitter

ttRType = np.dtype({"names": ['timestamp', 'tx', 'ty', 'tz', 'ROO', 'R01',
                             'R02', 'R10', 'R11', 'R12', 'R20', 'R21', 'R22'], 
                    "formats": [np.float64, np.float32, np.float32, np.float32,\
                                np.float32, np.float32, np.float32, np.float32,\
                                np.float32, np.float32, np.float32, np.float32,\
                                np.float32]})

class PathListener():
    '''Creates an object that listens on the rgbdslam/batchclouds topic for 
     point clouds.'''

    def __init__(self):
        self.cloud_arr = []
        self.emitter = ZmqEmitter("5558", topic="pointCloud")
        self.ctr = 0  # For down sampling
        print 'Initializing point cloud listener...'
        rospy.init_node('ptcloud_listener', anonymous=True)
        print 'Done.'
        print 'Subscribing to lsd_slam_interface/path...'
        rospy.Subscriber("lsd_slam_interface/poses", Path,
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
    l = PathListener()
    # rospy.on_shutdown( l.saveCloud )

if __name__ == '__main__':
    # Init rospy node
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    emitter = ZmqEmitter("5557", 'poses')
    RTdata = np.empty(0, dtype=ttRType)
    ctr = 0
    rate = rospy.Rate(10.0)

    outstr = '#timestamp\ttx\tty\ttz\tR00\tR01\tR02\tR10\tR11\tR12\tR20\tR21\tR22\n'
    while not rospy.is_shutdown():
        try:
            ts = rospy.get_time()
            (trans, rot) = listener.lookupTransform('/camera_link', '/map',\
                                                   rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,\
                tf.ExtrapolationException):
            continue
        # Convert pose to RT
        rot = tf.transformations.quaternion_matrix(rot)
        R = rot[0:3, 0:3]
        # Format data properly for broadcasting
        newPose = np.array([(ts, ) + trans + tuple(R.flatten())],\
                            dtype=ttRType)
        # Handle the first pose estimate
        if len(RTdata) < 1:
            RTdata = np.concatenate((RTdata, newPose))
            continue
        last_RT = RTdata[-1]
        # Check against distance threshold in attempt to enforce a smooth
        # detector track
        inter_pose_dist = np.sqrt((newPose['tx'] - last_RT['tx']) ** 2 +\
			           (newPose['ty'] - last_RT['ty']) ** 2 +\
				   (newPose['tz'] - last_RT['tz']) ** 2)
        # HACK - Check for stopping condition
        if newPose['tx'] != last_RT['tx'] or newPose['ty'] != last_RT['ty'] or\
	   newPose['tz'] != last_RT['tz']:
            RTdata = np.concatenate((RTdata, newPose))
            # Emit the data, but only once a second
            if ctr % 10 == 0:
                emitter.send_zipped_pickle(RTdata)
            ctr += 1

            # Write to .ros/log std-log
            outstr += '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n'\
		      % (ts, trans[0], trans[1], trans[2], rot[0][0], rot[0][1],\
		      rot[0][2], rot[1][0], rot[1][1], rot[1][2], rot[2][0],\
		      rot[2][1], rot[2][2])

        rate.sleep()

