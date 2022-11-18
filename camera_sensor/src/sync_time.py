#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import PointCloud2, Image
import message_filters
import numpy as np
import rosbag

"""
Publisher test 노드 
"""

bag = rosbag.Bag('test.bag', 'w')
count = 0
def callback(msg1, msg2, pub_img, pub_lidar):
    pub_img.publish(msg1)
    pub_lidar.publish(msg2)
    
def main():
    rospy.init_node('sync')
    image_sub = message_filters.Subscriber('image_raw', Image)
    lidar_sub = message_filters.Subscriber('velodyne_points', PointCloud2)
    pub_img = rospy.Publisher('image_sync',Image,queue_size=1)
    pub_lidar = rospy.Publisher('points_sync',PointCloud2,queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, lidar_sub], 1, 0.01, allow_headerless=True)
    ts.registerCallback(callback,pub_img,pub_lidar)
    rospy.spin()


if __name__ == "__main__":


    try:
        main()

    except rospy.ROSInterruptException:
        pass
