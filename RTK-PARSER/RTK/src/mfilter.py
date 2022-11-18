#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from mi_msgs.msg import *
from sensor_msgs.msg import Image


image_pub = rospy.Publisher('/sync_image_raw', Image, queue_size=1)
RTK_msg_pub = rospy.Publisher('/sync_RTK_messages', RTK, queue_size=1)
gps_pub = rospy.Publisher('/sync_gps_messages', GPS, queue_size=1)

def m_filters_callback(t1_msg, t2_msg):
    print("SYNCRONIZATION")
    image_pub.publish(t1_msg)
    #RTK_msg_pub.publish(t2_msg)
    gps_pub.publish(t2_msg)

if __name__ == '__main__':

    rospy.init_node('message_filter',anonymous=True)
    
    t1_sub = message_filters.Subscriber("/image_raw", Image)
    t2_sub = message_filters.Subscriber("/RTK_messages", RTK)
    t3_sub = message_filters.Subscriber("/gps_messages", GPS)

    #message_filters.ApproximateTimeSynchronizer([subsciber], Queue size ,Time interval, allow headerless Flag(True or False))
    #m_filters = message_filters.ApproximateTimeSynchronizer([t1_sub, t2_sub], 10 , 0.1, allow_headerless=False)
    m_filters = message_filters.ApproximateTimeSynchronizer([t1_sub, t3_sub], 10 , 0.1, allow_headerless=False)
    
    m_filters.registerCallback(m_filters_callback)
    rospy.spin()



