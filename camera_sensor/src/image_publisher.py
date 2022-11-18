#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

def main():
    rospy.init_node("camer_topic")
    rate = rospy.Rate(60)
    count = 0
    cap = cv2.VideoCapture(0)
    #size = cap.size[:2]
    #print(size)
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('./output.avi', fourcc, 20.0, (1920,1080))
    bridge = CvBridge()
    image_pub = rospy.Publisher('/image_raw', Image, queue_size=1)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            #frame = cv2.flip(frame,1)
            #out.write(frame)
            print(frame.shape)
            image_pub.publish(img_msg)
            frame_resize = cv2.resize(frame,dsize=(1280,720),interpolation=cv2.INTER_AREA)

        rate.sleep()
    cap.release()
    #out.release()
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
