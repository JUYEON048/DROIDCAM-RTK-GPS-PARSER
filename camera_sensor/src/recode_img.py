#!/home/j/.pyenv/versions/rospy37/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('./output.jpg', fourcc, 20.0, (3840,2160))
count = 0
def image_callback(msg):
    bridge = CvBridge()
    frame_ = bridge.imgmsg_to_cv2(msg, "bgr8")
    print(frame_.shape)
    #cv2.imshow("fram",frame_)
    filename = './recodetest'+str(count)+'.png'
    cv2.imwrite(filename,frame_)
    #key = cv2.waitKey(1)
    ++count

def main():
    rospy.init_node("cv2_to_imgmsg")
    image_pub = rospy.Subscriber('/image_raw', Image, image_callback)
    rospy.spin()
    
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
