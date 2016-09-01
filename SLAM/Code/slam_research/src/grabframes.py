#!/usr/bin/python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import string
import roslib
from tf2_msgs.msg import TFMessage
import tf
import numpy as np
from geometry_msgs.msg import Twist



def rgb_cb(rgb):

    bridge = CvBridge()
    try:
      rgb_image = bridge.imgmsg_to_cv2(rgb,"bgr8")
    except CvBridgeError as e:
      print(e)

    rgb_image = np.array(rgb_image,dtype=np.uint8)
    gray_image = cv2.cvtColor(rgb_image,cv2.COLOR_BGR2GRAY)

    cv2.imwrite(str1,gray_image,None)
    # print " returning from rgb callback"

    image_sub.unregister()


def depth_cb(depth):

    bridge = CvBridge()
    try:
      depth_image = bridge.imgmsg_to_cv2(depth)
    except CvBridgeError as e:
      print(e)

    depth_image = np.array(depth_image, dtype=np.float32)


    np.save(str2,depth_image)
    # print " returning from depth callback"
    image_sub1.unregister()


def cb(data):
    global P, Q
    if (listener.frameExists("/base_link") and listener.frameExists("/odom")):
        P, Q = listener.lookupTransform("/base_link", "odom", rospy.Time(0))
        t.unregister()

def main(argv):
    global image_sub, image_sub1, str1, str2 ,count,listener,t
    count = 0
    loop = argv
    nub=str(argv)
    str1= nub + '.png'
    str2= nub + '.npy'
    rospy.init_node('grabframes', anonymous=True)
    # x_speed = 0.1  # 0.5 m/s
    # first thing, init a node!
    r = rospy.Rate(5.0)
    # publish to cmd_vel
    p = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=100)
    move=np.array(([0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0],[0.1,0.0]),dtype=float)
    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = move[loop][0];  # our forward speed
    twist.linear.y = 0;
    twist.linear.z = 0;  # we can't use these!
    twist.angular.x = 0;
    twist.angular.y = 0;  # or these!
    twist.angular.z = move[loop][1];  # no rotation

    # announce move, and publish the message

    # the value of range and the rate has to be determined
    # rospy.loginfo("About to be moving forward!")
    for i in range(25):
        p.publish(twist)
        r.sleep()
    # create a new message
    twist = Twist()

    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    # rospy.loginfo("Stopping!")
    p.publish(twist)
    image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, rgb_cb)
    image_sub1 = rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_cb)
    t = rospy.Subscriber("tf", TFMessage, cb)
    listener = tf.TransformListener()
    rospy.sleep(5)
    return P, Q


if __name__ == '__main__':
        main()




