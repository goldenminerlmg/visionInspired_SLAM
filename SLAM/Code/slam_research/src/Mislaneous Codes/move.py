#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist



def main():
    x_speed = 0.1  # 0.5 m/s
    # first thing, init a node!
    rospy.init_node('move')
    r = rospy.Rate(5.0)
    # publish to cmd_vel
    p = rospy.Publisher('RosAria/cmd_vel', Twist,queue_size=100)

    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = x_speed;                   # our forward speed
    twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!
    twist.angular.x = 0; twist.angular.y = 0;   #          or these!
    twist.angular.z = 0;                        # no rotation

    # announce move, and publish the message

    # the value of range and the rate has to be determined
    rospy.loginfo("About to be moving forward!")
    for i in range(50):
        p.publish(twist)
        r.sleep() # 10*5hz = 2s

    # create a new message
    twist = Twist()

    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    rospy.loginfo("Stopping!")
    p.publish(twist)

if __name__=="__main__":
    main()
