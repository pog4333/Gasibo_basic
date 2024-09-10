#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    r = msg.ranges[1]
    for range in msg.ranges:
        if range < r :
            r = range
    print(f"min range = {r}")

def main():
    sub = rospy.Subscriber("laser/scan",LaserScan,callback)
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    rospy.init_node("min_laser")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Twist()
        msg.angular.z = 0.3
        msg.linear.x = 1
        pub.publish(msg)
        rate.sleep

if __name__ == "__main__":
    main()