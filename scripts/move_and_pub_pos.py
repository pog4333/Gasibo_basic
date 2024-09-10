#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def callback(msg):
    print(f"X = {msg.pose.pose.position.x}, Y= {msg.pose.pose.position.y}")
    # rospy.spin()

def main():
    print("work")
    sub = rospy.Subscriber("/odom", Odometry, callback)
    pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
    rospy.init_node("demo_node")
    rate =rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Twist()
        msg.angular.z = 0.3
        msg.linear.x =0.3
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()

