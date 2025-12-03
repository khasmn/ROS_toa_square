#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

PI = 3.1415926535897
speed = 2.0
angular_speed = 1.5
side_length = 2.0 # length of each side of the square

current_pose = None

def pose_callback(msg):
    global current_pose
    current_pose = msg

if __name__ == '__main__':
    rospy.init_node('draw_square', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    vel_msg = Twist()
    rate = rospy.Rate(20)

    # Wait for first pose
    while current_pose is None and not rospy.is_shutdown():
        rate.sleep()

    for _ in range(4):
        # Save starting position
        start_x = current_pose.x
        start_y = current_pose.y
        # Move forward until distance >= side_length
        while not rospy.is_shutdown():
            dx = current_pose.x - start_x
            dy = current_pose.y - start_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= side_length:
                break
            vel_msg.linear.x = speed
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            rate.sleep()
        vel_msg.linear.x = 0
        pub.publish(vel_msg)
        rospy.sleep(0.5)

        # Save starting angle
        start_theta = current_pose.theta
        # Turn until rotated 90 degrees
        while not rospy.is_shutdown():
            angle_turned = abs(current_pose.theta - start_theta)
            # Handle wrap-around
            if angle_turned > PI:
                angle_turned = 2*PI - angle_turned
            if angle_turned >= PI/2.0:
                break
            vel_msg.linear.x = 0
            vel_msg.angular.z = angular_speed
            pub.publish(vel_msg)
            rate.sleep()
        vel_msg.angular.z = 0
        pub.publish(vel_msg)
        rospy.sleep(0.5)

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)
    print("Square drawing complete!")
