#!/usr/bin/env python3

import rospy
import time
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.srv import TeleportAbsoluteResponse

def turtleSetPen(r, g, b, width, off):
    rospy.wait_for_service('/turtle1/set_pen')
    try:
        turtle_setpen_ = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        resp_ = turtle_setpen_(r, g, b, width, off)
        rospy.loginfo(f"Pen set to RGB({r}, {g}, {b}), Width: {width}.")
        return resp_
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def turtleGotoPose(x: float, y: float, theta: float):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    rospy.loginfo(f'Goto => x: {x} y: {y} theta: {theta}')
    try:
        turtle_gotoPose_ = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        resp_ = turtle_gotoPose_(x, y, theta)
        time.sleep(1.0) 
        return resp_
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return TeleportAbsoluteResponse()

def main():
    rospy.init_node('turtle_move_node')
    
    rospy.loginfo("Waiting for Turtlesim services...")
    
    turtleSetPen(0, 0, 0, 3, 0)
    
    rospy.loginfo("Starting turtle movement...")
    
    turtleGotoPose(6.0, 2.0, 1.57)
    
    turtleGotoPose(9.0, 5.0, 3.14)
    
    turtleGotoPose(6.0, 8.0, 4.71)
    
    turtleGotoPose(3.0, 5.0, 6.28)

    turtleGotoPose(6.0, 2.0, 1.57)

    rospy.loginfo("Turtle movement complete.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
