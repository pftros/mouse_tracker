#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose

from math import sqrt, atan2, cos, sin, pi

class TurtleController:

    def __init__(self):
        self.cmd_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.cmd_msg = Twist()
        self.mouse_position_sub = rospy.Subscriber("/mouse_position", Point, self.mouseCallback, queue_size=1)
        self.turtle_pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.turtlePoseCallback, queue_size=1)

        self.pose = Pose()
        self.pose.x = 5.55
        self.pose.y = 5.55

        rospy.spin()

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2)
        + pow((goal_y - self.pose.y), 2))
        return distance

    def get_ang_distance(self, goal_x, goal_y):
        ang_distance = (atan2(goal_y - self.pose.y,
        goal_x - self.pose.x) - self.pose.theta)
        return ang_distance

    def mouseCallback(self, msg):        
        screen_x = rospy.get_param("res_x")
        screen_y = rospy.get_param("res_y")

        mouse_x = (msg.x / screen_x) * 11.08
        mouse_y = ((screen_y - msg.y) / screen_y) * 11.08

        distance = self.get_distance(mouse_x, mouse_y)
        angle = self.get_ang_distance(mouse_x, mouse_y)

        self.cmd_msg.linear.x = distance*cos(angle)
        self.cmd_msg.linear.y = distance*sin(angle)
        self.cmd_msg.angular.z = angle

        self.cmd_pub.publish(self.cmd_msg)

    def turtlePoseCallback(self, msg):
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta

def main():
  rospy.init_node('turtle_controller', anonymous=True)
  turtle_controller = TurtleController()

if __name__ == '__main__':
    main()
