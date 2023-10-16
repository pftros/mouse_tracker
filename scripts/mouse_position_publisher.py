#!/usr/bin/env python3

from pynput.mouse import Controller

import rospy
from geometry_msgs.msg import Point


class MousePositionPublisher:
    def __init__(self):
        self.position_pub = rospy.Publisher("mouse_position", Point, queue_size=1)
        self.mouse = Controller()
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.publish_mouse_position)

    def publish_mouse_position(self, _):
        curr_position = self.mouse.position
        position_msg = Point()
        position_msg.x = curr_position[0]
        position_msg.y = curr_position[1]
        self.position_pub.publish(position_msg)


def main():
    rospy.init_node("mouse_position_publisher", anonymous=True)
    mouse_position_publisher = MousePositionPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()

