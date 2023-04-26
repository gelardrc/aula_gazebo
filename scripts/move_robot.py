#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

class agent:
    def __init__(self):
        pass


def move():
    while not (rospy.is_shutdown()):
        continue
    pass
if __name__ == "__main__":
    rospy.init_node('move',anonymous=False)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped)
    move()