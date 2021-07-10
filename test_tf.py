#!/usr/bin/env python3

try:
    import rospy
except ImportError:
    import rospy2 as rospy

import tf2_ros
from std_msgs.msg import String

class TestROS1Node(object):
    def __init__(self):
        rospy.init_node("test_node", log_level = rospy.DEBUG)
        rospy.logdebug("initializing")
        self.rate = rospy.Rate(10)
        self.count = 0

    def start(self):
        self.b = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()

        while not rospy.is_shutdown():
            self.rate.sleep()
            self.count += 1

        rospy.logwarn("shutting down")

    def on_message(self, msg):
        rospy.loginfo("received message: %s" % msg.data)

if __name__ == "__main__":
    TestROS1Node().start()

