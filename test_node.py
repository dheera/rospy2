#!/usr/bin/env python3

try:
    import rospy
except ImportError:
    import rospy2 as rospy

from std_msgs.msg import String

class TestROS1Node(object):
    def __init__(self):
        rospy.init_node("test_node", log_level = rospy.DEBUG)
        rospy.logdebug("initializing")
        self.param_rate = rospy.get_param("rate", 10.0)
        self.sub_message = rospy.Subscriber("/message", String, self.on_message)
        self.pub_message = rospy.Publisher("/message", String, queue_size = 1)
        self.count = 0
        rospy.loginfo_once("this is a info message that is supposed to only appear once")
        rospy.loginfo_once("this is a info message that is supposed to only appear once")

    def start(self):
        self.rate = rospy.Rate(self.param_rate)
        self.pub_message.publish("message that uses a ros1 shortcut of directly passing a Python str to a publisher")
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.count += 1
            self.pub_message.publish(String(data = "foo %d" % self.count))
            rospy.logwarn_throttle(2, "this is a warning message that should appear at most every 2 seconds")

        rospy.logwarn("shutting down")

    def on_message(self, msg):
        rospy.loginfo("received message: %s" % msg.data)

if __name__ == "__main__":
    TestROS1Node().start()

