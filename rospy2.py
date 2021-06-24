#!/usr/bin/env python3

import rclpy
import sys

DEBUG = ?
INFO = ?

rclpy.init(args = sys.argv)

node = None
logger = None

def init_node(node_name, anonymous=False, log_level=rospy.INFO, disable_signals=False):
    global node, logger
    node = rclpy.create_node(node_name)
    logger = node.get_logger()

def is_shutdown():
    pass

def logdebug(log_text):
    global node, logger
    logger.debug(log_text)

def loginfo(log_text):
    global node, logger
    logger.info(log_text)

def logwarn(log_text):
    global node, logger
    logger.warn(log_text)

def logerr(log_text):
    global node, logger
    logger.error(log_text)

def logfatal(log_text):
    global node, logger
    logger.fatal(log_text)

def on_shutdown(h):
    pass

def signal_shutdown(reason):
    pass

def spin():
    global node, logger
    rclpy.spin(node)

class Publisher(object):
    pass

class Subscriber(object):
    pass

class Service(object):
    pass

class ServiceProxy(object):
    pass


