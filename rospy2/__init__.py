#!/usr/bin/env python3

import rclpy
import sys
from constants import *

rclpy.init(args = sys.argv)

_node = None
_logger = None

def get_param():
    pass

def init__node(_node_name, anonymous=False, log_level=rospy.INFO, disable_signals=False):
    global _node, _logger
    _node = rclpy.create__node(_node_name)
    _logger = _node.get__logger()

def is_shutdown():
    pass

def logdebug(log_text):
    global _node, _logger
    _logger.debug(log_text)

def loginfo(log_text):
    global _node, _logger
    _logger.info(log_text)

def logwarn(log_text):
    global _node, _logger
    _logger.warn(log_text)

def logerr(log_text):
    global _node, _logger
    _logger.error(log_text)

def logfatal(log_text):
    global _node, _logger
    _logger.fatal(log_text)

def on_shutdown(h):
    pass

def set_param():
    pass

def signal_shutdown(reason):
    pass

def sleep():
    pass

def spin():
    global _node, _logger
    rclpy.spin(_node)

def timer():
    pass

def wait_for_message():
    pass

def wait_for_service():
    pass

class Publisher(object):
    pass

class Subscriber(object):
    pass

class Service(object):
    pass

class Duration(object):
    pass

class Time(object):
    pass

class Timer(object):
    pass

class ServiceProxy(object):
    pass

class ROSException(Exception):
    pass

class ROSException(ROSException):
    pass

class ROSInternalException(ROSException):
    pass

class ROSInterruptException(ROSException):
    pass

class ROSSerializationException(ROSException):
    pass

class ROSTimeMovedBackwardsException(ROSException):
    pass

class ServiceException(Exception):
    pass

class TransportException(Exception):
    pass

class TransportInitError(Exception):
    pass

class TransportTerminated(Exception):
    pass

