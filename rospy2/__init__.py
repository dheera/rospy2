#!/usr/bin/env python3

import rclpy
import sys
from constants import *

rclpy.init(args = sys.argv)

# Variables starting with an underscore are stored ROS2 objects and are not
# intended to be used directly.
# All normally-named functions/classes/methods are designed to function exactly
# as per their ROS1 namesake.

_node = None
_logger = None

def get_param(param_name, default_value = None):
    global _node, _logger
    return _node.get_parameter_or(param_name, default_value)._value

def init_node(node_name, anonymous=False, log_level=rospy.INFO, disable_signals=False):
    global _node, _logger
    _node = rclpy.create_node(
        node_name,
        allow_undeclared_parameters = True,
        automatically_declare_parameters_from_overrides = True,
    )
    _logger = _node.get_logger()

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

def set_param(parameter_name, parameter_value):
    if type(parameter_value) is str:
        parameter_type = rclpy.Parameter.Type.STRING
    elif type(parameter_value) is float:
        parameter_type = rclpy.Parameter.Type.DOUBLE
    elif type(parameter_value) is int:
        parameter_type = rclpy.Parameter.Type.INT
    elif type(parameter_value) is bool:
        parameter_type = rclpy.Parameter.Type.BOOL
    else:
        raise Exception("Invalid parameter value type: %s" % str(type(parameter_value)))

    param = rclpy.parameter.Parameter(
        "parameter_name",
        parameter_type,
        parameter_value,
    )
    _node.set_parameters([param])

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
    def __init__(self, topic_type, topic_name):
        self._pub = _node.create_publisher(topic_type, topic_name)

    def __del__(self):
        pass

    def publish(self, msg):
        self._pub.publish(msg)

class Subscriber(object):
    def __init__(self, topic_name, topic_type, callback, args = ()):
        self._sub = _node.create_subscription(topic_type, topic_name, callback)

    def __del__(self):
        _node.destroy_subscription(self._sub)

    def unregister(self):
        _node.destroy_subscription(self._sub)

class Service(object):
    def __init__(self, service_name, service_type, callback):
        self._srv = _node.create_service(service_type, service_name, callback)

    def __del__(self):
        pass

class ServiceProxy(object):
    def __init__(self, service_name, service_type):
        self._client = _node.create_client(service_type, service_name)

    def __del__(self):
        pass
    
    def __call__(self, req):
        resp = self._client.call_async(req)
        rclpy.spin_until_future_complete(_node, resp)
        return resp

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

