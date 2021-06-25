#!/usr/bin/env python3

import rospy2 as rospy

from geometry_msgs.msg import *
from std_msgs.msg import *

print("Testing ROS1 constructors for common data types")

print("Point")
print(Point())
print(Point(1,2,3))
print(Point(1.0, 2.0, 3.0))
print(Point(x=1, y=2, z=3))
print(Point(x=1.0, y=2.0, z=3.0))

print("Quaternion")
print(Quaternion())
print(Quaternion(1,2,3, 4))
print(Quaternion(1.0, 2.0, 3.0, 4.0))
print(Quaternion(x=1, y=2, z=3, w=4))
print(Quaternion(x=1.0, y=2.0, z=3.0, w=4.0))

print("Int8")
print(Int8())
print(Int8(1))

print("UInt8")
print(UInt8())
print(UInt8(1))

print("Int16")
print(Int16())
print(Int16(1))

print("UInt16")
print(UInt16())
print(UInt16(1))

print("Int32")
print(Int32())
print(Int32(1))

print("UInt32")
print(UInt32())
print(UInt32(1))

print("Int64")
print(Int64())
print(Int64(1))

print("UInt64")
print(UInt64())
print(UInt64(1))

print("Float32")
print(Float32())
print(Float32(1))
print(Float32(1.0))

print("Float64")
print(Float64())
print(Float64(1))
print(Float64(1.0))

print("Success")
