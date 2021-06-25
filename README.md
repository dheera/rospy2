# rospy2: A ROS1-like interface for ROS2

Many people are still using ROS1 because it takes time to rewrite all your nodes for ROS2.

There is a [ROS1-ROS2 bridge](https://github.com/ros2/ros1_bridge) but it requires installing both ROS1 and ROS2 at the same time.

This provides a radically different solution: Your ROS1 code can work directly on the ROS2 network, as long as you aren't doing any crazy things.

All you need to do is install this package to your system:
```
sudo python3 setup.py install
```

And then in your node, change the line
```
import rospy
```

to
```
import rospy2 as rospy
```

You can even make nodes that work in both ROS1 and ROS2:
```
try:
    import rospy
except ImportError:
    import rospy2 as rospy
```

You should then be able to just run your node and it should work on ROS2. A test node is provided:

```
source /opt/ros/foxy/setup.bash
./test_node.py
```

# Status

## Supported

* Publishers

* Subscribers

* Rate

* Parameters in the same namespace as the node

## Not supported yet

* Parameters in other namespaces

* Service calls (supported but not tested)

## Known issues

* Some *message* types changed between ROS1 and ROS2. For example rosgraph_msgs/Log is now rcl_interfaces/Log and the message definition is slightly different.

* There is no "parameter server" in ROS2, so ROS1 nodes that expect global parameters aren't going to work.

* Since your callbacks will receive actual ROS2 messages, those that have numeric arrays will see them as an array.array or numpy.ndarray instead of a Python list. This may trip up some code that expects a Python list.

