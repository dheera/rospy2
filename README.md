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

You should then be able to just run your node and it should work on ROS2.

