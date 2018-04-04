# HCI Node

- [HCI Node](#hci-node)
    - [Dependency Installation](#dependency-installation)
    - [Description](#description)
        - [Corner Extractor Node](#corner-extractor-node)
            - [Subscribed](#subscribed)
            - [Published](#published)
        - [Pedestrian Warner Node](#pedestrian-warner-node)
            - [Subscribed](#subscribed)
            - [Published](#published)
            - [Called Service](#called-service)
        - [Messages](#messages)
        - [ROS Parameters](#ros-parameters)
        - [What if I want to contribute without Cabot?](#what-if-i-want-to-contribute-without-cabot)
        - [Troubleshooting](#troubleshooting)


## Dependency Installation
```
rosdep install hci_node
```

## Description
This package contains several nodes that all serve for HCI purposes, including

- [Corner Extractor Node](#corner-extractor-node): Extract corner infor from Path
- [Sound Play Node](#sound-play-node): warns the user before and after a turn through text-to-speech
- [Pedestrian Warner Node](#pedestrian-warner-node): warns the user about pedestrian ahead and slow down Cabot

### Corner Extractor Node
```
rosrun hci_node corner_extractor.py
```
This node will listen to global plan published by navigation stack, extract the corners and publish them. The paramters can be configured in `__init__` in `corner_extractor.py`

#### Subscribed

- `/move_base/NavfnROS/plan`: `nav_msgs::Path`

#### Published

- `/corners`: `geometry_msgs::PoseArray`

### Pedestrian Warner Node

This node read pedestrians detected by Kinect, then make sound to tell the user. It also slow the robot down.

#### Subscribed

- `/people_points`: `kinect2_tracker::user_points`, center of mass for each person

#### Published

- `/say`: `std_messages::String`, things for GUI to say

#### Called Service

- `/move_base/DWAPlannerROS/set_parameters`, Dynamic Reconfigure service


### Messages
 - `waypoint` message, containing the waypoint CaBot is trying to reach and the current distance from the waypoint

### ROS Parameters
 - `turn_threshold` - maximum angle (in radians) before user is warned about turn
 - `warn_threshold` - maximum distance (in meters) before user is warned about turn
 - `isFeet` -  True = use feet for distance units, False = use meters as distance units

### What if I want to contribute without Cabot?

We get you covered. Cabot has a nice simulator. Just run `roslaunch localizer simulated_planner.launch`, and you will get everything you want! (except lidar scan data) You can add destination and pose estimation directly through `rviz`

### Troubleshooting
- For soundplay, please refer to the tutorials/troubleshooting in the [sound_play ROS library](http://wiki.ros.org/sound_play)
- For corner extractor, ask Yanda
