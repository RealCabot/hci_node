# HCI Node

## Dependency Installation
```
rosdep install hci_node
```

## Description
This node warns the user before and after a turn through text-to-speech.

### Corner Extractor Node
```
rosrun hci_node corner_extractor.py
```
This node will listen to global plan published by navigation stack, extract the corners and publish them. The paramters can be configured in `__init__` in `corner_extractor.py`

#### Subscribed

- `/move_base/NavfnROS/plan`: `nav_msgs::Path`

#### Published

- `/corners`: `geometry_msgs::PoseArray`

### Sound Play Node (Original HCI Node)
The launch file launches a sound_play node (which plays the sound) and then launches a say_node (which receives waypoint messages).

#### Running node
```
roslaunch hci_node hci.launch 
``` 
In a separate terminal:
```
rostopic pub /waypt hci_node/waypoint '{pointNum: 1, dist: .5}' -1
```
If you didn't hear anything after entering the last command, check your speaker volume and speaker drivers.

#### Subscribed

 - `waypt` - `hci_node::waypoint` messages published by local_planner

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
