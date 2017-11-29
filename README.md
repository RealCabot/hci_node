## HCI Node

### Installation
```
sudo apt-get install ros-kinetic-sound-play
```
In your `<project_workspace>/src/` folder:
```
git clone https://github.com/RealCabot/hci_node
```

### Description
This node warns the user before and after a turn. 

Currently, the launch file launches a sound_play node (which plays the sound) and then launches a say_node (which receives waypoint messages).

#### Subscribes to
 - `waypt` - "waypoint" messages published by local_planner

#### Messages
 - `waypoint` message, containing the waypoint CaBot is trying to reach and the current distance from the waypoint


### Running node
```
roslaunch hci_node hci.launch 
``` 
In a separate terminal:
```
rostopic pub /waypt hci_node/waypoint '{pointNum: 1, dist: .5}' -1
```
If you didn't hear anything after entering the last command, check your speaker volume and speaker drivers.

### Troubleshooting
Please refer to the tutorials/troubleshooting in the sound_play ROS library:
http://wiki.ros.org/sound_play
