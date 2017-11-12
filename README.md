## HCI Node

### Installation
```
sudo apt-get install ros-kinetic-sound-play
```
In your "<project_workspace>/src/" folder:
```
git clone https://github.com/RealCabot/hci_node
```

### Description
This node does text-to-speech on directions ("turn left") from the local planner 
Subscribes to "text-to-speech" topic

Currently, the launch file launches a sound_play node (which plays the sound) and then launches a say_node (which receives text). There should be a way to combine these two nodes into one node.

### Running node
```
roslaunch hci_node hci.launch 
``` 
In a separate terminal:
```
rostopic pub /speak std_msgs/String '{data: "hallow from the other side"}' -1
```
If you didn't hear anything after entering the last command, check your speaker volume and speaker drivers.

### Troubleshooting
Please refer to the tutorials/troubleshooting in the sound_play ROS library:
http://wiki.ros.org/sound_play
