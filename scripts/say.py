#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

soundhandle = None

def speak(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    soundhandle.say(data.data)


def say_something():
    soundhandle = SoundClient()
    rospy.init_node('say', anonymous=True)
    rospy.Subscriber("chatter", String, speak)
    rospy.sleep(1) #give time for soundplay node to initialize
    soundhandle.say('Is this thing on?')
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    say_something()


