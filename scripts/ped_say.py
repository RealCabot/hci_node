#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
import numpy as np

from geometry_msgs import *

from math import *
from kinect2_tracker.msg import user_points

soundhandle = None
warnThreshold = None  # distance to point (in meters) before we warn the user

def getPed(msg):
    global soundhandle, warnThreshold

    peds = msg.people_points
    if len(peds) > 0: #number of pedestrians

        for p in peds:
            newPt = p.point.z #z is depth??
            if newPt < warnThreshold:

                warnMsg = "Pedestrains ahead"
                print warnMsg
                soundhandle.say(warnMsg)  # speak warning to user


# Code based on: https://answers.ros.org/question/10829/text-to-speech-in-a-python-node/
def init():
    global soundhandle, warnThreshold
    warnThreshold= 1.2 #in meters
    soundhandle = SoundClient()
    rospy.init_node('say', anonymous=True)
    rospy.Subscriber("people_points", user_points, getPed)
    rospy.sleep(1)  # give time for soundplay node to initialize
    soundhandle.say('Its your boi') #confirm that soundplay is working
    rospy.spin()


if __name__ == '__main__':
    init()
