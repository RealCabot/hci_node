#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
from kinect2_tracker.msg import user_points

soundhandle = None
warnThreshold = None  # distance to point (in meters) before we warn the user

class Pedestrian_Warner:
    def __init__(self):
        self.soundhandle = SoundClient()
        self.warnThreshold = 1.2 # in meters
        rospy.init_node('pedestrian_warner', anonymous=True)
        rospy.Subscriber("/people_points", user_points, self.getPed)
        rospy.sleep(1)  # give time for soundplay node to initialize
        soundhandle.say('Its your boi') #confirm that soundplay is working
        rospy.spin()

    def getPed(self, msg):

        peds = msg.people_points
        if len(peds) > 0:
            warned_number = len([p for p in peds if p.point.z < warnThreshold])
            self.soundhandle.say('{} pedestrains ahead'.format(warned_number))

if __name__ == '__main__':
    warner = Pedestrian_Warner()
