#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
from kinect2_tracker.msg import user_points

class Pedestrian_Warner:
    def __init__(self):
        self.soundhandle = SoundClient()
        self.warnThreshold = 1.2 # in meters
        self.prev_peds = 0
        rospy.init_node('pedestrian_warner', anonymous=True)
        rospy.Subscriber("/people_points", user_points, self.getPed)
        rospy.sleep(1)  # give time for soundplay node to initialize
        self.soundhandle.say('Its your boi') #confirm that soundplay is working
        rospy.spin()

    def getPed(self, msg):
        peds = msg.people_points
        if len(peds) > 0:
            curr_peds = len([p for p in peds if p.point.z < self.warnThreshold])
            if self.prev_peds != curr_peds:
                self.soundhandle.say('{num} pedestrains ahead'.format(num=curr_peds))
            self.prev_peds = curr_peds

if __name__ == '__main__':
    warner = Pedestrian_Warner()
