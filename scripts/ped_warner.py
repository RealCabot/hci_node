#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
from kinect2_tracker.msg import user_points
import dynamic_reconfigure.client

class Pedestrian_Warner:
    def __init__(self):
        
        self.soundhandle = SoundClient()
        self.warnThreshold = 1.2 # in meters
        self.prev_peds = 0
        
        rospy.init_node('pedestrian_warner', anonymous=True)
        
        rospy.Subscriber("/people_points", user_points, self.getPed)
        self.config_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=5)
        self.max_speed = self.config_client.get_configuration()['max_vel_x']

        rospy.sleep(1)  # give time for soundplay node to initialize
        self.soundhandle.say('Its your boi') #confirm that soundplay is working
        rospy.spin()

    def getPed(self, msg):
        peds = msg.people_points
        curr_peds = len([p for p in peds if p.point.z < self.warnThreshold])
        if self.prev_peds != curr_peds:
            if curr_peds:
                self.soundhandle.say('{num} pedestrains ahead'.format(num=curr_peds))
            else:
                self.soundhandle.say('All clear!')
            self.config_client.update_configuration({'max_vel_x': self.scared_speed(curr_peds)})
            self.prev_peds = curr_peds
    
    def scared_speed(self, num_peds):
        return self.max_speed / (num_peds+1)


if __name__ == '__main__':
    warner = Pedestrian_Warner()
