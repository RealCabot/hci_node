#!/usr/bin/env python
import rospy
#from scipy.spatial import distance
from sound_play.libsoundplay import SoundClient
from kinect2_tracker.msg import user_points
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from corner_extract_standalone import Extractor
import dynamic_reconfigure.client

class Pedestrian_Warner:
    def __init__(self):
        
        self.soundhandle = SoundClient()
        self.corner_extractor = Extractor()
        self.warnThreshold = 2  # in meters. Originally 1.2
        self.prev_peds = 0

        self.cornerPts = None
        rospy.init_node('pedestrian_warner', anonymous=True)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.get_corners)
        #rospy.Subscriber("corners", PoseArray, self.get_corners)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.warn_corner)
        rospy.Subscriber("/people_points", user_points, self.get_ped)
        self.config_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=5)
        self.max_speed = self.config_client.get_configuration()['max_vel_x']

        rospy.sleep(1)  # give time for soundplay node to initialize
        self.soundhandle.say('Its your boi') # confirm that soundplay is working
        rospy.spin()

    def get_ped(self, msg):
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

    # called when the corner extractor publishes points
    def get_corners(self, msg):
        self.cornerPts = self.corner_extractor.analyze_plan(msg)

    # warn the user if we are approaching a corner
    def warn_corner(self, msg):
        if self.cornerPts is not None and len(self.cornerPts) > 0:
            # find euclidean distances between current position and list of corner points
            curr_pt = msg.pose.pose.position
            dist = []
            for c in self.cornerPts:
                corn_pt = c.pose.position
                d = np.sqrt((curr_pt.x-corn_pt.x)**2 + (curr_pt.y-corn_pt.y)**2)  # find euclidean distance
                dist.append(d)

            min_dist = min(dist)
            min_ind = np.argmin(dist)

            if min_dist < self.warnThreshold:
                # print '!_TURN AHEAD_!'
                turn_dir = self.cornerPts[min_ind].turnType
                self.cornerPts.pop(min_ind)  # delete closest point so we stop warning the user
                # format to only print 1 decimal place
                self.soundhandle.say('Turn ' + turn_dir + ' in {dist} meters'.format(dist='%.1f' % min_dist))


if __name__ == '__main__':
    warner = Pedestrian_Warner()
