#!/usr/bin/env python
import rospy
from scipy.spatial import distance
from sound_play.libsoundplay import SoundClient
from kinect2_tracker.msg import user_points
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client

class Pedestrian_Warner:
    def __init__(self):
        
        self.soundhandle = SoundClient()
        self.warnThreshold = 3  # in meters. Originally 1.2
        self.prev_peds = 0

        self.cornerPts = None
        rospy.init_node('pedestrian_warner', anonymous=True)
        rospy.Subscriber("corners", PoseArray, self.get_corners)
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
        corners = msg.poses
        # make a Nx3 matrix, where N is number of corners, [x y z]^T
        self.cornerPts = np.zeros((0, 3))
        for p in corners:
            coord = p.position
            pt = np.array([[coord.x, coord.y, coord.z]])
            self.cornerPts = np.vstack((self.cornerPts, pt))
        print('Received corners:\n', self.cornerPts)

    # warn the user if we are approaching a corner
    def warn_corner(self, msg):
        if self.cornerPts is not None and len(self.cornerPts) > 0:
            # find min euclidean distance between current position and list of corner points
            coord = msg.pose.pose.position
            curr_pt = np.array([[coord.x, coord.y, coord.z]]) #CaBot's current position
            dist = distance.cdist(self.cornerPts, curr_pt, 'euclidean') # find distance between curr_pt and each row
            #print(self.cornerPts, curr_pt)
            min_dist = np.amin(dist)
            min_ind = np.argmin(dist)
            print 'pt:', self.cornerPts[min_ind, :], 'ind:', min_ind, 'dist:', min_dist

            if min_dist < self.warnThreshold:
                print '!_TURN AHEAD_!'
                # delete closest point so we stop warning the user
                self.cornerPts = np.delete(self.cornerPts, min_ind, 0)
                self.soundhandle.say('Turn {dist} meters ahead'.format(dist=min_dist))


if __name__ == '__main__':
    warner = Pedestrian_Warner()
