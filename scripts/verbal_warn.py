#!/usr/bin/env python
import rospy
import tf
from kinect2_tracker.msg import user_IDs
from kinect2_tracker.msg import user_points
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from corner_extractor import Extractor
import dynamic_reconfigure.client
import math
import time 
import numpy

# This node provides verbal warnings (text-to-speech) for both the user and the pedestrian
# Warns pedestrians if they get too close to CaBot
# Warns the user of upcoming turns

PED_DISTANCE_TOLERENCE = 0.11
ANGLE_MATCH_INDEX_PLUS_MINUS = 15                                                                                                                        # 10 degrees


class SoundClient:
    def __init__(self):
        self.pub = rospy.Publisher('/say', String, queue_size=10)
    def say(self, sentence):
        self.pub.publish(sentence)

class Pedestrian_Warner:
    def __init__(self):
        rospy.init_node('pedestrian_warner', anonymous=True)
        self.soundhandle = SoundClient()
        self.corner_extractor = Extractor()

        self.warnThreshold = rospy.get_param("~warn_threshold", 1.2)  # in meters. Distance to warn pedestrian
        self.cornerWarn = rospy.get_param("~corner_threshold", 2)  # in meters. Distance to warn user of upcoming turn
        self.pedSensitivity = rospy.get_param("~pedestrian_sensitivity", 10)  # in meters. Distance to warn user of upcoming turn
        self.tooCloseDistance = rospy.get_param("~too_close_distance", 0.8) # The human is so close that Kinnect cannot get the correct height
        self.prev_peds = 0
        self.ped_same_time = 0 # The consecutive time the pedestrain count is the same

        self.cornerPts = None
        self.dist_unit = rospy.get_param("dist_units", 'ft')
        self.config_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=7)
        self.max_speed = self.config_client.get_configuration()['max_vel_x']
        self.last_sentence=""
        self.last_time = 0
        
        self.potential_peds = []

        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.get_corners)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.warn_corner)
        #rospy.Subscriber("/people_skeleton", user_IDs, self.get_ped)
        rospy.Subscriber("/scan", LaserScan, self.filter_peds)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/base_link', '/kinect_camera_frame', rospy.Time(0),rospy.Duration(4.0) )
        rospy.Subscriber("/people_points", user_points, self.get_peds)
        

        rospy.sleep(1)  # give time for soundplay node to initialize
        self.soundhandle.say('Pass me the bottle')  # confirm that soundplay is working
        rospy.spin()

    def stablizer(self, new_value):
        if self.prev_peds == new_value:
            self.ped_same_time+=1
        else:
            self.prev_peds = new_value
            self.ped_same_time = 0
        return self.ped_same_time == self.pedSensitivity

    def filter_peds(self, msg):

        laser_vals, ang_inc, ang_min = msg.ranges, msg.angle_increment, msg.angle_min

        num_peds = 0
        for (angle, dist) in self.potential_peds:
            matched_index = int((angle - ang_min) / ang_inc)
            dist_lasers = laser_vals[matched_index - ANGLE_MATCH_INDEX_PLUS_MINUS : matched_index + ANGLE_MATCH_INDEX_PLUS_MINUS]
            for d in dist_lasers:
	            if (d - PED_DISTANCE_TOLERENCE/2) < dist and dist < (d + PED_DISTANCE_TOLERENCE/2):
	                num_peds += 1
	                break

        if self.stablizer(num_peds):
            if num_peds:
                sentence = '{num} pedestrains ahead, slowing down'.format(num=num_peds)
            else:
                sentence = ''
            if sentence!=self.last_sentence:
                rospy.loginfo(sentence)
                self.soundhandle.say(sentence)
                self.last_sentence = sentence
                self.config_client.update_configuration({'max_vel_x': self.scared_speed(num_peds)})
        
    def get_peds(self, msg):
        center_pts = msg.people_points
        self.potential_peds = [(math.atan(p.point.x/p.point.z), math.sqrt(p.point.x**2 + p.point.z**2)) for p in center_pts if p.point.z!=0]
        # print(self.potential_peds)

    def scared_speed(self, num_peds):  
    	return self.max_speed
        # if num_peds == 0:
        #     return self.max_speed
        # else:
        #     return self.max_speed - 0.02	

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

            if min_dist < self.cornerWarn:
                # print '!_TURN AHEAD_!'
                turn_dir = self.cornerPts[min_ind].turnType #either left or right
                self.cornerPts.pop(min_ind)  # delete closest point so we stop warning the user

                # convert meters to feet, round to 1 decimal place
                if self.dist_unit == 'ft':
                    v_dist = str(round(3.2808 * min_dist, 1))  # 1m = 3.2808 ft
                    units = ' feet'
                # use meters, round to nearest integer
                else:
                    v_dist = str(int(round(min_dist)))
                    units = ' meters'

                # format to only print 1 decimal place
                self.soundhandle.say('Turn ' + turn_dir + ' in ' + v_dist + units)


if __name__ == '__main__':
    warner = Pedestrian_Warner()
