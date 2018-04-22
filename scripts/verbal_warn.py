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
import math as m 
import time 


# This node provides verbal warnings (text-to-speech) for both the user and the pedestrian
# Warns pedestrians if they get too close to CaBot
# Warns the user of upcoming turns

PEDESTRAIN_HEIGHT_MIN = 1.2/2
PEDESTRAIN_HEIGHT_MAX = 2.0/2
#ANG_MIN = None
#ANG_INC = None
THRESH = 0.11


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
        self.pedSensitivity = rospy.get_param("~pedestrian_sensitivity", 3)  # in meters. Distance to warn user of upcoming turn
        self.tooCloseDistance = rospy.get_param("~too_close_distance", 0.8) # The human is so close that Kinnect cannot get the correct height
        self.prev_peds = 0
        self.ped_same_time = 0 # The consecutive time the pedestrain count is the same

        self.cornerPts = None
        self.dist_unit = rospy.get_param("dist_units", 'ft')
        self.config_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=7)
        self.max_speed = self.config_client.get_configuration()['max_vel_x']
        self.last_sentence=" "
        self.last_time = 0;
        
        self.angles = {}
        

        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.get_corners)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.warn_corner)
        #rospy.Subscriber("/people_skeleton", user_IDs, self.get_ped)
        rospy.Subscriber("/scan", LaserScan, self.get_laser)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/base_link', '/kinect_camera_frame', rospy.Time(0),rospy.Duration(4.0) )
        rospy.Subscriber("/people_points", user_points, self.get_center)
        

        rospy.sleep(1)  # give time for soundplay node to initialize
        self.soundhandle.say('Pass me the bottle')  # confirm that soundplay is working
        rospy.spin()

    def get_laser(self, msg):

        laser_vals = msg.ranges
        ang_inc = msg.angle_increment
        ang_min = msg.angle_min
        inc = 0
        for scan in laser_vals:
            angle = ang_min+(inc*ang_inc)
            if int(angle) in self.angles:
                self.angles[int(angle)].append(scan)
            else:
            	self.angles[int(angle)] = [scan]
                
            inc += 1
        
        
    def get_both(self, center):
        angle_cent = int(m.atan(center.x/center.y))
        dist_cent  = m.sqrt(center.x**2 + center.y**2)
        #print("dist_center: ", dist_cent)
        for dist_laser in self.angles[angle_cent]:
	    #print("dist_laser: ", dist_laser)
            if (dist_laser - THRESH/2) < dist_cent and dist_cent < (dist_laser + THRESH/2):
                return True
        return False     
       
        
    def get_center(self, msg):

        center_pts = msg.people_points
        centers = [self.listener.transformPoint('/base_link', p).point for p in center_pts] 
        #print time.time()%4.00 <= 1
        if (round(time.time())%4.00) == 0: 
		    ped_num = 0
		    for center in centers:
		        #rospy.loginfo("centers: ", center)
		        ped_there = self.get_both(center)
		        if ped_there:
		            ped_num += 1
		    
		    
		    if ped_num:
		    	sentence = self.last_sentence
		    	now_time = time.time()
		    	if (now_time - self.last_time) > 2: 
		            sentence = '{num} pedestrians ahead'.format(num=ped_num)
		            self.last_time = now_time
		    else:
		    	sentence = ' '

		    if sentence!=self.last_sentence:
		        rospy.loginfo(sentence)
		        #self.soundhandle.say(sentence_say)
		        self.soundhandle.say(sentence)
		        self.last_sentence = sentence
		        self.config_client.update_configuration({'max_vel_x': self.scared_speed(ped_num)})




    # def get_ped(self, msg):

    #     ped_num = len(msg.users)

        
        
    #     if ped_num:
    #         sentence = 'pedestrains ahead'
    #     else:
    #         sentence = 'All clear!'
    #         #sentence = 'I am deeply emotional'
    #     if sentence!=self.last_sentence:
    #         rospy.loginfo(sentence)
    #         self.soundhandle.say(sentence)
    #         self.last_sentence = sentence
    #         self.config_client.update_configuration({'max_vel_x': self.scared_speed(ped_num)})
    
    def scared_speed(self, num_peds):  
        if num_peds == 0:
            return self.max_speed
        else:
            return self.max_speed - 0.02	

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
