#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
from math import atan2, pi
import numpy as np
from angles import shortest_angular_distance

class Extractor:
    def __init__(self):
        self.check_window = 500
        self.tolerence = pi/6
        rospy.init_node('corner_extractor')
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.analyze_plan)
        rospy.spin()

    def analyze_plan(self, plan):
        print('Got new plan of length {}'.format(len(plan.poses)))
        angles = [self.yaw_from_sposes(spose_a, spose_b) for spose_a, spose_b in zip(plan.poses, plan.poses[1:])]
        prev_status = 'None'
        status = 'None'
        left_intervals = []
        right_intervals = []
        right_start = -1
        left_start = -1
        for (i,(angle_a, angle_b)) in enumerate(zip(angles, angles[self.check_window:])):
            difference = shortest_angular_distance(angle_a, angle_b)
            if difference > self.tolerence:
                status = 'left'
                if prev_status == 'right':
                    right_end = i
                    right_intervals.append((right_start, right_end))
                if prev_status != 'left':
                    print('Left starts at {}'.format(i))
                    left_start = i
            elif -difference > self.tolerence:
                status = 'right'
                if prev_status == 'left':
                    left_end = i
                    left_intervals.append((left_start, left_end))
                if prev_status != 'right':
                    print('Right starts at {}'.format(i))
                    right_start = i
            else:
                status = 'None'
                if prev_status == 'left':
                    left_end = i
                    left_intervals.append((left_start, left_end))
                if prev_status == 'right':
                    right_end = i
                    right_intervals.append((right_start, right_end))
            prev_status = status
        # If the path ends with left or right turn
        if prev_status == 'left':
            left_end = i
            left_intervals.append((left_start, left_end))
        if prev_status == 'right':
            right_end = i
            right_intervals.append((right_start, right_end))

        left_corner_poses = [plan.poses[(s+e)//2+self.check_window//2].pose for s,e in left_intervals]
        right_corner_poses = [plan.poses[(s+e)//2+self.check_window//2].pose for s,e in right_intervals]
        pub = rospy.Publisher('corners', PoseArray, queue_size=10)
        
        msg = PoseArray()
        msg.header.frame_id='/map'
        msg.poses=left_corner_poses+right_corner_poses

        pub.publish(msg)
        print('Published corners of length {}'.format(len(msg.poses)))


    def yaw_from_sposes(self, spose_a, spose_b):
        pos_a = spose_a.pose.position
        pos_b = spose_b.pose.position
        return atan2(pos_b.y - pos_a.y, pos_b.x - pos_a.x)

if __name__ == '__main__':
    corner_extractor = Extractor()