#!/usr/bin/env python
# license removed for brevity
#import rospy
#from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
from math import atan2, pi
from angles import shortest_angular_distance
from d1_toolbox import get_intervals, non_maximum_suppression, dilate, filter_noise

class Corner:
    def __init__(self, pose, t_type):
        self.pose = pose
        self.turnType = t_type

class Extractor:
    def __init__(self):
        self.check_window = 500
        self.tolerence = pi/4  # originally pi/6. changed to remove extraneous corner after turn #2.
        self.dilate_length = 20
        self.noise_length = 10

    def analyze_plan(self, plan):
        
        def yaw_from_poses(pose_a, pose_b):
            pos_a = pose_a.position
            pos_b = pose_b.position
            return atan2(pos_b.y - pos_a.y, pos_b.x - pos_a.x)

        def isLeftTurn(angle_tuple):
            difference = shortest_angular_distance(angle_tuple[0], angle_tuple[1])
            return difference > self.tolerence
        
        def isRightTurn(angle_tuple):
            difference = shortest_angular_distance(angle_tuple[0], angle_tuple[1])
            return -difference > self.tolerence

        plan = [pp.pose for pp in plan.poses]
        print('Got new plan of length {}'.format(len(plan)))
        angles = [yaw_from_poses(pose_a, pose_b) for pose_a, pose_b in zip(plan, plan[1:])]

        left_intervals = get_intervals(zip(angles, angles[self.check_window:]), isLeftTurn, shift=self.check_window//2)
        right_intervals = get_intervals(zip(angles, angles[self.check_window:]), isRightTurn, shift=self.check_window//2)

        left_intervals = dilate(filter_noise(left_intervals, self.noise_length), self.dilate_length)
        right_intervals = dilate(filter_noise(right_intervals, self.noise_length), self.dilate_length)

        left_corner_poses = non_maximum_suppression(plan, left_intervals)
        right_corner_poses = non_maximum_suppression(plan, right_intervals)
        
        msg = PoseArray()
        msg.header.frame_id = '/map'
        msg.poses=left_corner_poses+right_corner_poses

        print(left_intervals, right_intervals)
        #self.pub.publish(msg)
        print('Published corners of length {}'.format(len(msg.poses)))
        # ========added by Andrew========
        # create array of corners and denote which corners are left and right turns
        corners = []
        for l_p in left_corner_poses:
            corners.append(Corner(l_p, 'left'))
        for r_p in right_corner_poses:
            corners.append(Corner(r_p, 'right'))
        return corners
        # ===============================


if __name__ == '__main__':
    corner_extractor = Extractor()