#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
import numpy as np

# from std_msgs.msg import String
from hci_node.msg import waypoint
from math import *

soundhandle = None
wayPoints = None  # dictionary of waypoints and turn angles
warnThreshold = None  # distance to point (in meters) before we warn the user
turnThreshold = None  # angle of turn beyond which we warn user
currPt = 0  # detect when waypoint changes to next waypoint
isFeet = None #choose between feet or meters
warned = False
footToMeter = 3.2808    #1m = 3.2808 ft
meterToFoot = .31       #1 foot = .31 meters

# NOTE: THIS FUNCTION ASSUMES A CONTINUOUS STREAM OF DISTANCE AND WAYPOINT NUMBERS
# ABRUPT TRANSITIONS WILL GIVE INACCURATE READINGS
def getPos(msg):
    global soundhandle, warnThreshold, turnThreshold, warned, currPt
    newPt = msg.pointNum
    ptKey = str(newPt)
    distToPoint = msg.dist

    if isFeet: #convert meters to feet
        if (distToPoint < meterToFoot): #use decimal if dist is less than 1 ft
            dist_str = round(distToPoint * footToMeter, 2)
        else:
            dist_str = str(int(distToPoint * footToMeter)) + " feet"
    else: #use meters
        dist_str = str(round(distToPoint, 2)) + " meters"

    # Detect waypoint number has updated. There's probably a better way to do this
    if currPt != newPt:
        if warned == True:
            completeMsg = "Turn completed. Next turn in " + dist_str
            print completeMsg
            soundhandle.say(completeMsg)  # indicate turn is finished

        warned = False
        currPt = newPt

     # warn the user if not already warned
    if not warned and ptKey in wayPoints:

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        currPoint = wayPoints[ptKey]
        currAngle = currPoint["theta"]
        # print "currAngle: ", currAngle, "distToPoint: ", distToPoint

        if abs(currAngle) >= turnThreshold and distToPoint <= warnThreshold:
            if currAngle < 0:  # right turn is negative
                direction = "right"
            else:  # left turn is positive
                direction = "left"

            angleStr = str(round(np.rad2deg(abs(currAngle)), 3))
            warnMsg = "Turn " + direction + " " + angleStr + " degrees in " + dist_str
            print warnMsg
            soundhandle.say(warnMsg)  # speak warning to user
            warned = True


# Code based on: https://answers.ros.org/question/10829/text-to-speech-in-a-python-node/
def init():
    global soundhandle, wayPoints, turnThreshold, warnThreshold, isFeet
    soundhandle = SoundClient()
    rospy.init_node('say', anonymous=True)
    wayPoints = rospy.get_param("/waypoints")
    turnThreshold = rospy.get_param("~turn_threshold", pi / 3)
    warnThreshold = rospy.get_param("~warn_threshold", 2)
    isFeet = rospy.get_param("~isFeet", True)
    # print "wayPoints", wayPoints
    rospy.Subscriber("waypt", waypoint, getPos)
    rospy.sleep(1)  # give time for soundplay node to initialize
    soundhandle.say('Voice feedback on')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    init()
