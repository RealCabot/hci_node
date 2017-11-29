#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient
import numpy as np

#from std_msgs.msg import String
from hci_node.msg import waypoint
from math import *

soundhandle = None
wayPoints = None #dictionary of waypoints and turn angles
WARN_THRESHOLD = 5 #distance to point (in meters) before we warn the user
TURN_THRESHOLD = pi/3 #angle of turn beyond which we warn user
currPt = 0 #detect when waypoint changes to next waypoint
warned = False

def getPos(msg):
    global soundhandle, WARN_THRESHOLD, TURN_THRESHOLD, warned, currPt
    newPt = msg.pointNum
    ptKey = str(newPt)
    distToPoint = msg.dist

     #Detect waypoint has updated. There's probably a better way to do this
    if currPt != newPt:
        if warned == True:
            completeMsg = "Turn completed. Next turn in " + str(distToPoint) + " meters."
            print completeMsg
            soundhandle.say(completeMsg)  # indicate turn is finished

        warned = False
        currPt = newPt

     #warn the user if not already warned
    if not warned and ptKey in wayPoints:

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        currPoint = wayPoints[ptKey]
        currAngle = currPoint["theta"]
        #print "currAngle: ", currAngle, "distToPoint: ", distToPoint

        if abs(currAngle) >= TURN_THRESHOLD and distToPoint <= WARN_THRESHOLD:
            if currAngle < 0:   #right turn is negative
                direction = "right"
            else:               #left turn is positive
                direction = "left"
	
   	    angleStr = str(round(np.rad2deg(abs(currAngle)), 3))
            warnMsg = "Turn " + direction + " " + angleStr + " degrees in " + str(round(distToPoint, 2)) + " meters."
            print warnMsg
            soundhandle.say(warnMsg) #speak warning to user
            warned = True


#Code based on: https://answers.ros.org/question/10829/text-to-speech-in-a-python-node/
def init():
    global soundhandle, wayPoints
    soundhandle = SoundClient()
    rospy.init_node('say', anonymous=True)
    wayPoints = rospy.get_param("/waypoints")

    #print "wayPoints", wayPoints
    rospy.Subscriber("waypt", waypoint, getPos)
    rospy.sleep(1) #give time for soundplay node to initialize
    soundhandle.say('Voice feedback on')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    init()


