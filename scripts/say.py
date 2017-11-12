#!/usr/bin/env python
import rospy
from sound_play.libsoundplay import SoundClient

#self.soundhandle = SoundClient()
soundhandle = SoundClient()
rospy.init_node('say')
rospy.sleep(1)
#self.soundhandle.say('Take me to your leader.')
soundhandle.say('YEAAAA BOYYYY')
