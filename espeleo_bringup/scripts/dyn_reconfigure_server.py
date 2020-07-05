#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from espeleo_bringup.cfg import EspeleoConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {}""".format(config))
    return config

if __name__ == "__main__":
    rospy.init_node("espeleo", anonymous = False)

    srv = Server(EspeleoConfig, callback)
    rospy.spin()
