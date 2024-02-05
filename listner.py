#!/usr/bin/env python

import rospy
from novatel_oem7_msgs.msg
import BESTPOS

def callback(data):
    rospy.loginfo("Received GPS data:\nLatitude: %f\nLongitude: %f\n", data.lat, data.lon)

def listener():
    rospy.init_node('listener_node', anonymous=True)

    # Subscribe to the BESTPOS topic
    rospy.Subscriber("/novatel/oem7/bestpos", BESTPOS, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
