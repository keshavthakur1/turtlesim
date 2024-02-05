#!/usr/bin/env python

import rospy

def simple_node():
    # Initialize the ROS node with a unique name
    rospy.init_node('simple_node', anonymous=True)

    # Print a message to the console
    rospy.loginfo("Simple ROS Node is running!")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    simple_node()
