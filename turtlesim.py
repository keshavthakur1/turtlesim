#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_turtle():
    # Initialize the ROS node with a unique name
    rospy.init_node('move_turtle', anonymous=True)

    # Create a publisher for the turtle's velocity
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Create a Twist message for linear and angular velocities
    move_cmd = Twist()
    move_cmd.linear.x = 1.0  # Linear velocity (m/s)]
    move_cmd.angular.z = 1.0  # Angular velocity (rad/s)

    # Move the turtle in a square
    for _ in range(4):
        turtle_vel_pub.publish(move_cmd)
        rospy.sleep(2)  # Sleep for 2 seconds

        # Stop the turtle
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        turtle_vel_pub.publish(move_cmd)
        rospy.sleep(1)  # Sleep for 1 second

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass
