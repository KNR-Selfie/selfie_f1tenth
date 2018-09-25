#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

UPDATE_RATE = 50

def steering_angle_callback(msg):
    cmd = AckermannDriveStamped()

    cmd.drive.steering_angle = msg.data
    cmd.drive.speed = speed

    drive_pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('selfie_angle_to_ackermann')

    global speed
    speed = rospy.get_param('~speed', 0.5)

    global drive_pub
    drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)

    angle_sub = rospy.Subscriber('steering_angle', Float64, steering_angle_callback, queue_size=1)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown(): rate.sleep()
