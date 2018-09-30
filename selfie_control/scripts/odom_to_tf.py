#!/usr/bin/env python
import rospy

import tf
from nav_msgs.msg import Odometry

def odom_callback(msg):
    position = msg.pose.pose.position
    rotation = msg.pose.pose.orientation

    br.sendTransform((position.x, position.y, 0),
                     (rotation.x, rotation.y, rotation.z, rotation.w),
                     rospy.Time.now(),
                     'base_link',
                     'odom')

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')

    global br
    br = tf.TransformBroadcaster()

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()
