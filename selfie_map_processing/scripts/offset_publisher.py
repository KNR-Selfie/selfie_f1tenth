#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import math
import sys
import pickle
import numpy as np

from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from selfie_map_processing.cfg import MapProcessingConfig

UPDATE_RATE = 100

def config_callback(config, level):
    global L
    L = config['L']

    rospy.loginfo("Reconfigure request: L=" + str(L) + "m")

    return config

if __name__ == '__main__':
    rospy.init_node('selfie_offset_publisher')

    srv = Server(MapProcessingConfig, config_callback)

    # Read output of map preprocessing
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        map_data = pickle.load(f)

    pathpoints = map_data['pathpoints']

    # Announce topic publishers
    offset_pub = rospy.Publisher('steering_state', Float64, queue_size=UPDATE_RATE)
    path_pub = rospy.Publisher('path', Path, queue_size=UPDATE_RATE)

    # Configure transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        try:
            pose = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        x = pose.translation.x
        y = pose.translation.y

        quaternion = (
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w
        )

        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)

        point = (x, y)
        current_idx = np.argmin(np.sum(np.square(np.array(point) - pathpoints), 1))
        current_point = pathpoints[current_idx]
        next_idx = (current_idx + 2) % len(pathpoints)
        next_point = pathpoints[next_idx]
        alpha = math.atan2(next_point[1] - current_point[1],
                           next_point[0] - current_point[0])

        theta = euler[2] - alpha

        dx = x - current_point[0]
        dy = y - current_point[1]

        offset = -dx*math.sin(alpha) + dy*math.cos(alpha)

        offset_pub.publish(offset + L*np.sin(theta))

        ### DEBUG ###

        path = Path()
        path.header.frame_id = 'map'
        path.poses = []

        for i in range(5):
            idx = (current_idx + i) % len(pathpoints)
            pnt = pathpoints[idx]

            pose = PoseStamped()
            pose.header.frame_id = 'map'

            pose.pose.position.x = pnt[0]
            pose.pose.position.y = pnt[1]

            path.poses.append(pose)

        path_pub.publish(path)

        rate.sleep()
