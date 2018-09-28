#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import math
import sys
import pickle

from scipy import interpolate
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from selfie_map_processing.cfg import MapProcessingConfig

UPDATE_RATE = 50

def generate_interpolation(lookup_table, resolution, origin):
    x = []
    y = []
    z = []
    for ix in range(lookup_table.shape[1]):
        for iy in range(lookup_table.shape[0]):
            x.append(ix*resolution + map_data['origin'][0])
            y.append((lookup_table.shape[0] - iy - 1)*resolution + map_data['origin'][1])
            z.append(lookup_table[iy, ix])

    return interpolate.interp2d(x, y, z)

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

    eval_offset = generate_interpolation(map_data['offsets'],
                                         map_data['resolution'],
                                         map_data['origin'])

    eval_direction = generate_interpolation(map_data['directions'],
                                            map_data['resolution'],
                                            map_data['origin'])

    # Announce topic publisher
    offset_pub = rospy.Publisher('steering_state', Float64, queue_size=UPDATE_RATE)

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

        offset = eval_offset(x, y)
        path_direction = eval_direction(x, y)

        theta = path_direction - euler[2]

        offset_pub.publish(Float64(offset + L*math.sin(theta)))

        rate.sleep()
