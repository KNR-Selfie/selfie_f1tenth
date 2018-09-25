#!/usr/bin/env python

import rospy
import tf2_ros
import math
import sys
import pickle

from scipy import interpolate
from std_msgs.msg import Float64

UPDATE_RATE = 50

def generate_interpolation(offsets, resolution, origin):
    x = []
    y = []
    z = []
    for ix in range(offsets.shape[1]):
        for iy in range(offsets.shape[0]):
            x.append(ix*resolution + map_data['origin'][0])
            y.append((offsets.shape[0] - iy - 1)*resolution + map_data['origin'][1])
            z.append(offsets[iy, ix])

    return interpolate.interp2d(x, y, z)

if __name__ == '__main__':
    rospy.init_node('selfie_offset_publisher')

    # Read output of map preprocessing
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        map_data = pickle.load(f)

    eval_offset = generate_interpolation(map_data['offsets'],
                                         map_data['resolution'],
                                         map_data['origin'])

    # Announce topic publisher
    offset_pub = rospy.Publisher('position_offset', Float64, queue_size=UPDATE_RATE)

    # Configure transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        try:
            t = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        offset_pub.publish(Float64(eval_offset(t.transform.translation.x,
                                               t.transform.translation.y)))

        rate.sleep()
