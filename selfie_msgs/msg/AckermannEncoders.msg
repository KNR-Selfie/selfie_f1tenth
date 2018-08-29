## Encoders update for a car-like vehicle using Ackermann steering.

Header header

# Assumes Ackermann front-wheel steering. The left and right front
# wheels are generally at different angles. To simplify, the measured
# angle corresponds to the yaw of a virtual wheel located at the
# center of the front axle, like on a tricycle. Positive yaw is to
# the left.
#
float32 steering_angle # virtual steering angle (radians)

# Angular speed measured at the center of the rear axle.
#
float32 angular_speed # forward speed (rad/s)
