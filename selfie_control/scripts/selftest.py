#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class tester:
  def __init__(self):
    self.set_speed = 0.5
    self.set_acceleration = 5
    self.wait_time = 0.5  #seconds
    self.imu_msgs = []
    self.dist_msgs = []
    self.speed_msgs = []
    self.msg_start = AckermannDriveStamped()
    self.msg_stop = AckermannDriveStamped()
    msg = AckermannDriveStamped()
    msg.drive.speed = self.set_speed
    msg.drive.acceleration = self.set_acceleration
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 10
    self.msg_start = msg
    self.msg_stop = msg
    self.msg_turn = msg
    self.msg_turn.drive.steering_angle = 1  #skręt w prawo
    self.msg_stop.drive.speed = 0

  def imu_callback(self, data):
    self.imu_msgs.append(data)

  def distance_callback(self, data):
    self.dist_msgs.append(data)
  
  def speed_callback(self, data):
    self.speed_msgs.append(data)



  def view_results(self):
    avg_speed = 0
    n_msgs = len(self.speed_msgs)
    sum = 0
    for s in self.speed_msgs:
      sum = sum + s.data
    try:
      avg_speed = sum / n_msgs
    except ZeroDivisionError:
      pass

    size = len(self.imu_msgs)
    linear_acc = np.zeros([size, 3])
    angular_vel = np.zeros([size, 3])
    
    i=0
    for msg in self.imu_msgs:
      linear = msg.linear_acceleration
      x_lin = linear.x
      y_lin = linear.y
      z_lin = linear.z
      lin_vec = [x_lin, y_lin, z_lin]
      linear_acc[i, :] = lin_vec

      angular = msg.angular_velocity
      x_ang = angular.x
      y_ang = angular.y
      z_ang = angular.z
      ang_vec = [x_ang, y_ang, z_ang]
      angular_vel[i,:] = ang_vec
      i = i+1

  #   f = open('/home/marcel/catkin_ws/src/selfie_f1tenth/selfie_control/scripts/selftest_try/linear', 'wb+')
  #   np.save(f, linear_acc)
  #   f.close()
  
  #   f = open('/home/marcel/catkin_ws/src/selfie_f1tenth/selfie_control/scripts/selftest_try/angular', 'wb+')
  #   np.save(f, angular_vel)
  #   f.close()

    avg_x_acc = np.mean(linear_acc[:,0])
    avg_y_acc = np.mean(linear_acc[:,1])
    avg_z_acc = np.mean(linear_acc[:,2])

    avg_x_rot = np.mean(angular_vel[:,0])
    avg_y_rot = np.mean(angular_vel[:,1])
    avg_z_rot = np.mean(angular_vel[:,2])

    print('linear acceleration: ' + repr(np.mean(linear_acc, axis=0)))
    print('angular speed: ' + repr(np.mean(angular_vel, axis=0)))

    arr = self.dist_msgs
    dist = arr[len(arr)-1].data - arr[0].data

    # x acceleration
    if (self.set_acceleration - avg_x_acc > 1 ):
      rospy.logerr('x acceleration = ' + repr(avg_x_acc) + 
        ' but ' + repr(self.set_acceleration) + ' expected')
    else:
      rospy.loginfo('x acceleration as expected ( ' +repr(avg_x_acc) +
         ' ) vs ' + repr(self.set_acceleration))
    # gravitation
    if((avg_z_acc - 9.81) > 0.5):
      rospy.logerr('gravitation = ' + repr(avg_z_acc) + ' but ' + repr(9.81) + ' expected')
    else:
      rospy.loginfo('gravitation ok =' + repr(avg_z_acc))

    #general stats
    rospy.loginfo('test time: ' + repr(self.wait_time) + ' s')
    rospy.loginfo('avg speed = '+ repr(avg_speed) + ' kmph')
    rospy.loginfo('distance travelled = ' + repr(dist) + ' m')
    rospy.loginfo('/speed topic freq =' + repr(n_msgs/self.wait_time) + ' Hz')
    

  def self_test(self):
    rospy.init_node('self_test', anonymous=True)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
    rospy.Subscriber('/imu', Imu, self.imu_callback)
    rospy.Subscriber('/distance', Float32, self.distance_callback)
    rospy.Subscriber('/speed', Float32, self.speed_callback)

    

    drive_pub.publish(self.msg_start)

    rospy.sleep(self.wait_time)

    drive_pub.publish(self.msg_stop)

    rospy.signal_shutdown('end of test')



if __name__ == '__main__':
  test = tester()
  try:
      test.self_test()
  except rospy.ROSInterruptException:
      pass
  test.view_results()