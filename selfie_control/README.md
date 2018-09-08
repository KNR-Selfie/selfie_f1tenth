### More detailed info about node selfie_stm32_bridge:
This node is responsible for communication with STM32 through USB. STM32 sends orientation, linear acceleration and angular velocity from IMU, velocity of robot from encoders and also sends control flags.
This variables are sent to sensor_msg/Imu.h and std_msg/Float32.h.
STM32 gets from this node variables from ackermann_msgs/AckermannDriveStamped.

Uses: imu_publisher, velo_publisher, ackerman_subscriber

Commands:
sudo chmod 777 ./dev/ttyACM0
source ./devel/setup.bash
rosrun selfie_stm32_bridge selfie_stm32_bridge
