#!/bin/bash

# Takeoff
echo "Publishing local setpoint for the drone ..."
rostopic pub --once /setpoint/local_pos geometry_msgs/Point "x: 0.0
y: 0.0
z: 4.0"

# Bring camera down
echo "Bringing camera down ..."
rostopic pub --once /mavros/mount_control/command mavros_msgs/MountControl "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
mode: 2
pitch: -90.0
roll: 0.0
yaw: 0.0
altitude: 0.0
latitude: 0.0
longitude: 0.0"


# Arm and set OFFBOARD mode
echo "Arming and setting OFFBOARD mode ..."
rosservice call /arm_and_offboard "{}"

# Give some time for the drone to go over the tag
echo "Sleeping 15 seconds to give the drone time to takeoff"
sleep 15

# Move the Husky
echo "Moving the Husky ..."
rosrun mavros_apriltag_tracking zigzag

# ROSbag for Husky
# rosbag play Husky_cmd_vel.bag
