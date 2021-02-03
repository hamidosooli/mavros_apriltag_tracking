#!/usr/bin/env python
"""
BSD 3-Clause License

Copyright (c) 2020, Mohamed Abdelkader Zahana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 """
import rospy
import tf

from math import pi, sqrt, sin, cos, atan
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_apriltag_tracking.srv import PIDGains, PIDGainsResponse
from mavros_msgs.msg import MountControl

from apriltag_ros.msg import AprilTagDetectionArray
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class FCUModes:
    def __init__(self):
	    pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e
##########################################################################################
"""
This is a PI controller which takes target positions (ex_, ey_, ez_) in body directions (i.e. relative to body).
It ouputs velocity commands (body_vx_cmd_, body_vx_cmd_, body_vz_cmd_) in body directions.
"""
class PositionController:
    def __init__(self):
        # Error in body x direction (+x is drone's right)
        self.ex_ = 0.0
        # Error in body y direction (+y is  drone's front)
        self.ey_ = 0.0
        # Error in body z direction (+z is  up)
        self.ez_ = 0.0
        # Error integral in x
        self.ex_int_ = 0.0
        # Error integral in y
        self.ey_int_ = 0.0
        # Error integral in z
        self.ez_int_ = 0.0

        # Proportional gain for horizontal controller
        self.kP_xy_ = rospy.get_param('~horizontal_controller/kP', 1.5)
        # Integral gain for horizontal controller
        self.kI_xy_ = rospy.get_param('~horizontal_controller/kI', 0.01)
        # Integral gain for vertical controller
        self.kP_z_ = rospy.get_param('~vertical_controller/kP', 2.0)
        # Integral gain for vertical controller
        self.kI_z_ = rospy.get_param('~vertical_controller/kI', 0.01)

        # Controller outputs. Velocity commands in body sudo-frame
        self.body_vx_cmd_ = 0.0
        self.body_vy_cmd_ = 0.0
        self.body_vz_cmd_ = 0.0

        # Controller outputs in local frame
        self.local_vx_cmd_ = 0.0
        self.local_vy_cmd_ = 0.0
        self.local_vz_cmd_ = 0.0

        # Maximum horizontal velocity (m/s)
        self.vXYMAX_ = rospy.get_param('~horizontal_controller/vMAX', 1.0)
        # Maximum upward velocity (m/s)
        self.vUpMAX_ = rospy.get_param('~vertical_controller/vUpMAX', 1.0)
        # Maximum downward velocity (m/s)
        self.vDownMAX_ = rospy.get_param('~vertical_controller/vDownMAX', 0.5)

        # Flag for FCU state. True if vehicle is armed and ready
        # Prevents building controller integral part when vehicle is idle on ground
        self.engaged_ = False

        # Subscribe to drone FCU state
        rospy.Subscriber('mavros/state', State, self.cbFCUstate)

        # Service for modifying horizontal PI controller gains
        rospy.Service('horizontal_controller/pid_gains', PIDGains, self.setHorizontalPIDCallback)
        # Service for modifying vertical PI controller gains
        rospy.Service('vertical_controller/pid_gains', PIDGains, self.setVerticalPIDCallback)

    def setHorizontalPIDCallback(self, req):
        if req.p < 0. or req.i < 0.0:
            rospy.logerr("Can not set negative PID gains.")
            return []

        self.kP_xy_ = req.p
        self.kI_xy_ = req.i

        rospy.loginfo("Horizontal controller gains are set to P=%s I=%s", self.kP_xy_, self.kI_xy_)

        return []

    def setVerticalPIDCallback(self, req):
        if req.p < 0. or req.i < 0.0:
            rospy.logerr("Can not set negative PID gains.")
            return []

        self.kP_z_ = req.p
        self.kI_z_ = req.i

        rospy.loginfo("Vertical controller gains are set to P=%s I=%s", self.kP_z_, self.kI_z_)

        return []


    def cbFCUstate(self, msg):
        if msg.armed and msg.mode == 'OFFBOARD' :
            self.engaged_ = True
        else:
            self.engaged_ = False

    def resetIntegrators(self):
        self.ex_int_ = 0.
        self.ey_int_ = 0.
        self.ez_int_ = 0.

    def computeVelSetpoint(self):
        """
        Computes XYZ velocity setpoint in body sudo-frame using a PI controller
        """
        # Compute commands
        self.body_vx_cmd_ = self.kP_xy_*self.ex_ + self.kI_xy_*self.ex_int_
        self.body_vy_cmd_ = self.kP_xy_*self.ey_ + self.kI_xy_*self.ey_int_
        self.body_vz_cmd_ = self.kP_z_*self.ez_ + self.kI_z_*self.ez_int_

        # Horizontal velocity constraints
        vel_magnitude = sqrt(self.body_vx_cmd_**2 + self.body_vy_cmd_**2)
        if vel_magnitude > self.vXYMAX_ : # anti-windup scaling
            scale = self.vXYMAX_/vel_magnitude
            self.body_vx_cmd_ = self.body_vx_cmd_*scale
            self.body_vy_cmd_ = self.body_vy_cmd_*scale
        else:
            if self.engaged_: # if armed & offboard
                self.ex_int_ = self.ex_int_ + self.ex_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_xy_ for now!
                self.ey_int_ = self.ey_int_ + self.ey_

        # Vertical velocity constraints
        if self.body_vz_cmd_ > self.vUpMAX_ : # anti-windup scaling
            self.body_vz_cmd_ = self.vUpMAX_
        elif self.body_vz_cmd_ < -self.vDownMAX_:
            self.body_vz_cmd_ = -self.vDownMAX_
        else:
            if self.engaged_: # if armed & offboard
                self.ez_int_ = self.ez_int_ + self.ez_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_z_ for now!

        return self.body_vx_cmd_, self.body_vy_cmd_, self.body_vz_cmd_

##########################################################################################

class Commander:
    def __init__(self):
        # Instantiate a setpoint topic structure
        self.setpoint_ = PositionTarget()

        # use velocity and yaw setpoints
        self.setBodyVelMask()

        # Velocity setpoint by user
        self.vel_setpoint_ = Vector3()

        # Position setpoint by user
        self.pos_setpoint_ = Point()

        # Current local velocity
        self.local_vel_ = TwistStamped()

        # Current body velocity
        self.body_vel_ = TwistStamped()

        # Yaw setpoint by user (degrees); will be converted to radians before it's published
        self.yaw_setpoint_ = 0.0

        # Camera setpoint
        self.cam_setpoint_ = MountControl()

        # Camera setpoint by user (degrees)
        self.cam_pitch_setpoint_ = -90.0
        self.cam_roll_setpoint_ = 0.0
        self.cam_yaw_setpoint_ = 0.0

        # Husky position
        self.husky_x_ = 0.0
        self.husky_y_ = 0.0
        self.husky_z_ = 0.0

        # Current drone position (local frame)
        self.drone_pos_ = Point()

        # FCU modes
        self.fcu_mode_ = FCUModes()

        # setpoint publisher (velocity to Pixhawk)
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # camera setpoint publisher
        self.cam_setpoint_pub_ = rospy.Publisher('/mavros/mount_control/orientation', MountControl, queue_size = 10)

        # Subscriber for user setpoints (body velocity)
        #rospy.Subscriber('setpoint/body_vel', Vector3, self.velSpCallback)

        # Subscriber for user setpoints (local position)
        #rospy.Subscriber('setpoint/local_pos', Point, self.posSpCallback)

        # Subscriber for user setpoints (yaw in degrees)
        rospy.Subscriber('setpoint/yaw_deg', Float32, self.yawSpCallback)

        # Subscriber for camera setpoints (orientation in degrees)
        rospy.Subscriber('/mavros/mount_control/command', MountControl, self.camSpCallback)

        # Subscriber to current drone's position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)

        # Subscriber to body velocity
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.bodyVelCallback)

        # Subscriber to local velocity
        rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.localVelCallback)

        # Subscriber to Husky position
        rospy.Subscriber('husky_velocity_controller/odom', Odometry, self.huskyPosSpCallback)

        # Service for arming and setting OFFBOARD flight mode
        rospy.Service('arm_and_offboard', Empty, self.armAndOffboard)

        # Service for autoland
        rospy.Service('auto_land', Empty, self.autoLand)

        # Service for holding at current position
        # rospy.Service('hold', Empty, self.hold)

    # def hold(self, req):
    #     self.pos_setpoint_.x = self.drone_pos_.x
    #     self.pos_setpoint_.y = self.drone_pos_.y
    #     self.pos_setpoint_.z = self.drone_pos_.z

    #     self.setLocalPositionMask()
    #     return EmptyResponse()

    def bodyVelCallback(self, msg):
        self.body_vel_ = msg

    def localVelCallback(self, msg):
        self.local_vel_ = msg

    def autoLand(self, req):
        self.fcu_mode_.setAutoLandMode()

        return EmptyResponse()

    def armAndOffboard(self, req):
        self.fcu_mode_.setArm()
        self.fcu_mode_.setOffboardMode()

        return EmptyResponse()

    def dronePosCallback(self, msg):
        self.drone_pos_.x = msg.pose.position.x
        self.drone_pos_.y = msg.pose.position.y
        self.drone_pos_.z = msg.pose.position.z

    def velSpCallback(self, msg):
        """
        Velocity setpoint callback
        """
        self.vel_setpoint_.x = msg.x
        self.vel_setpoint_.y = msg.y
        self.vel_setpoint_.z = msg.z

        self.setBodyVelMask()

    def posSpCallback(self, msg):
        """
        Position setpoint callback
        """
        self.pos_setpoint_.x = msg.x
        self.pos_setpoint_.y = msg.y
        self.pos_setpoint_.z = msg.z

        self.setLocalPositionMask()

    def yawSpCallback(self, msg):
        """
        Yaw setpoint callback
        """
        self.yaw_setpoint_ = msg.data

    def camSpCallback(self, msg):
        """
        Camera orientation setpoint callback
        """
        self.cam_setpoint_.pitch = msg.pitch
        self.cam_setpoint_.roll = msg.roll
        self.cam_setpoint_.yaw = msg.yaw

    def setLocalPositionMask(self):
        """
        Sets type_mask for position setpoint in local frame +  yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def setBodyVelMask(self):
        """
        Sets type_mask for velocity setpoint in body frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def setLocalVelMask(self):
        """
        Sets type_mask for velocity setpoint in local frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def publishSetpoint(self):
        self.setpoint_.header.stamp = rospy.Time.now()

        # Only one type of the following setpoints will be consumed based on the type_mask
        self.setpoint_.position.x = self.pos_setpoint_.x
        self.setpoint_.position.y = self.pos_setpoint_.y
        self.setpoint_.position.z = self.pos_setpoint_.z

        self.setpoint_.velocity.x = self.vel_setpoint_.x
        self.setpoint_.velocity.y = self.vel_setpoint_.y
        self.setpoint_.velocity.z = self.vel_setpoint_.z

        self.setpoint_.yaw = self.yaw_setpoint_ * pi / 180. # convert to radians

        self.setpoint_pub_.publish(self.setpoint_)


    def publishCamSetpoint(self):
        self.cam_setpoint_.header.stamp = rospy.Time.now()

        self.cam_setpoint_.pitch = self.cam_pitch_setpoint_
        self.cam_setpoint_.roll = self.cam_roll_setpoint_
        self.cam_setpoint_.yaw = self.cam_yaw_setpoint_

        self.cam_setpoint_pub_.publish(self.cam_setpoint_)

    def huskyPosSpCallback(self, msg):
        self.husky_x_ = msg.pose.pose.position.x
        self.husky_y_ = msg.pose.pose.position.y
        self.husky_z_ = msg.pose.pose.position.z
#####################################################################################
"""
"""
class TagPixel:
    def __init__(self):
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.TagPoseCallback)
        rospy.Subscriber('cgo3_camera/camera_info', CameraInfo, self.TagePixelCallback)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.p_image = ()
        self.valid = False

        self.x_c = 0.0
        self.y_c = 0.0

        self.height = 0.0
        self.width = 0.0

    def TagPoseCallback(self, msg):
        if len(msg.detections) > 0:
            self.valid = True
        if self.valid:
            self.pos_x = -msg.detections[0].pose.pose.pose.position.y
            self.pos_y = -msg.detections[0].pose.pose.pose.position.z
            self.pos_z = msg.detections[0].pose.pose.pose.position.x

    def TagePixelCallback(self, msg):
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(msg)
        self.p_image = np.nan_to_num(self.pcm.project3dToPixel((self.pos_x, self.pos_y, self.pos_z)))

        self.x_c = msg.K[2]
        self.y_c = msg.K[5]

        self.height = msg.height
        self.width = msg.width
        # rospy.loginfo("Pixel position is u = %s, v = %s", self.p_image[0], self.p_image[1])
#####################################################################################
"""
Provides methods to track a point in both body and local frames.
It uses PositionController class to compute control signals and Commander class to send them to the drone FCU

Set self.local_tracking_ = True to track points in local frame, and False to track in body relative frame
"""
class Tracker:
    def __init__(self):
        # setpoints in local frame
        self.local_xSp_  = 0.0
        self.local_ySp_  = 0.0
        self.local_zSp_  = 0.0

        # Relative setpoints (i.e. with respect to body horizontal-frame)
        self.relative_xSp_  = 0.0
        self.relative_ySp_  = 0.0
        self.relative_zSp_  = 0.0

        # Flag to select between local vs. relative tracking
        # set False for relative target tracking
        self.local_tracking_ = None

        # Commander object to send velocity setpoints to FCU
        self.commander_ = Commander()

        # Controller object to calculate velocity commands
        self.controller_ = PositionController()

        # Tag Pixel
        self.tagpxl_ = TagPixel()

        # Subscriber for user setpoints (local position)
        rospy.Subscriber('setpoint/local_pos', Point, self.localPosSpCallback)

        # Subscriber for user setpoints (relative position)
        rospy.Subscriber('setpoint/relative_pos', Point, self.relativePosSpCallback)

        # Publisher for velocity errors in body frame
        self.bodyVel_err_pub_ = rospy.Publisher('analysis/body_vel_err', PointStamped, queue_size=10)

        # Publisher for velocity errors in local frame
        self.localVel_err_pub_ = rospy.Publisher('analysis/local_vel_err', PointStamped, queue_size=10)

        # Publisher for position errors in local frame
        self.localPos_err_pub_ = rospy.Publisher('analysis/local_pos_err', PointStamped, queue_size=10)

        # Publisher for position error between drone and target
        self.relativePos_err_pub_ = rospy.Publisher('analysis/relative_pos_err', PointStamped, queue_size=10)

    def computeControlOutput(self):
        if self.local_tracking_:
            self.controller_.ex_ = self.local_xSp_ - self.commander_.drone_pos_.x
            self.controller_.ey_ = self.local_ySp_ - self.commander_.drone_pos_.y
            self.controller_.ez_ = self.local_zSp_ - self.commander_.drone_pos_.z
            self.commander_.setLocalVelMask()
        else: # relative tracking
            self.controller_.ex_ = self.relative_xSp_
            self.controller_.ey_ = self.relative_ySp_
            self.controller_.ez_ = self.relative_zSp_
            self.commander_.setBodyVelMask()

        self.commander_.vel_setpoint_.x, self.commander_.vel_setpoint_.y, self.commander_.vel_setpoint_.z = self.controller_.computeVelSetpoint()

    def localPosSpCallback(self, msg):
        self.local_xSp_ = msg.x
        self.local_ySp_ = msg.y
        self.local_zSp_ = msg.z

        # In case we are switching from relative to local tracking
        # to avoid jumps caused by accumulation in the integrators
        if not self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = True

    def relativePosSpCallback(self, msg):
        self.relative_xSp_ = msg.x
        self.relative_ySp_ = msg.y
        self.relative_zSp_ = msg.z

        # In case we are switching from local to relative tracking
        # to avoid jumps caused by accumulation in the integrators
        if self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = False

    def publishErrorSignals(self):
        """
        Publishes all error signals for debugging and tuning
        """

        # Local velocity errors
        localVelErr_msg = PointStamped()
        localVelErr_msg.header.stamp = rospy.Time.now()
        localVelErr_msg.point.x = self.commander_.setpoint_.velocity.x - self.commander_.local_vel_.twist.linear.x
        localVelErr_msg.point.y = self.commander_.setpoint_.velocity.y - self.commander_.local_vel_.twist.linear.y
        localVelErr_msg.point.z = self.commander_.setpoint_.velocity.z - self.commander_.local_vel_.twist.linear.z

        self.localVel_err_pub_.publish(localVelErr_msg)

        # Body velocity errors
        # this message uses convention of +x-right, +y-forward, +z-up
        # the setpoint msg follows the same convention
        # However, the feedback signal (actual body vel from mavros) follows +x-forward, +y-left, +z-up
        # Required conversion is done below
        bodyVelErr_msg = PointStamped()
        bodyVelErr_msg.header.stamp = rospy.Time.now()
        bodyVelErr_msg.point.x = self.commander_.setpoint_.velocity.x - (-self.commander_.body_vel_.twist.linear.y)
        bodyVelErr_msg.point.y = self.commander_.setpoint_.velocity.y - self.commander_.body_vel_.twist.linear.x
        bodyVelErr_msg.point.z = self.commander_.setpoint_.velocity.z - self.commander_.body_vel_.twist.linear.z

        self.bodyVel_err_pub_.publish(bodyVelErr_msg)

        # Local position errors
        localPosErr_msg = PointStamped()
        localPosErr_msg.header.stamp = rospy.Time.now()
        localPosErr_msg.point.x = self.local_xSp_ - self.commander_.drone_pos_.x
        localPosErr_msg.point.y = self.local_ySp_ - self.commander_.drone_pos_.y
        localPosErr_msg.point.z = self.local_zSp_ - self.commander_.drone_pos_.z

        self.localPos_err_pub_.publish(localPosErr_msg)

        # Relative position errors
        relPosErr_msg = PointStamped()
        relPosErr_msg.header.stamp = rospy.Time.now()
        relPosErr_msg.point.x = self.relative_xSp_
        relPosErr_msg.point.y = self.relative_ySp_
        relPosErr_msg.point.z = self.relative_zSp_

        self.relativePos_err_pub_.publish(relPosErr_msg)


class DecisionMaking:
    def __init__(self):
        self.method = 'Without Game theory'
        self.uav_psi = 0.0
        self.uav_theta = 0.0
        self.cam_psi = 0.0
        self.cam_theta = 0.0

        self.BI = np.zeros((1, 2))

        self.LeftPlayer = 1000 * np.ones((4, 4))
        self.RightPlayer = 1000 * np.ones((4, 4))
        self.L_Theta = np.zeros((4, 4))
        self.R_Theta = np.zeros((4, 4))

    def BackwardInduction(self, matrix1, matrix2, whois):
        i = 1
        j = 1
        counter = 0
        Nextindex1 = np.zeros((8, 2))
        Nextindex2 = np.zeros((6, 2))
        if whois == 1:
            Player2matrix = matrix2.copy()
            Player1matrix = matrix1.copy()  # leader
        elif whois == 2:
            Player2matrix = matrix1.copy()
            Player1matrix = matrix2.copy()  # leader
    #########################################################
        # First Quarter
        if Player2matrix[i, j] < Player2matrix[i, j - 1]:
            Nextindex1[counter, :] = [i, j]
        else:
            Nextindex1[counter, :] = [i, j - 1]

        if Player2matrix[i - 1, j] < Player2matrix[i - 1, j - 1]:
            Nextindex2[counter, :] = [i - 1, j]
        else:
            Nextindex2[counter, :] = [i - 1, j - 1]

        counter += 1
    #########################################################
        # Second Quarter
        if Player2matrix[i, j + 2] < Player2matrix[i, j + 1]:
            Nextindex1[counter, :] = [i, j + 2]
        else:
            Nextindex1[counter, :] = [i, j + 1]

        if Player2matrix[i - 1, j + 2] < Player2matrix[i - 1, j + 1]:
            Nextindex2[counter, :] = [i - 1, j + 2]
        else:
            Nextindex2[counter, :] = [i - 1, j + 1]

        counter += 1
    #########################################################
        # Third Quarter
        if Player2matrix[i + 2, j] < Player2matrix[i + 2, j - 1]:
            Nextindex1[counter, :] = [i + 2, j]
        else:
            Nextindex1[counter, :] = [i + 2, j - 1]

        if Player2matrix[i + 1, j] < Player2matrix[i + 1, j - 1]:
            Nextindex2[counter, :] = [i + 1, j]
        else:
            Nextindex2[counter, :] = [i + 1, j - 1]

        counter += 1
    #########################################################
        # Fourth Quarter
        if Player2matrix[i + 2, j + 2] < Player2matrix[i + 2, j + 1]:
            Nextindex1[counter, :] = [i + 2, j + 2]
        else:
            Nextindex1[counter, :] = [i + 2, j + 1]

        if Player2matrix[i + 1, j + 2] < Player2matrix[i + 1, j + 1]:
            Nextindex2[counter, :] = [i + 1, j + 2]
        else:
            Nextindex2[counter, :] = [i + 1, j + 1]

        counter += 1
    #########################################################
        # Second Step
        for k in range(0, 3, 2):
            l1 = Nextindex1[k, 0]
            r1 = Nextindex1[k, 1]

            l2 = Nextindex2[k, 0]
            r2 = Nextindex2[k, 1]

            if Player1matrix[int(l1), int(r1)] < Player1matrix[int(l2), int(r2)]:
                Nextindex1[counter, :] = Nextindex1[k, :]
            else:
                Nextindex1[counter, :] = Nextindex2[k, :]
    #########################################################
            l1 = Nextindex1[k + 1, 0]
            r1 = Nextindex1[k + 1, 1]

            l2 = Nextindex2[k + 1, 0]
            r2 = Nextindex2[k + 1, 1]

            if Player1matrix[int(l1), int(r1)] < Player1matrix[int(l2), int(r2)]:
                Nextindex2[counter, :] = Nextindex1[k + 1, :]
            else:
                Nextindex2[counter, :] = Nextindex2[k + 1, :]
            counter += 1
    #########################################################
        # Third Step
        for n in range(4, 6):
            l1 = Nextindex1[n, 0]
            r1 = Nextindex1[n, 1]

            l2 = Nextindex2[n, 0]
            r2 = Nextindex2[n, 1]

            if Player2matrix[int(l1), int(r1)] < Player2matrix[int(l2), int(r2)]:
                Nextindex1[counter, :] = Nextindex1[n, :]
            else:
                Nextindex1[counter, :] = Nextindex2[n, :]
            counter += 1
    #########################################################
        # Fourth Step
        l1 = Nextindex1[6, 0]
        r1 = Nextindex1[6, 1]

        l2 = Nextindex1[7, 0]
        r2 = Nextindex1[7, 1]

        if Player1matrix[int(l1), int(r1)] < Player1matrix[int(l2), int(r2)]:
            self.BI = Nextindex1[6, :]
        else:
            self.BI = Nextindex1[7, :]

    def decision(self, uavPsi, uavTheta, camPsi, camTheta, x_err, y_err):
        if self.method == 'Without Game theory':
            self.uav_psi = uavPsi
            self.uav_theta = uavTheta
            self.cam_psi = camPsi
            self.cam_theta = camTheta

        elif self.method == 'First Algorithm':
            # Camera (Row Player) strategies
            if x_err > 0 and y_err < 0: # L U (Picture 1'st Quarter)
                row = 0
            elif x_err > 0 and y_err > 0: # L D (Picture 4'th Quarter)
                row = 1
            elif x_err < 0 and y_err < 0: # R U (Picture 2'nd Quarter)
                row = 2
            elif x_err < 0 and y_err > 0: # R D (Picture 3'rd Quarter)
                row = 3

            # UAV (Column Player) strategies
            if x_err > 0 and y_err < 0: # L U
                col = 0
            elif x_err > 0 and y_err > 0: # L D
                col = 1
            elif x_err < 0 and y_err < 0: # R U
                col = 2
            elif x_err < 0 and y_err > 0: # R D
                col = 3

            self.LeftPlayer[row, col] = sqrt(camPsi**2 + camTheta**2)
            self.RightPlayer[row, col] = sqrt(uavPsi**2 + uavTheta**2)
            self.L_Theta[row, col] = atan(camTheta/camPsi)
            self.R_Theta[row, col] = atan(uavTheta/uavPsi)
            # Leaf Algorithm
            self.BackwardInduction(self.LeftPlayer, self.RightPlayer, 1)
            l1 = self.BI[0]
            r1 = self.BI[1]
            self.BackwardInduction(self.LeftPlayer, self.RightPlayer, 2)
            l2 = self.BI[0]
            r2 = self.BI[1]

            if (sqrt( self.LeftPlayer[int(l1), int(r1)]**2 + self.RightPlayer[int(l1), int(r1)]**2 ) <
                sqrt( self.LeftPlayer[int(l2), int(r2)]**2 + self.RightPlayer[int(l2), int(r2)]**2 )):
                # Left Player (Camera) is optimal leader
                self.cam_psi = self.LeftPlayer[int(l1), int(r1)] * cos(self.L_Theta[int(l1), int(r1)])
                self.cam_theta = self.LeftPlayer[int(l1), int(r1)] * sin(self.L_Theta[int(l1), int(r1)])
            else:
                # Right Player (UAV) is optimal leader
                self.uav_psi = self.RightPlayer[int(l2), int(r2)] * cos(self.R_Theta[int(l2), int(r2)])
                self.uav_theta = self.RightPlayer[int(l2), int(r2)] * sin(self.R_Theta[int(l2), int(r2)])

        elif self.method == 'Second Algorithm':

            # Camera (Row Player) strategies
            if x_err > 0 and y_err < 0: # L U (Picture 1'st Quarter)
                row = 0
            elif x_err > 0 and y_err > 0: # L D (Picture 4'th Quarter)
                row = 1
            elif x_err < 0 and y_err < 0: # R U (Picture 2'nd Quarter)
                row = 2
            elif x_err < 0 and y_err > 0: # R D (Picture 3'rd Quarter)
                row = 3

            # UAV (Column Player) strategies
            if x_err > 0 and y_err < 0: # L U
                col = 0
            elif x_err > 0 and y_err > 0: # L D
                col = 1
            elif x_err < 0 and y_err < 0: # R U
                col = 2
            elif x_err < 0 and y_err > 0: # R D
                col = 3

            self.LeftPlayer[row, col] = sqrt(camPsi**2 + camTheta**2)
            self.RightPlayer[row, col] = sqrt(uavPsi**2 + uavTheta**2)
            self.L_Theta[row, col] = atan(camTheta/camPsi)
            self.R_Theta[row, col] = atan(uavTheta/uavPsi)

            V_P1 = np.amin(np.amax(self.LeftPlayer, axis=0))
            Loc_P1 = np.argmin(np.argmax(self.LeftPlayer, axis=0))
            V_P2 = np.amin(np.amax(self.RightPlayer, axis=1))
            Loc_P2 = np.argmin(np.argmax(self.RightPlayer, axis=1))
            adloc_P1 = Loc_P1
            adloc_P2 = Loc_P2

            # Using admissible Nash equilibrium in case of different equilibriums
            if self.LeftPlayer[Loc_P1, Loc_P2] != V_P1 and self.RightPlayer[Loc_P1, Loc_P2] != V_P2:
                if V_P1 < V_P2:
                    for k in range(4):
                        if self.LeftPlayer[Loc_P1, k] == V_P1:
                            V_P2 = self.RightPlayer[Loc_P1, k]
                            adloc_P2 = k
                elif V_P2 < V_P1:
                    for k in range(4):
                        if self.RightPlayer[k, Loc_P2] == V_P2:
                            V_P1 = self.LeftPlayer[k, Loc_P2]
                            adloc_P1 = k

            self.cam_psi = self.LeftPlayer[Loc_P1, Loc_P2] * cos(self.L_Theta[Loc_P1, Loc_P2])
            self.cam_theta = self.LeftPlayer[Loc_P1, Loc_P2] * sin(self.L_Theta[Loc_P1, Loc_P2])
            self.uav_psi = self.RightPlayer[Loc_P1, Loc_P2] * cos(self.R_Theta[Loc_P1, Loc_P2])
            self.uav_theta = self.RightPlayer[Loc_P1, Loc_P2] * sin(self.R_Theta[Loc_P1, Loc_P2])

    def saver(self):
        np.savetxt('/home/hamid/catkin_ws/Husky_X.csv', Husky_X, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Husky_Y.csv', Husky_Y, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Husky_Z.csv', Husky_Z, delimiter=",")

        np.savetxt('/home/hamid/catkin_ws/Drone_X.csv', Drone_X, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Drone_Y.csv', Drone_Y, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Drone_Z.csv', Drone_Z, delimiter=",")

        np.savetxt('/home/hamid/catkin_ws/Drone_Yaw.csv', Drone_Yaw, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Drone_Pitch.csv', Drone_Pitch, delimiter=",")

        np.savetxt('/home/hamid/catkin_ws/Cam_Yaw.csv', Cam_Yaw, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Cam_Pitch.csv', Cam_Pitch, delimiter=",")

        np.savetxt('/home/hamid/catkin_ws/Sim_time.csv', Sim_time, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Sim_time_ERR.csv', Sim_time_ERR, delimiter=",")

        np.savetxt('/home/hamid/catkin_ws/X_err_tmp.csv', X_err_tmp, delimiter=",")
        np.savetxt('/home/hamid/catkin_ws/Y_err_tmp.csv', Y_err_tmp, delimiter=",")
    #################################################################################################
        fig = plt.figure(dpi=300)
        # fig.set_size_inches(19.2, 10.8)
        ax = fig.gca(projection='3d')
        ax.plot(Husky_X, Husky_Y, Husky_Z, label='UGV Trajectory')
        ax.plot(Drone_X, Drone_Y, Drone_Z, label='UAV Trajectory')
        ax.legend()
        # ax.set_xlim(0, 20)
        # ax.set_ylim(0, 20)
        # ax.set_zlim(0, 5)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(self.method)
        ax.view_init(20, -120)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '.eps')
        ax.view_init(0, 0)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '00.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '00.eps')
        ax.view_init(0, 90)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '090.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '090.eps')
        ax.view_init(0, 180)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '0180.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '0180.eps')
        ax.view_init(90, 0)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '900.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '900.eps')
        ax.view_init(90, 90)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '9090.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '9090.eps')
        ax.view_init(90, 180)
        fig.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '90180.png')
        plt.savefig('/home/hamid/catkin_ws/' + self.method + '90180.eps')
    #################################################################################################
        fig1 = plt.figure(dpi=300)
        ax1 = fig1.gca()
        ax1.plot(Sim_time, Cam_Yaw)
        ax1.set_xlabel('Time (sec)')
        ax1.set_ylabel('$Camera_{\psi}$ (degrees)')
        ax1.set_title(self.method)
        fig1.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/Cam_Yaw.png')
        plt.savefig('/home/hamid/catkin_ws/Cam_Yaw.eps')

        fig2 = plt.figure(dpi=300)
        ax2 = fig2.gca()
        ax2.plot(Sim_time, Cam_Pitch)
        ax2.set_xlabel('Time (sec)')
        ax2.set_ylabel(r'$Camera_{\theta}$ (degrees)')
        ax2.set_title(self.method)
        fig2.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/Cam_Pitch.png')
        plt.savefig('/home/hamid/catkin_ws/Cam_Pitch.eps')

        fig3 = plt.figure(dpi=300)
        ax3 = fig3.gca()
        ax3.plot(Sim_time, Drone_Yaw)
        ax3.set_xlabel('Time (sec)')
        ax3.set_ylabel('$UAV_{\psi}$ (degrees)')
        ax3.set_title(self.method)
        fig3.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/UAV_Yaw.png')
        plt.savefig('/home/hamid/catkin_ws/UAV_Yaw.eps')

        fig4 = plt.figure(dpi=300)
        ax4 = fig4.gca()
        ax4.plot(Sim_time, Drone_Pitch)
        ax4.set_xlabel('Time (sec)')
        ax4.set_ylabel(r'$UAV_{\theta}$ (degrees)')
        ax4.set_title(self.method)
        fig4.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/UAV_Pitch.png')
        plt.savefig('/home/hamid/catkin_ws/UAV_Pitch.eps')

        fig5 = plt.figure(dpi=300)
        ax5 = fig5.gca()
        ax5.plot(Sim_time_ERR, X_err_tmp)
        ax5.set_xlabel('Time (sec)')
        ax5.set_ylabel('$Error_x$ (pixels)')
        ax5.set_title(self.method)
        fig5.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/X_Err.png')
        plt.savefig('/home/hamid/catkin_ws/X_Err.eps')

        fig6 = plt.figure(dpi=300)
        ax6 = fig6.gca()
        ax6.plot(Sim_time_ERR, Y_err_tmp)
        ax6.set_xlabel('Time (sec)')
        ax6.set_ylabel('$Error_y$ (pixels)')
        ax6.set_title(self.method)
        fig6.tight_layout()
        plt.savefig('/home/hamid/catkin_ws/Y_Err.png')
        plt.savefig('/home/hamid/catkin_ws/Y_Err.eps')
#################################################################################################
if __name__ == '__main__':
    rospy.init_node('Offboard_control_node', anonymous=True)


    tracker = Tracker()
    dm = DecisionMaking()
    # UAV constants
    Yaw_min = -90.0
    Yaw_max = 90.0
    Pitch_min = -35.0
    Pitch_max = 35.0
    # Camera constants
    Pan_min = -75.0
    Pan_max = 75.0
    Tilt_min = -90.0
    Tilt_max = 30.0

    step = 0.0
    loop = rospy.Rate(20)

    Husky_X = []
    Husky_Y = []
    Husky_Z = []

    Drone_X = []
    Drone_Y = []
    Drone_Z = []

    Drone_Yaw = []
    Drone_Pitch = []

    Cam_Yaw = []
    Cam_Pitch = []

    Sim_time = []
    Sim_time_ERR = []

    X_err_tmp = []
    Y_err_tmp = []

    while not rospy.is_shutdown():

        step += 1
        """
        # Example of how to use the PositionController

        K = PositionController() # Should be created outside ROS while loop

        # The following should be inside ROS while loop
        # update errors
        K.ex_ = relative_position_in_x # body directions
        K.ey_ = relative_position_in_y
        K.ez_ = relative_position_in_z

        cmd.vel_setpoint_.x, cmd.vel_setpoint_.y, cmd.vel_setpoint_.z = K.computeVelSetpoint()
        cmd.setBodyVelMask()
        # Then, publish command as below (publishSetpoint)

        """
        tracker.computeControlOutput()
#################################################################################################
        if tracker.tagpxl_.valid:
            X_Err = tracker.tagpxl_.p_image[0] - tracker.tagpxl_.x_c # Horizontal Error
            Y_Err = tracker.tagpxl_.p_image[1] - tracker.tagpxl_.y_c # Vertical Error

            if tracker.tagpxl_.width < X_Err:
                X_Err = tracker.tagpxl_.width
            elif X_Err < -tracker.tagpxl_.width:
                X_Err = tracker.tagpxl_.width

            if tracker.tagpxl_.height < Y_Err:
                Y_Err = tracker.tagpxl_.height
            elif Y_Err < -tracker.tagpxl_.height:
                Y_Err = tracker.tagpxl_.height

            X_ratio = X_Err / tracker.tagpxl_.width
            Y_ratio = Y_Err / tracker.tagpxl_.height
#################################################################################################
            UAV_PSI = X_ratio * (Yaw_max - Yaw_min)
            if UAV_PSI > Yaw_max:
                UAV_PSI = Yaw_max
            elif UAV_PSI < Yaw_min:
                UAV_PSI = Yaw_min
#################################################################################################
            UAV_THETA = Y_ratio * (Pitch_max - Pitch_min)
            if UAV_THETA > Pitch_max:
                UAV_THETA = Pitch_max
            elif UAV_THETA < Pitch_min:
                UAV_THETA = Pitch_min
#################################################################################################
            CAM_PSI = X_ratio * (Pan_max - Pan_min)
            if CAM_PSI > Pan_max:
                CAM_PSI = Pan_max
            elif CAM_PSI < Pan_min:
                CAM_PSI = Pan_min
#################################################################################################
            CAM_THETA = Y_ratio * (Tilt_max - Tilt_min)
            if CAM_THETA > Tilt_max:
                CAM_THETA = Tilt_max
            elif CAM_THETA < Tilt_min:
                CAM_THETA = Tilt_min
#################################################################################################
            dm.method = 'Without Game theory'
            # dm.method = 'First Algorithm'
            # dm.method = 'Second Algorithm'
            dm.decision(UAV_PSI, UAV_THETA, CAM_PSI, CAM_THETA, X_Err, Y_Err)
#################################################################################################
            tracker.commander_.yaw_setpoint_ = dm.uav_psi
            tracker.commander_.drone_pos_.z += sqrt((tracker.tagpxl_.pos_x**2) +
                                                    (tracker.tagpxl_.pos_y**2) +
                                                    (tracker.tagpxl_.pos_z**2)) * sin(dm.uav_theta)
            tracker.commander_.cam_yaw_setpoint_ = dm.cam_psi
            tracker.commander_.cam_pitch_setpoint_ = dm.cam_theta

            X_err_tmp = np.append(X_err_tmp, X_Err)
            Y_err_tmp = np.append(Y_err_tmp, Y_Err)
            Sim_time_ERR = np.append(Sim_time_ERR, rospy.get_time())
#################################################################################################
        Husky_X = np.append(Husky_X, tracker.commander_.husky_x_)
        Husky_Y = np.append(Husky_Y, tracker.commander_.husky_y_)
        Husky_Z = np.append(Husky_Z, tracker.commander_.husky_z_)

        Drone_X = np.append(Drone_X, tracker.commander_.drone_pos_.x)
        Drone_Y = np.append(Drone_Y, tracker.commander_.drone_pos_.y)
        Drone_Z = np.append(Drone_Z, tracker.commander_.drone_pos_.z)

        Drone_Yaw = np.append(Drone_Yaw, tracker.commander_.yaw_setpoint_)
        Drone_Pitch = np.append(Drone_Pitch, dm.uav_theta)

        Cam_Yaw = np.append(Cam_Yaw, tracker.commander_.cam_yaw_setpoint_)
        Cam_Pitch = np.append(Cam_Pitch, tracker.commander_.cam_pitch_setpoint_)

        Sim_time = np.append(Sim_time, rospy.get_time())
#################################################################################################
        tracker.commander_.publishSetpoint()
        tracker.commander_.publishCamSetpoint()
        tracker.publishErrorSignals()
        # cmd.publishSetpoint()
        rospy.on_shutdown(dm.saver)
        loop.sleep()
#################################################################################################
