# ========================================================================
# Copyright 2015 BRAIN Corporation. All rights reserved. This software is
# provided to you under BRAIN Corporation's Beta License Agreement and
# your use of the software is governed by the terms of that Beta License
# Agreement, found at http://www.braincorporation.com/betalicense.
# ========================================================================

'''
This file has all necessary ways to publish imu, odom, tf and joint_wheel_state.
'''
import tf
import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose2D, TwistWithCovarianceStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from odom_estimator import LowsheenOdometryEstimator



class LowsheenOdometryPublisher(object):
    def __init__(self):
        '''
        Parameters
        ----------
        pose_covariance: float
            odometry yaw covariance
        '''

        # Encoder: 10,000 ticks per revolution (40,000 if you count all rising and falling edges on both A and B channels)
        # Wheel Diameter: 13", Wheel Base: 25"
        self.odometry_estimator = LowsheenOdometryEstimator(
            wheel_radius=0.1651, wheel_base=0.635, ticks_per_revolution=40000, use_imu_heading=True)

        self.odom_pub_stm = rospy.Publisher('/lowsheen/odom_stm', Odometry, queue_size=10)
        self.odom_pub_bno = rospy.Publisher('/lowsheen/odom_bno', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/lowsheen/sensors/imu', Imu, queue_size=10)
        self.wheel_pub = rospy.Publisher('/lowsheen/sensors/wheel_encoders', TwistWithCovarianceStamped, queue_size=10)
        self.bno_odom_broadcaster = tf.TransformBroadcaster()
        self.stm_odom_broadcaster = tf.TransformBroadcaster()

        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        self.pose_covariance = self.odometry_estimator.get_pose_covariance()
        self.gyro_start_time = self.accel_start_time = self.encoder_start_time = rospy.get_time()

    def publish(self, mule_state):

        odometery_estimate = self.odometry_estimator.process_mule_state(mule_state)

        # publish odom and tf
        if not odometery_estimate is None:
            self.publish_odometry(*odometery_estimate)
            self.publish_wheel_encoders(odometery_estimate[2], odometery_estimate[3], odometery_estimate[4])
            #self.publish_imu(mule_state, odometery_estimate[2], odometery_estimate[4])

            #self.publish_transform(odometery_estimate[0], odometery_estimate[1], odometery_estimate[2], odometery_estimate[4], odometery_estimate[5], odometery_estimate[6])



    def publish_imu(self, mule_state, heading, angular_velocity):
        '''
        publish the imu data
        '''
        if self.imu_pub.get_num_connections() > 0:
            imu = Imu()
            imu.header.frame_id = "odom"
            imu.header.stamp = rospy.Time.now()
            now = imu.header.stamp.secs + imu.header.stamp.nsecs*0.000000001
            if self.gyro_start_time == 0:
                self.gyro_start_time = now

            degree_to_rad = math.pi / 180.0
             # Rate noise density @ 50Hz Bandwidth
            gyro_rnd = 0.011 # degree*sqrt(Hz)/sec
            gyro_time = now - self.gyro_start_time
            # Accumulated error in gyro (standard deviation)
            gyro_std = gyro_rnd * math.sqrt(gyro_time) * degree_to_rad
            gyro_variance = gyro_std * gyro_std
                                                                                     
            imu.angular_velocity.z = angular_velocity
            imu.angular_velocity_covariance[0] = gyro_variance
            imu.angular_velocity_covariance[4] = gyro_variance
            imu.angular_velocity_covariance[8] = gyro_variance
 
            # publish the data
            self.imu_pub.publish(imu)

    def publish_transform(self, x, y, heading, angular_velocity, bno_x, bno_y):
        stm_odom_quat = tf.transformations.quaternion_from_euler(0, 0, heading)
        self.stm_odom_broadcaster.sendTransform(translation=(x, y, 0.0),
                                            rotation=stm_odom_quat,
                                            time=rospy.Time.now(),
                                            child="gyro_link_stm",
                                            parent=self.base_frame)
        bno_odom_quat = tf.transformations.quaternion_from_euler(0, 0, angular_velocity)
        self.bno_odom_broadcaster.sendTransform(translation=(bno_x, bno_y, 0.0),
                                            rotation=bno_odom_quat,
                                            time=rospy.Time.now(),
                                            child="gyro_link_bno",
                                            parent=self.base_frame)

    def publish_odometry(self, x, y, heading, linear_velocity, angular_velocity, bno_x, bno_y):
        '''
        publish the odom information 
        (values based on kobuki odometry publishing kobiki_node/src/library/odometry.cpp)
        '''
        odom_stm = Odometry()
        odom_bno = Odometry()
        odom_bno.header.stamp = odom_stm.header.stamp = rospy.Time.now()
        odom_bno.header.frame_id = self.base_frame
        odom_stm.header.frame_id = self.base_frame

        # position
        odom_stm_quat = tf.transformations.quaternion_from_euler(0, 0, heading)
        odom_bno_quat = tf.transformations.quaternion_from_euler(0, 0, angular_velocity)
        odom_bno.pose.pose.position.x = bno_x
        odom_stm.pose.pose.position.x = x
        odom_bno.pose.pose.position.y = bno_y
        odom_stm.pose.pose.position.y = y
        odom_bno.pose.pose.position.z = odom_stm.pose.pose.position.z = 0.
        odom_stm.pose.pose.orientation = Quaternion(*odom_stm_quat)
        odom_bno.pose.pose.orientation = Quaternion(*odom_bno_quat)

        # velocity
        # x, y switch to follow ros convention 
        odom_bno.twist.twist.linear.x = odom_stm.twist.twist.linear.x = linear_velocity
        odom_bno.twist.twist.linear.y = odom_stm.twist.twist.linear.y = 0.
        odom_bno.twist.twist.angular.z = odom_stm.twist.twist.angular.z = 0. # angular_velocity

        # Pose covariance (required by robot_pose_ekf)
        odom_bno.pose.covariance[0] = odom_stm.pose.covariance[0] = 0.1
        odom_bno.pose.covariance[7] = odom_stm.pose.covariance[7] = 0.1
        odom_bno.pose.covariance[35] = odom_stm.pose.covariance[35] = self.pose_covariance

        odom_bno.twist.covariance[0] = odom_stm.twist.covariance[0] = 0.005

        #odom_bno.pose.covariance[14] = odom_stm.pose.covariance[14] = np.finfo(np.float64).max  # set a non-zero covariance on unused
        #odom_bno.pose.covariance[21] = odom_stm.pose.covariance[21] = np.finfo(np.float64).max  # dimensions (z, pitch and roll); this
        #odom_bno.pose.covariance[28] = odom_stm.pose.covariance[28] = np.finfo(np.float64).max  # is a requirement of robot_pose_ekf
 
        self.odom_pub_stm.publish(odom_stm)
        self.odom_pub_bno.publish(odom_bno)

    def publish_wheel_encoders(self, heading, linear_velocity, angular_velocity):
        '''
        Publishes the wheel encoder velocity as a linear velocity (Twist)
        '''
        encoders = TwistWithCovarianceStamped()
        encoders.header.stamp = rospy.Time.now()
        encoders.header.frame_id = self.base_frame
        now = encoders.header.stamp.secs + encoders.header.stamp.nsecs*0.000000001
        if self.encoder_start_time == 0:
            self.encoder_start_time = now

        encoders.twist.twist.linear.x = linear_velocity

        # Wheel encoder error density
        encoder_err = 0.02
        encoder_time = now - self.encoder_start_time
        encoder_std = encoder_err * math.sqrt(encoder_time)
        encoder_variance = encoder_std * encoder_std

        #encoders.twist.covariance[0] = encoder_variance
        encoders.twist.covariance[0] = 0.005

        self.wheel_pub.publish(encoders)
