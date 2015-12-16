# ========================================================================
# Copyright 2015 BRAIN Corporation. All rights reserved. This software is
# provided to you under BRAIN Corporation's Beta License Agreement and
# your use of the software is governed by the terms of that Beta License
# Agreement, found at http://www.braincorporation.com/betalicense.
# ========================================================================

'''
calculates odom position using wheel encoder and imu data.
'''
import math
import logging
import numpy as np


class DiffDriveKinematics(object):
    '''
    Simple diff. wheel kinematics based on wheel encoder ticks
    '''
    def __init__(self, wheel_radius, wheel_base):
        self._wheel_radius = wheel_radius
        self._wheel_base = wheel_base

    def forward(self, encoder_deltas):
        ds = self._wheel_radius * (encoder_deltas[0] + encoder_deltas[1])/2.0
        dheading = self._wheel_radius * (encoder_deltas[0] - encoder_deltas[1])/self._wheel_base
        return ds, dheading


class LowsheenOdometryEstimator(object):
    def __init__(self, wheel_radius, wheel_base, ticks_per_revolution, use_imu_heading=True,
                 logger=None):
        '''
        Parameters
        ----------
        wheel_radius: float
            wheel radius in m
        wheel_base: float
            wheel base in m
        ticks_per_revolution: init
            number of encoder ticks (only one channel) per revolution
        use_imu_heading: boolean
            set this to true to override heading based on imu instead of wheel encoder
        logger: object with standard python logger interface that used to print logging message. \
            If used inside ROS, RospyLogger should be passed in, otherwise you will not see the log.
        '''
        if logger is None:
            logger = logging.getLogger()
            logger.setLevel(logging.INFO)
        self._logger = logger

        self.last_encoder_ticks = None
        MAX_UINT16 = np.iinfo(np.uint16).max
        self.max_timestamp = MAX_UINT16
        self.ticks_to_rad = (2 * math.pi / ticks_per_revolution)
        self.diff_drive = DiffDriveKinematics(wheel_radius, wheel_base)
        self.use_imu_heading = use_imu_heading
        self.reset()

    def get_pose_covariance(self):
        '''
        :returns float
            yaw pose covariance of the estimate
        Odometry yaw covariance must be much bigger than the covariance provided
        by the imu, as the later takes much better measures.
        TODO: figure out realistic values.
        '''
        return 0.05 if self.use_imu_heading else 0.2

    def reset(self):
        self.last_timestamp = -1
        self.position_ticks = np.array([0, 0], dtype='int')
        self.last_encoder_ticks = None
        self.x, self.y, self.heading = 0., 0., 0.
        self.bno_x, self.bno_y = 0., 0.
        self._bad_angular_velocity_estimate_counter = 0

    def process_mule_state(self, mule_state):
        '''
        computes the new odom values based on the mule encoder ticks and heading angle.
        publishes the measured data to all ros topics
        '''
        motor_encoders = np.array(mule_state.motor_encoders)
        motor_encoders[0] = -motor_encoders[0]
        if self.last_encoder_ticks is None:
            self.last_encoder_ticks = motor_encoders
            self.last_timestamp = mule_state.timestamp
            return None
        else:
            # calculate deltas from last state
            logging.debug("Mule: %d, encoder = [%d, %d], heading = %4.4f, ang. vel = %4.4f" % 
                          (mule_state.timestamp, motor_encoders[0], 
                           motor_encoders[1], 
                           mule_state.heading_angle, 
                           mule_state.angular_velocity))
            #for testing assign some random ticks from last ticks
            delta_encoder_ticks = motor_encoders - self.last_encoder_ticks
            delta_timestamp = mule_state.timestamp - self.last_timestamp
            delta_timestamp = delta_timestamp if (delta_timestamp > 0) else \
                delta_timestamp + self.max_timestamp
            # convert from int ms to seconds
            delta_timestamp = delta_timestamp/1000.

            # update differential drive estimates
            ds, dencoders_heading = self.diff_drive.forward(delta_encoder_ticks * self.ticks_to_rad)
            encoders_angular_velocity = dencoders_heading/delta_timestamp
            imu_angular_velocity = mule_state.angular_velocity

            self._detect_encoders_and_gyro_deviations(encoders_angular_velocity, imu_angular_velocity)

            # imu override the wheel based calculations
            if (self.use_imu_heading):
                angular_velocity = imu_angular_velocity
                self.heading = mule_state.heading_angle
            else:
                angular_velocity = encoders_angular_velocity
                self.heading = self.heading + angular_velocity*delta_timestamp
                # limit the angle to -pi to pi
                self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

            self.x = self.x + ds*math.cos(self.heading)
            self.y = self.y + ds*math.sin(self.heading)

            self.bno_x = self.bno_x + ds*math.cos(angular_velocity)
            self.bno_y = self.bno_y + ds*math.sin(angular_velocity)

            linear_velocity = ds/delta_timestamp

            # update the state
            self.last_timestamp = mule_state.timestamp
            self.position_ticks += delta_encoder_ticks
            self.last_encoder_ticks = motor_encoders

            return self.x, self.y, self.heading, linear_velocity, angular_velocity, self.bno_x, self.bno_y

    def _detect_encoders_and_gyro_deviations(self, encoders_angular_velocity, imu_angular_velocity):
            if abs(encoders_angular_velocity - imu_angular_velocity) > 0.1:
                self._bad_angular_velocity_estimate_counter += 1
                if self._bad_angular_velocity_estimate_counter > 3:
                    self._logger.error("Large deviation between imu and encoders estimate of angular velocity (%s vs %s)" %
                                       (imu_angular_velocity, encoders_angular_velocity))
                    self._bad_angular_velocity_estimate_counter = 0
            else:
                self._bad_angular_velocity_estimate_counter = 0
