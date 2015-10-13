#!/usr/bin/env python
# ============================================================================
# Copyright 2015 BRAIN Corporation. All rights reserved. This software is
# provided to you under BRAIN Corporation's Beta License Agreement and
# your use of the software is governed by the terms of that Beta License
# Agreement, found at http://www.braincorporation.com/betalicense.
# ============================================================================


import imp
import rospy
from lowsheen.msg import MuleState
from odometry_publisher import LowsheenOdometryPublisher
from odom_estimator import LowsheenOdometryEstimator

#odompub = imp.load_source('LowsheenOdometryPublisher', '/home/adam/Projects/shining_software/catkin_ws/src/lowsheen/scripts/odometry_publisher.py')
#odomest = imp.load_source('LowsheenOdometryEstimator', '/home/adam/Projects/shining_software/catkin_ws/src/lowsheen/src/lowsheen_lib/odom_estimator.py')


class FixedBagOdom(object):
    def __init__(self):
        self.pub = LowsheenOdometryPublisher()
        self.est = LowsheenOdometryEstimator(
            wheel_radius=0.1651, wheel_base=0.635, ticks_per_revolution=40000)
        
        rospy.Subscriber("/lowsheen/sensors/sensor_state", MuleState, self.process_mule_state)

    def process_mule_state(self, mule_state):
        # Replace bad motor encoder data with a copy of working encoder
        #print mule_state.motor_encoders
        self.pub.publish(mule_state)
        #odom_estimate = self.est.process_mule_state(mule_state)
        #if odom_estimate is not None:
            #self.pub.publish(*odom_estimate)


if __name__ == '__main__':
    try:
        # initialize node
        rospy.init_node('mule_fixed_odom', anonymous=True)
        rospy.loginfo('Initializing mule_fixed_odom')

        # Read paramaters
        rate = rospy.Rate(30)

        fixed_odom = FixedBagOdom()

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass