#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from abc import abstractmethod

import os
import sys
import random
import numpy as np
import math
import json
import rospy
import message_filters
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Vector3Stamped
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import Odometry
from era_gazebo.msg import AttenuationMsg
import carla

secure_random = random.SystemRandom()

# ==============================================================================
# -- CarlaEgoVehicle ------------------------------------------------------------
# ==============================================================================


class CarlaChannelModeler(object):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        rospy.init_node('carla_channel_modeler', anonymous=True)
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self.timeout = rospy.get_param('/carla/timeout', '2')
        self.world = None
        self.player = None
        self.role_name1 = 'hero1'
	self.role_name2 = 'hero2'
        # check argument and set spawn_point
        
        self.pose_subscriber1 = message_filters.Subscriber("/carla/{}/odometry".format(self.role_name1), Odometry)
	self.pose_subscriber2 = message_filters.Subscriber("/carla/{}/odometry".format(self.role_name2), Odometry)
	self.attenuation_pub = rospy.Publisher("/carla/attenuation", PoseWithCovarianceStamped, queue_size = 1)
	self.ts = message_filters.ApproximateTimeSynchronizer([self.pose_subscriber1, self.pose_subscriber2], queue_size = 1, slop = 0.05)
	self.ts.registerCallback(self.callback)
		
    def callback(self, args1, args2):
	msg = AttenuationMsg()
	msg.header = args1.header
	vector = [args1.pose.pose.position.x- args2.pose.pose.position.x, args1.pose.pose.position.y- args2.pose.pose.position.y, args1.pose.pose.position.z- args2.pose.pose.position.z]
	norm = np.linalg.norm(vector)
	print(norm)
	# att = self.channel_attenuation(norm)
	# msg.attenuation = att
	# self.attenuation_pub(msg)
      
    def channel_attenuation(self, dist):
	att = dist*2
	return att
      
    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
            sys.exit(1)

        rospy.loginfo("CARLA world available. Computing Attenuation...")

        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    channel_modeler = CarlaChannelModeler()
    try:
        channel_modeler.run()
    finally:
        if channel_modeler is not None:
            print('Done')


if __name__ == '__main__':
    main()
