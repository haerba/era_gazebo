#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning a Ego Vehicle in ROS

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""

from abc import abstractmethod

import os
import sys
import random
import math
import json
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import Odometry

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
        self.role_name = rospy.get_param('~role_name', 'hero2')
	print(self.role_name)
        # check argument and set spawn_point
        
        self.pose_subscriber = rospy.Subscriber("/carla/{}/odometry".format(self.role_name), Odometry, self.callback, queue_size = 1)
        rospy.loginfo('listening to server %s:%s', self.host, self.port)
	self.pose_publisher = rospy.Publisher("/{}/pose".format(self.role_name), PoseWithCovarianceStamped, queue_size = 1)
	
    def callback(self, odometry_msg):
	msg = PoseWithCovarianceStamped()
	msg.header = odometry_msg.header
        # Get the pose With Covariance
	msg.pose = odometry_msg.pose
	self.pose_publisher.publish(msg)
	
        
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

        rospy.loginfo("CARLA world available. Spawn ego vehicle...")

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
            channel_modeler.destroy()


if __name__ == '__main__':
    main()
