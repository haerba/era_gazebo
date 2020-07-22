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
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, Int16
from era_gazebo.msg import AttenuationMsg, GNURadioInformation
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
	self.map_subscriber = message_filters.Subscriber("/{}/combined_map".format(self.role_name2), OccupancyGrid)
	self.gnu_info = rospy.Publisher("/gnu_info", GNURadioInformation, queue_size = 1)
	self.ts = message_filters.ApproximateTimeSynchronizer([self.pose_subscriber1, self.pose_subscriber2], queue_size = 1, slop = 0.05)
	self.ts.registerCallback(self.callback)
		
    def callback(self, args1, args2, args3):
	walls = Int16()
	att = Float32()
	distance = Float32()
	msg = GNURadioInformation()
	msg.header = args1.header
	vector = [args1.pose.pose.position.x- args2.pose.pose.position.x, args1.pose.pose.position.y- args2.pose.pose.position.y, args1.pose.pose.position.z- args2.pose.pose.position.z]
	distance.data = np.linalg.norm(vector)
	msg.distance = distance
	pos1 = [args1.pose.pose.position.x, args1.pose.pose.position.y]
	pos2 = [args2.pose.pose.position.x, args2.pose.pose.position.y]
	walls.data = self.obstacle_calculator(pos1, pos2, args3.data, args3.info.width)
	att.data = self.channel_attenuation(distance.data, walls.data)
	msg.walls = walls
	msg.attenuation = att
	self.gnu_info.publish(msg)
      
    def channel_attenuation(self, dist, walls):
	att = dist*2+walls
	return att

    def obstacle_calculator(self, pos1, pos2, grid, width):
	walls = 0
	pos1_value = pos1[0]+pos1[1]*width
	pos2_value = pos2[0]+pos2[1]*width
	vector = [pos2[0] -pos1[0], pos2[1] -pos1[1]]
	k = 1
	if(vector[0] == 0):
	    up = abs(vector[1])/vector[1]
	    while(k <= np.linalg.norm(vector)):
		if(grid(pos1_value+width*up*k) > 0):
		    walls++
		k++
	elif(vector[1] == 0):
	    up = abs(vector[0])/vector[0]
	    while(k <= np.linalg.norm(vector)):
		if(grid(pos1_value+up*k) > 0):
		    walls++
	        k++
	else:
		

	return walls
      
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
