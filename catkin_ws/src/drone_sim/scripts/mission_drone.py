#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import Waypoint
from datetime import datetime

import time
import math
import numpy as np
import csv

from drone_autopilot import Drone

class DroneMission:
    def __init__(self):
        
        self.drone = Drone()

        self.path_test = []
        self.generate_cyrcle()

        " Command variables "
        self.id_task_prev = None
        self.is_command_new = True

        " Mission executuion topic "
        rospy.Subscriber('/drone_commands_gs', Waypoint, self.callback_commands) #from ground_station.py

        self.rate = rospy.Rate(20)
        self.trajectory = []
        
 

    def callback_commands(self, msg):
        """ Callback function for getting data and commands form ground_station.py
        from topic '/drone_commands_gs' 

        Args:
            msg (Waypoint): message in Waypoint format.
        """        
        print("ASDASDASD")
        param_command = msg
        id_task = param_command.command
        rospy.loginfo('Get id task: ' + str(id_task))

        if self.id_task_prev != id_task:
            self.id_task_prev = id_task
            self.is_command_new = True

        if self.is_command_new == True:
            if id_task == 0:
                print("So test")
                
            if id_task == 1:
                print("TEST vel_publisher")
                self.drone.pub_vel([1.0, 0.0, 0.0], yaw_rot=0)       

            """ Drone control test commands """
            if id_task == 11: #arm
                self.drone.arm()
            if id_task == 12: #disarm
                self.drone.dis_arm()
            if id_task == 13: #takeoff
                attitude = float(param_command.z_alt)
                self.drone.arm()
                self.drone.takeoff(height=attitude)
            if id_task == 14: #land
                self.drone.land()
            if id_task == 15: #set home
                self.drone.set_new_home_pos()
            if id_task == 16: #RTL mode
                self.drone.set_mode("RTL")
            
            if id_task == 21:  # 'Go to local pos (in flight!)
                m = int(param_command.param1)
                point = [float(param_command.x_lat), float(
                    param_command.y_long), float(param_command.z_alt)]
                yaw_input = float(param_command.param2)
                self.drone.goTo(point, mode=m, yaw=yaw_input)

            list_waypoints = [[0,0,3,180], [0,5,3,90], [5,5,3,0], [5,0,3,270]]
            
            if id_task == 22:
                for point in list_waypoints:
                    self.drone.goTo([point[0], point[1], point[2]], mode="global")

            if id_task == 23:
                for point in self.path_test:
                    self.drone.goTo([point[0], point[1], point[2]], mode="global")

    def go_demo_mission(self):
        with open('/home/sim/ardupilot_docker/trajectory_konstantin.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header (x, y, z)
            for row in reader:
                self.trajectory.append([float(row[1]), float(row[0]), float(row[2])])

        self.drone.takeoff(height=1.0)

        rospy.sleep(2.0)

        for point in self.trajectory:
            self.drone.goTo([-point[0], point[1], point[2]], mode="global")

        self.drone.goTo([0, 0, 1.0], mode="global")

        rospy.sleep(2.0)

        self.drone.land()

    def generate_cyrcle(self):
        a = -10
        b = 8
        r = 7.5
        stepSize = math.pi * 2 / 20
        positions = []
        t = 0
        while t < 2 * math.pi:
            positions.append([r * math.cos(t) + a, r * math.sin(t) + b, 3.0])
            t += stepSize
        self.path_test = positions
        self.drone.path_len = len(positions)
         

    def demo_flight_mission(self, event=None):

        with open('/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/scripts/trajectory.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header (x, y, z)
            for row in reader:
                self.trajectory.append([float(row[1]), float(row[0]), float(row[2])])

        # self.drone.takeoff(height=1.0)
        # rospy.sleep(1.0)
    
        " path "
        path_goal_id = self.drone.path_counter % len(self.trajectory)
        print(path_goal_id)
        goal_p = self.trajectory[path_goal_id]

        self.drone.goToVel([-goal_p[0], goal_p[1], goal_p[2]])
        self.drone.publish_id_path.publish(str(path_goal_id))



if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)

    mission = DroneMission()

    # Set point
    # mission.go_demo_mission()
    mission.drone.takeoff(height=1.0)
    rospy.sleep(10.0)

    # To vel control
    rospy.Timer(rospy.Duration(0.1), mission.demo_flight_mission)

    while not rospy.is_shutdown():
        # pass
        rospy.spin()
        
