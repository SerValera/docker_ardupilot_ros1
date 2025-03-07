#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import Waypoint 

class GroundStation(object):
    def __init__(self):
        self.command = 0
        self.param_command = Waypoint()
        self.dict_commands = {
                    'WARNING OCCURIED': 0,
                    'Init': 1,
                    'Start race and recording': 2,
                    'Stop race and recording': 3,
                    'RC remote': 4,
                    'Gesture': 5,

                    'Drone: Arming': 11,
                    'Drone: Disarming': 12,
                    'Drone: Takeoff': 13,
                    'Drone: Land': 14,
                    'Drone: Set home pose': 15,
                    'Drone: RTL': 16,

                    '(from ground) Start local mission': 20,
                    '(in flight!) Move relative Home Pose': 21,
                    
                    'Start drone 1': 51,
                    'Start drone 2': 52,

                    'Start cyrcle Main': 54,
                    'Start cyrcle Aruco': 55,

                    'Start overtaking ': 60,
                    'Follow to Aruco ': 61, 

                    'Data set point ': 70,
                    'Set alt and r ': 71
                    }

        self.publish_command = rospy.Publisher('/drone_commands_gs', Waypoint, queue_size=10)
        self.rate = rospy.Rate(10)

    def get_coordinates_input(self):
        self.param_command.x_lat = float(input("local x: "))
        self.param_command.y_long = float(input("local y: "))
        self.param_command.z_alt = float(input("local z: "))

    def moveByUser(self):
        try:
            for key in sorted(self.dict_commands, key=self.dict_commands.get):
                print(str(self.dict_commands[key]) + " - " + key)
            print('q - Quit from terminal')
            
            self.param_command = Waypoint()
            self.command = str(input("Write number of command: "))

            if self.command == 'q':
                raise SystemExit

            self.param_command.command = int(self.command)       

            if self.command == "13": #takeoff, set altitude
                self.param_command.z_alt = float(input("altitude: "))

            if self.command == "20": #start local mission
                print('local mission in X direction (relative home position)')
                self.param_command.param1 = int(input('Number of steps:'))
                self.param_command.param2 = int(input('Time to data recording (int(sec)):'))
                self.param_command.x_lat = float(input('Length of step (meters) X:'))
                self.param_command.z_alt = float(input('Flight Altitude (meters) Z:'))

            if self.command == "21": #goto local pos (relatively local pose)
                self.param_command.param1 = int(input('choose mode: 0 - global, 1 - relative'))
                self.get_coordinates_input()
                self.param_command.param2 = float(input("YAW: "))

            if self.command == "71": #start local mission
                print('For data set collection, altitude and radiouse ')
                self.param_command.param1 = float(input('Z:'))
                self.param_command.param2 = float(input('R:'))

            if self.command == "q":
                print('q')

            self.publish_command.publish(self.param_command) 

        except SyntaxError or ValueError:
            self.command=0


if __name__ == '__main__':
    try:
        rospy.init_node('ground_station', anonymous=True)
        node = GroundStation()
        while not rospy.is_shutdown():
            node.moveByUser()

    except rospy.ROSInterruptException:
        pass
