#!/usr/bin/env python3

import os

from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, HomePosition, AttitudeTarget, Waypoint, RCIn, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome, StreamRate
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry

from mavros_msgs.msg import *
from mavros_msgs.srv import *

from math import *
from tf.transformations import *

from numpy.linalg import norm
import numpy as np

import rospy
import time

class Drone:
    def __init__(self):
        print("Ardupilot INIT")
        self.id = 0
        self.name = ''

        self.k_vel = 3.0
        self.k_vel_z = 4.0

        self.hz = 20
        self.rate = rospy.Rate(self.hz) #hz

        self.is_record_coords = False

        # --- Coordinates variables ---
        self.pose_local = None # local pose in PoseStamped object. Updates in callback
        self.pose_local_origin = None
        self.pose_global = None # global pose in NavSatFix object. Updates in callback
        self.pose_home = None # home pose in HomePosition. Will update after found gps pose
        self.current_orientation = None # current orientation
        self.relative_altitude = 0.0 # relative altitude 
        self.velocity = None
        self.alt_ground = 0.0
        self.sp = None
        self.last_waypoint = False

        self.cords_first_point = None
        
        self.pose_global_target = None # for setting target position 
        self.yaw = 0
        self.yaw_correction = 0.0

        self.global_path = None # waypoints from mission planner 
        self.orientation_grad = [None, None, None]

        # --- Home position ---
        self.local_home_pos = [0.0, 0.0] # local home position
        self.is_home_set = False
        self.delta_local_origin = [0.0, 0.0]
        self.origin_pose = None

        self.pose_local_origin = [None, None, None]
        self.gps_h_acc = None

        # --- State machine variables --- 
        self.current_state = State() # Current autopilot state
        self.prev_request = None
        self.prev_state = None
        self.current_status = "Init" # Drone state: Init, Arming, TakeOff, Hover, Landing, Grounded, Recording
        self.aim_point_id = None
        self.yaw_dataset = []

        # --- Path folower velocity ---
        self.path_counter = 0
        self.path_len = 0

        # --- Path follow overtake ---
        self.overtake_counter = 0
        self.overtake_len = 0

        self.is_update_yaw_once = False

        self.start_mission_once = False
        self.stop_mission_once = False
        
        # --- Publishers ArduPilot ---
        self.mission_publisher = rospy.Publisher('/mission_command', String, queue_size=10)

        self.setpoint_publisher_local = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) # set local position
        self.setpoint_publisher_global = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10) # set global position
        self.pub_setpoint_raw_att = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10) # for takeoff (set throttle)
        self.vel_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped , queue_size=1)
        self.sethome_publisher = rospy.Publisher('/mavros/home_position/set', HomePosition, queue_size=10) # set new home position
        self.telem_show = rospy.Publisher('/telemetry', String, queue_size=10) # set new home position
        self.demo_flight_publish = rospy.Publisher('/demo_flight', Bool, queue_size=1) # set new home position
        self.stop_mission_trigger = rospy.Publisher('/stop_mission_trigger', Bool, queue_size=1) # set new home position
        self.publish_gp_origin = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)
        self.publish_rc_overdrive = rospy.Publisher('/mavros/rc/overrid', OverrideRCIn, queue_size=10)
        self.publish_rc_in = rospy.Publisher('/mavros/rc/in', RCIn, queue_size=10)

        self.publish_rviz_drone1 = rospy.Publisher('drone1/rviz_local_pose', PoseStamped, queue_size=10)
        self.publish_id_path = rospy.Publisher('/id_path', String, queue_size=10)
        rospy.Subscriber('/drone2/id_path', String, self.id_path_callback)
        self.current_id_path_drone2 = None


        self.pub_local_pos_origin = rospy.Publisher('/local_pos_origin', PoseStamped, queue_size=10)

        self.set_rate_client = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        self.battery_voltage = None

        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_local_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.drone_pose_global_callback)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.drone_get_home_position)
        rospy.Subscriber('/mavros/global_position/local', Odometry, self.drone_orientation_callback)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.drone_relat_at)
        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_callback)
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_callback)

        # Wait for Flight Controller connection
        rospy.loginfo('Wait Flight Controller connection...')
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()
        rospy.loginfo("Flight Controller connection connected")

        self.set_rate_client(stream_id=0, message_rate=50, on_off=True)


    def set_gp_origin(self):
        point = GeoPointStamped()
        point.header.stamp = rospy.Time.now()
        point.position.latitude = self.pose_global.latitude
        point.position.longitude = self.pose_global.longitude
        point.position.altitude = self.pose_global.altitude
        self.publish_gp_origin.publish(point)        

    def get_local_vector_from_global(self, p1, p2):
        """
        This function transform global point to local

        Args:
            p1 [float, float]: geo pose [lat, lon]
            p2 [float, float]: origin geo cords [lat, lon]

        Returns:
            dp_0, dp_1 [float, float]: delta from p1 in meters relative to origin p2
        """
        R = 6378137.0
        dp_0 = (p1[0] - p2[0]) * (math.pi/ 180) * R
        dp_1 = (p1[1] - p2[1]) * (math.pi/ 180) * (R * math.cos(p1[0] * math.pi/180))
        return [dp_0, dp_1]

    def set_origin(self, geo_pose):
        """_summary_

        Args:
            geo_pose ([float, float]): [lat, lon] origin geo-position
        """        
        origin = geo_pose
        drone = [self.pose_global.latitude, self.pose_global.longitude]
  
        self.delta_local_origin = self.get_local_vector_from_global(drone, origin)

        self.delta_local_origin[0] = round(self.delta_local_origin[0], 2)
        self.delta_local_origin[1] = round(self.delta_local_origin[1], 2)
        
        print("delta_local_origin [dx, dy]:", self.delta_local_origin)

    ### --------------------------------
    ### ------ CALLBACK FUNCTIONs ------
    def id_path_callback(self, data):
        self.current_id_path_drone2 = int(data.data)
        
    def waypoint_callback(self, data):
        # rospy.loginfo("Got waypoint: %s", data)
        if len(data.waypoints) != 0:
            # rospy.loginfo("is_current: %s", data.waypoints[len(data.waypoints)-1].is_current)
            self.last_waypoint = data.waypoints[len(data.waypoints)-1].is_current	# Checks status of "is_current" for last waypoint

    def state_callback(self, state):    
        self.current_state = state
        self.set_rate_client(stream_id=0, message_rate=50, on_off=True)
        
    def drone_pose_local_callback(self, msg):
        # print(self.pose_local)
        self.pose_local = msg 
        euler = euler_from_quaternion([self.pose_local.pose.orientation.x, self.pose_local.pose.orientation.y, self.pose_local.pose.orientation.z, self.pose_local.pose.orientation.w])
        self.orientation_grad = [round(euler[0] * 180 / pi, 1), round(euler[1] * 180 / pi, 1), round(euler[2] * 180 / pi, 1)] #putch, roll, yaw   
        
    def drone_pose_global_callback(self, msg):
        self.pose_global = msg
    
    def drone_get_home_position(self, msg):
        self.pose_home = msg
        if self.pose_home != None:
            self.is_home_set = True
        # self.alt_ground = self.pose_global.altitude

    def drone_orientation_callback(self, msg):
        self.current_orientation = msg

    def drone_relat_at(self, msg):
        self.relative_altitude  = float(msg.data)

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage

    def velocity_callback(self, msg):
        self.velocity = msg

    def arm(self):
        """ Arms the motors of the quad, the rotors will spin slowly. TODO: check - The drone cannot takeoff until armed first 
        """       
        self.current_status = 'Arming'
        print('Arming')
    
        " wait for FCU connection "
        while not self.current_state.connected:
            print('Waiting for FCU connection...')
            self.rate.sleep()

        prev_request = rospy.get_time()
        prev_state = self.current_state
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "GUIDED" and (now - prev_request > 2.): #GUIDED
                self.set_mode_client(base_mode=0, custom_mode="GUIDED") #GUIDED
                prev_request = now 
            else:
                if not self.current_state.armed and (now - prev_request > 2.):
                   self.arming_client(True)
                   prev_request = now 

            if prev_state.armed != self.current_state.armed:
                print("Vehicle armed: %r" % self.current_state.armed)

            if prev_state.mode != self.current_state.mode: 
                print("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            if self.current_state.armed:
                self.current_status = "Armed"
                break

            self.rate.sleep()

    def dis_arm(self):
        """ Disarms the motors of the quad.
        """        
        print ("Disarming")
        rospy.wait_for_service(self.name + '/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy(self.name + '/mavros/cmd/arming', CommandBool)
            response = arming_cl(value = False)
            rospy.loginfo(response)
            
        except rospy.ServiceException as e:
            print("Disarming failed: %s" %e)

    def takeoff(self, height, yaw_drone=0): # OK
        """ Takeoff from the current location to the specified loacl altitude.

        Args:
            height (float): _description_
            yaw_drone (float, optional): _description_. Defaults to 0.
        """    
        if not self.current_state.armed:
            self.arm()

        print("Taking off")
        self.current_status = "TakeOff"
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_cl(altitude=height, latitude=0, longitude=0, min_pitch=0, yaw=yaw_drone*0.0174532925)
            rospy.loginfo(response)
            print('response', response)
        except rospy.ServiceException as e:
            print("Takeoff failed: %s" %e)

        while self.relative_altitude <= height - 0.5:
            self.current_status = "Taked off"
            pass
            # self.rate.sleep()      

    def land(self):
        """Land in the current position and orientation.
        """        
        rospy.loginfo("Landing")
        self.current_status = "Landing"
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            response = land_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Landing failed: %s" %e)

        while self.current_state.mode == "LAND" and self.current_state.armed:
            if self.current_state.armed == False:
                rospy.loginfo("Landed")
                self.current_status = "landed"
                break
            pass

    def set_mode(self, mode_ardupilot):
        """ Set FlightMode to Ardupilot

        Args:
            mode_ardupilot (String): FlightMode in string format
        """        
        rospy.loginfo("Setting Mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = change_mode(custom_mode=mode_ardupilot)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)

    def set_new_home_pos(self):
        """ Set current global position as a new home position """        
        test_home = []

        rospy.loginfo('Wait for local coords...')
        while self.pose_local == None: 
            pass

        self.local_home_pos = [self.pose_local.pose.position.x, self.pose_local.pose.position.y]
        print('local_home_pos ', self.local_home_pos)
        
        euler = euler_from_quaternion([self.pose_local.pose.orientation.x, self.pose_local.pose.orientation.y, self.pose_local.pose.orientation.z, self.pose_local.pose.orientation.w])
        self.orientation_grad = [round(euler[0] * 180 / pi, 1), round(euler[1] * 180 / pi, 1), round(euler[2] * 180 / pi, 1)]

        new_home_pose = []
        new_home_pose.append(True) #current_gps
        new_home_pose.append(self.orientation_grad[2]) #yaw
        new_home_pose.append(self.pose_global.latitude)
        new_home_pose.append(self.pose_global.longitude)
        new_home_pose.append(self.pose_global.altitude)
        self.set_home_client(new_home_pose[0], new_home_pose[1], new_home_pose[2], new_home_pose[3], new_home_pose[4])
    
    @staticmethod
    def get_setpoint(x, y, z, yaw=0.0): # early yaw=np.pi/2
        """ Set coordinates to PoseStamped() object

        Args:
            x (float): x coordinate
            y (float): y coordinate
            z (float): z coordinate
            yaw (float, optional): drone orientation. Defaults to 0.

        Returns:
            set_pose (PoseStamped): _description_
        """    
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        
        q = quaternion_from_euler(0, 0, yaw)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        return set_pose 
    
    @staticmethod
    def get_setpoint_global(lat, lon, alt, yaw=0.0):
        """ Set global coordinates to GeoPoseStamped() object

        Args:
            lat (float): latitude
            lon (float): longitude
            alt (float): altitude
            yaw (float, optional): drone orientation. Defaults to 0.

        Returns:
            set_pose_global(GeoPoseStamped): _description_
        """        
        set_pose_global = GeoPoseStamped()

        set_pose_global.header.stamp = rospy.Time.now()
        set_pose_global.header.frame_id = '1'

        set_pose_global.pose.position.latitude = lat
        set_pose_global.pose.position.longitude = lon
        set_pose_global.pose.position.altitude = alt

        q = quaternion_from_euler(0, 0, yaw)

        set_pose_global.pose.orientation.x = q[0]
        set_pose_global.pose.orientation.y = q[1]
        set_pose_global.pose.orientation.z = q[2]
        set_pose_global.pose.orientation.w = q[3]
        return set_pose_global

    def publish_setpoint(self, sp, yaw=0.0): #yaw=np.pi/2
        """ Publish setpoint by mavros to ardupilot 

        Args:
            sp (PoseStamped): Coordinates in PoseStamped()
            yaw (float, optional): drone orientation. Defaults to 0.
        """    
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw * pi/180)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher_local.publish(setpoint)


    def stop(self):
        """Stop ardupilot (need to be checked) """ 
        # TODO: check function       
        while self.current_state.armed or self.current_state.mode == "OFFBOARD":
            if self.current_state.armed:
                self.arming_client(False)
            if self.current_state.mode == "GUIDED_NOGPS":
                self.set_mode_client(base_mode=0, custom_mode="GUIDED_NOGPS")
            self.rate.spin()

    def pub_vel(self, velocity, yaw_rot=0.0):
        """ Publishes the goal pose to the /mavros/setpoint_velocity/cmd_vel topic to send the drone to the goal position.

        Args:
            velocity ([float, float, float]): drone speed [v_x, v_y, v_z]
        """        
        vel_cmd = TwistStamped()
        vel_cmd.header.frame_id = "map" #"base_link"
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.twist.linear.x = velocity[0] * 0.75
        vel_cmd.twist.linear.y = velocity[1] * 0.75
        vel_cmd.twist.linear.z = velocity[2] * 0.75
        vel_cmd.twist.angular.z = yaw_rot * 0.3
        self.vel_publisher.publish(vel_cmd)

    def vel_from_orinent(self, orient, th):

        pitch = orient[1]
        roll = orient[0]
        yaw_rot = orient[2] * 0.1

        speed = 1.0

        yaw_set = self.orientation_grad[2] - 90.0

        vx = math.sin(math.radians(roll)) * speed
        vy = math.sin(math.radians(pitch)) * speed

        if th is None:
            th = 0.0

        th_default = 0.4
        vz = round(th - th_default, 3)

        if abs(vz) < 0.035:
            vz = 0.0

        " rotate vector according to yaw of the drone "
        vx_r = vx * math.cos(yaw_set * math.pi / 180) - vy * math.sin(yaw_set * math.pi / 180)
        vy_r = vx * math.sin(yaw_set * math.pi / 180) + vy * math.cos(yaw_set * math.pi / 180)
        
        # print([round(vx,1), round(vy,1), round(vz,1)])

        # print("orient", orient[1], orient[0], self.yaw, vz)
        # print("th", th, vz)
        self.pub_vel([vx_r * 2.0, vy_r * 2.0, vz * 1.25], yaw_rot)

        

    def publish_setpoint_global(self, point, yaw):
        self.pose_global_target = GeoPoseStamped()
        self.pose_global_target.header.stamp = rospy.Time.now()
        self.pose_global_target.header.frame_id = str(0)

        self.pose_global_target.pose.position.latitude = point[0]
        self.pose_global_target.pose.position.longitude = point[1]
        self.pose_global_target.pose.position.altitude = point[2]

        quaternion = self.get_quaternion_from_euler(0, 0, yaw * 0.01745329252)

        self.pose_global_target.pose.orientation.x = quaternion[0]
        self.pose_global_target.pose.orientation.y = quaternion[1]
        self.pose_global_target.pose.orientation.z = quaternion[2]
        self.pose_global_target.pose.orientation.w = quaternion[3]

        self.setpoint_publisher_global.publish(self.pose_global_target)

    def get_bearing(lat1, long1, lat2, long2):
        """_summary_

        Args:
            lat1 (_type_): _description_
            long1 (_type_): _description_
            lat2 (_type_): _description_
            long2 (_type_): _description_

        Returns:
            _type_: _description_
        """        
        lat1, long1 = p1[0], p1[1] 
        lat2, long2 = p2[0], p2[1] 
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x,y)
        brng = numpy.degrees(brng)
        print(brng)
        return brng

    def goTo_global_test(self, aim_point_id, tol=0.01, speed_coef=0.0000025):
        pose_cord = np.array([self.pose_local.pose.position.x, self.pose_local.pose.position.y, self.pose_local.pose.position.z])

        lat_t = float(self.global_path[aim_point_id][8])
        long_t = float(self.global_path[aim_point_id][9])
        alt_t = float(self.global_path[aim_point_id][10])

        goal = np.array([lat_t, long_t, alt_t])
        
        current = np.array([self.pose_global.latitude, self.pose_global.longitude, self.pose_global.altitude])
        current[2] = alt_t
        print("Going to a waypoint...", goal, current)

        while norm(goal - current) > 0.00001:
            n = (goal - current) / norm(goal - current)
            current += speed_coef * n
            # current = self.pose_global
            # current[2] = alt_t

            print(norm(goal - current), current)
            self.publish_setpoint_global(current, 0)
            self.rate.sleep()

    def goTo_global(self, point, yaw):
        self.publish_setpoint_global(point, yaw)
        self.rate.sleep()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """Convert an Euler angle to a quaternion.

        Args:
            roll (float): The roll (rotation around x-axis) angle in radians.
            pitch (float): The pitch (rotation around y-axis) angle in radians.
            quaternion (float): The yaw (rotation around z-axis) angle in radians.

        Returns:
            quaternion([float,float,float,float]): The orientation in quaternion [x,y,z,w] format
        """        
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        quaternion = [qx, qy, qz, qw]

        return quaternion


    def shift_local_origin(self, point, delta):
        """_summary_

        Args:
            point (_type_): _description_
            delta (_type_): _description_

        Returns:
            point: _description_
        """        
        point[0] -= delta[1]
        point[1] -= delta[0]
        return point


    def goToVel_overtake(self, gp, tol=0.45, speed_coef=0.15):
        if self.overtake_counter < self.overtake_len:
            c_pose = np.array([self.pose_local.pose.position.x, self.pose_local.pose.position.y, self.pose_local.pose.position.z])
            g_pose = np.array([gp[0], gp[1], gp[2]])
            vector = [g_pose[0] - c_pose[0], g_pose[1] - c_pose[1], g_pose[2] - c_pose[2]]
            norm = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
            direction = [vector[0] / norm, vector[1] / norm, vector[2] / norm]
                
            " YAW "
            set_yaw = self.get_angle_two_vectors(direction) * 180.0 / math.pi
            cur_yaw = self.orientation_grad[2]
            dif_yaw = set_yaw - cur_yaw
            if abs(dif_yaw) > 180.0:
                dif_yaw += 360.0
            dif_yaw_drone = dif_yaw * 0.015

            self.pub_vel(direction, yaw_rot=dif_yaw_drone)

            if norm <= tol: 
                self.overtake_counter += 1


    def goToVel(self, gp, tol=0.4, speed_coef=0.15):
        c_pose = np.array([self.pose_local.pose.position.x, self.pose_local.pose.position.y, self.pose_local.pose.position.z])
        g_pose = np.array([gp[0], gp[1], gp[2]])
        vector = [g_pose[0] - c_pose[0], g_pose[1] - c_pose[1], g_pose[2] - c_pose[2]]
        norm = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
        direction = [vector[0] / norm, vector[1] / norm, vector[2] / norm]
            
        " YAW "
        set_yaw = self.get_angle_two_vectors(direction) * 180.0 / math.pi + self.yaw_correction
        cur_yaw = self.orientation_grad[2]
        dif_yaw = set_yaw - cur_yaw

        if abs(dif_yaw) > 180.0:
            dif_yaw += 360.0
        dif_yaw_drone = dif_yaw * 0.25

        self.pub_vel(direction, yaw_rot=dif_yaw_drone)

        if norm <= tol: 
            self.path_counter += 1


    def get_angle_two_vectors(self, vector):
        vector_1 = [vector[0], vector[1]]
        vector_2 = [0, -1]

        yaw = np.arctan2(vector[1], vector[0])
        angle = np.arctan2(vector_1[1], vector_1[0]) - np.arctan2(vector_2[1], vector_2[0])

        if 2*np.pi - angle <= angle:
            angle = 2*np.pi - angle
        
        return yaw


    def goTo(self, gp, mode='global', tol=0.3, speed_coef=0.25, yaw = 0.0):
        """ Local

        Args:
            gp (_type_): _description_
            mode (str, optional): _description_. Defaults to 'global'.
            tol (float, optional): _description_. Defaults to 0.25.
            speed_coef (float, optional): _description_. Defaults to 0.1.
            yaw (int, optional): _description_. Defaults to 0.
        """        
        pose_cord = np.array([self.pose_local.pose.position.x, self.pose_local.pose.position.y, self.pose_local.pose.position.z])

        goal = gp

        " Count local position and geo origin shift "

        goal = self.shift_local_origin(goal, self.delta_local_origin)

        if mode =='global':
            goal = gp
        elif mode =='relative':
            goal = pose_cord + gp
            
        print("Going to a waypoint...", goal)
        self.current_status = "Going_to_local"
        self.sp = pose_cord

        # yaw_way = self.angle_of_vectors(goal - pose_cord, [1, 0, 0]) #East is Y and 0 degree, N Nord

        while norm(goal - pose_cord) > tol:
            pose_cord = np.array([self.pose_local.pose.position.x, self.pose_local.pose.position.y, self.pose_local.pose.position.z])

            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += speed_coef * n
            # yaw_way = self.angle_of_vectors(goal - pose_cord, [1, 0, 0]) #East is Y and 0 degree, N Nord
            # self.publish_setpoint(self.sp, yaw_way)
            self.publish_setpoint(self.sp, yaw)
            self.rate.sleep() 

        if norm(goal - pose_cord) <= tol:
            self.current_status = "end flight"

         

    def cmd_yaw(self, yaw):
        # not works
        """Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-condition-yaw
        Args:
            yaw (_type_): _description_
        """        
        cmd = Waypoint()
        cmd.frame = 3
        cmd.command = 115
        cmd.is_current = True
        cmd.autocontinue = True
        cmd.param1 = yaw
        cmd.param2 = 10.0
        cmd.param3 = 0
        cmd.param4 = 0
        
        wl = WaypointList()
        wl.waypoints.append(cmd)

        try:
            service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
            print(service)
            service(start_index=0, waypoints=wl.waypoints)
            
            if service.call(0, wl.waypoints).success: #burasi belki list istiyordur. birde oyle dene
                rospy.logwarn('write mission success')
            else:
                rospy.logwarn('write mission error')
        
        except rospy.ServiceException:
            print("Service call failed: %s" % e)

        " start autonomus mission "
        self.set_mode("AUTO")

        # rospy.logwarn(self.current_state.mode)

        # while self.current_state.mode == "AUTO":
        #     pass



    def set_yaw_way(self, gp, mode='global'):
        pose_cord = np.array([self.pose_local.pose.position.x, self.pose_local.pose.position.y, self.pose_local.pose.position.z])
        goal = gp

        if mode =='global':
            goal = gp
        elif mode =='relative':
            goal = pose_cord + gp

        self.sp = pose_cord
        yaw_way = self.angle_of_vectors(goal - pose_cord, [1, 0, 0]) #East if 0 degree 

        q = quaternion_from_euler(0, 0, yaw_way)
        pose = AttitudeTarget()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]     

        self.pub_setpoint_raw_att.publish(pose)


    def angle_of_vectors(self, v1, v2):
        """ Calculate angle between two three-dimensional vectors

        Args:
            v1 ([float, float, float]]): vector1 [x, y, z]
            v2 ([float, float, float]): vector2 [x, y, z]

        Returns:
            angle (float): angle in degrees
        """        
        dotProduct = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
        modOfVector1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)*math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
        angle = dotProduct/modOfVector1
        # print("Cos theta =",angle)
        angleInDegree = math.degrees(math.acos(angle))
        print("theta =",angleInDegree," degr")
        return angleInDegree

    def waypoint_path(self, poses, start=True):
        """ Function to flight in AUTO mode by waypoint list from task list poses. 
        The function moves the drone from the start pose to the end pose without takeoff and landing.
        Use it after takeoff or in flight.

        Args:
            poses ([CPose, ...]): list of poses in CPose formate.
        """        
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            rospy.logwarn("cleaned")

        except rospy.ServiceException:
            print("Service call failed: %s" % e)
        
        wl = WaypointList()

        " Do change speed " #TODO: set speed by cmd. now seems not working
        # wp_sp = Waypoint()
        # wp_sp.frame = 3
        # wp_sp.is_current = False
        # wp_sp.autocontinue = True
        # wp_sp.command = 178
        # wp_sp.param1 = 10.0
        # wp_sp.param2 = 5.0 #m/s
        # wl.waypoints.append(wp_sp)

        for pose in poses:
            print(pose)
            wp = Waypoint()
            wp.frame = 3
            wp.command = 16
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = pose.lat
            wp.y_long = pose.lon
            wp.z_alt = pose.z_loc

            " add altetdute if drone goes down "
            # wp.z_alt += self.alt_ground

            wl.waypoints.append(wp)
            goal_pose = pose # last of the list

        """ UPDATED: Land """
        # wp = Waypoint()
        # wp.frame = 3
        # wp.command = 21
        # wp.autocontinue = True
        # wp.x_lat = goal_pose.lat
        # wp.y_long = goal_pose.lon
        # wl.waypoints.append(wp)
        """ end """ 

        try:
            service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
            service(start_index=0, waypoints=wl.waypoints)
            
            if service.call(0, wl.waypoints).success: #burasi belki list istiyordur. birde oyle dene
                rospy.logwarn('write mission success')
            else:
                rospy.logwarn('write mission error')
        
        except rospy.ServiceException:
            print("Service call failed: %s" % e)

        " start autonomus fight by waypoint list "
        if start:
            self.set_mode("AUTO")
            time.sleep(0.5)

            while self.current_state.mode == "AUTO":
                dist = self.calc_dists_global([goal_pose.lat, goal_pose.lon], [self.pose_global.latitude, self.pose_global.longitude])
                # print(dist)

                # TOTEST:
                # if self.last_waypoint: # Checks status of "is_current" for last waypoint
                #     rospy.sleep(2)
                #     self.set_mode("GUIDED")
                #     self.last_waypoint = False

                # end auto flight by min distance.
                if dist <= 1.5:
                    rospy.loginfo('Reached target pos')
                    rospy.sleep(3.0)
                    self.set_mode("GUIDED")
                    rospy.sleep(0.25)
                    land_pose = [goal_pose.lat, goal_pose.lon, goal_pose.alt]
                    goal_pose.w -= 90.0
                    print(goal_pose.w)
                    self.goTo_global(land_pose, goal_pose.w)
                    rospy.sleep(0.25)
                    rospy.loginfo('End, start land')
                pass

    def calc_dists_global(self, p1, p2):
        """Calculate distance between two geo positions 

        Args:
            p1 ([float, float]): geoposition 1 [lat1, lon1]
            p2 ([float, float]): geoposition 2 [lat2, lon2]

        Returns:
            distance [float]: distance in meters
        """    
        R = 6378137.0 # approximate radius of earth in meters
        lat1 = math.radians(p1[0])
        lon1 = math.radians(p1[1])
        lat2 = math.radians(p2[0])
        lon2 = math.radians(p2[1])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance