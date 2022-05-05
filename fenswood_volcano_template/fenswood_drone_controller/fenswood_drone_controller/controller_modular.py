"""
Very simple script-based sequencer using the old school examples
from https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber
"""
import rclpy                                                    # type: ignore
from rclpy.node import Node

# import message definitions for receiving status and position
from mavros_msgs.msg import State                               # type: ignore
from sensor_msgs.msg import NavSatFix                           # type: ignore
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

# import message definition for sending setpoint
from geographic_msgs.msg import GeoPoseStamped                  # type: ignore
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

# import service definitions for changing mode, arming, take-off and generic command
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong    # type: ignore

#libraries import
import geopandas as gpd
import pandas as pd
from shapely.geometry import point, Polygon
from haversine import haversine
import shapefile
import matplotlib
import mapclassify
import pyvisgraph

from .PathPlan2 import get_waypoints
from .PathPlan2 import create_polygon

#from turtle import delay
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('modular_controller')
        self.last_state = None     # global for last received status message
        self.last_pos = None       # global for last received position message
        self.init_alt = None       # global for global altitude at start
        self.last_alt_rel = None   # global for last altitude relative to start
        self.loc_last_pos=None
        self.last_twist=None

         # create service clients for long command (datastream requests)...
        self.cmd_cli = self.create_client(CommandLong, '/vehicle_1/mavros/cmd/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('command_int service not available, waiting again...')
        # ... for mode changes ...
        self.mode_cli = self.create_client(SetMode, '/vehicle_1/mavros/set_mode')
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        # ... for arming ...
        self.arm_cli = self.create_client(CommandBool, '/vehicle_1/mavros/cmd/arming')
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        # ... and for takeoff
        self.takeoff_cli = self.create_client(CommandTOL, '/vehicle_1/mavros/cmd/takeoff')
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')
        # create publisher for setpoint
        self.target_pub = self.create_publisher(GeoPoseStamped, '/vehicle_1/mavros/setpoint_position/global', 10)
        self.loc_target_pub = self.create_publisher(PoseStamped, '/vehicle_1/mavros/setpoint_position/local', 10)
        self.target_twist_pub = self.create_publisher(Twist, '/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped', 10) 

        # and make a placeholder for the last sent target
        self.last_target = GeoPoseStamped()
        self.last_loc_target = PoseStamped()
        self.last_twist=Twist() 

    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_state = msg
        self.get_logger().debug('Mode: {}.  Armed: {}.  System status: {}'.format(msg.mode,msg.armed,msg.system_status))

    # on receiving positon message, save it to global
    def position_callback(self,msg):
        # determine altitude relative to start
        if self.init_alt:
            self.last_alt_rel = msg.altitude - self.init_alt
        self.last_pos = msg
        self.get_logger().debug('Drone at {}N,{}E altitude {}m'.format(msg.latitude,
                                                                        msg.longitude,
                                                                        self.last_alt_rel))
    def local_pose_callback(self,msg):
        self.last_local_pos = msg
        self.get_logger().debug('Drone at x:{},y:{},z:{}'.format(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z))

    def twist_callback(self,msg):
        self.last_twist= msg
        self.get_logger().debug('Drone at z:{}'.format(msg.twist.angular.x))


    def wait_for_new_status(self):
        if self.last_state:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_state.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_state.header.stamp.sec > last_stamp:
                    break
        else:
            # if never had a message, just wait for first one          
            for try_wait in range(60):
                if self.last_state:
                    break
                rclpy.spin_once(self)

    def request_data_stream(self,msg_id,msg_interval):
        cmd_req = CommandLong.Request()
        cmd_req.command = 511
        cmd_req.param1 = float(msg_id)
        cmd_req.param2 = float(msg_interval)
        future = self.cmd_cli.call_async(cmd_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def change_mode(self,new_mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = new_mode
        future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future)    # wait for response

    def arm_request(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)

    def takeoff(self,target_alt):
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        future = self.takeoff_cli.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)

    def flyto(self,lat,lon,alt):
        self.last_target.pose.position.latitude = lat
        self.last_target.pose.position.longitude = lon
        self.last_target.pose.position.altitude = alt
        self.target_pub.publish(self.last_target)
        self.get_logger().info('Sent drone to {}N, {}E, altitude {}m'.format(lat,lon,alt)) 
    
    def yaw(self,yawz):
        self.last_twist.angular.z =yawz
        self.target_twist_pub.publish(self.last_twist)
        self.get_logger().info('Set drone to {}'.format(yawz)) 

    def run(self):
        # set up subscribers
        state_sub = self.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10)
        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)
        loc_pos_sub = self.create_subscription(PoseStamped, '/vehicle_1/mavros/local_position/pose', self.local_pose_callback, 10)
        twist_sub = self.create_subscription(TwistStamped, '/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped', self.twist_callback, 10)

        # see https://mavlink.io/en/messages/common.html#MAV_STATE
        for try_standby in range(60):
            self.wait_for_new_status()
            if self.last_state.system_status==3:
                self.get_logger().info('Drone ready for flight')
                break 

        # send command to request regular position updates
        self.request_data_stream(33, 1000000)
        self.request_data_stream(32, 1000000)
        self.request_data_stream(30, 1000000)##########################################################################################

        self.get_logger().info('Requested position stream')

        # now change mode to GUIDED
        self.change_mode("GUIDED")
        self.get_logger().info('Request sent for GUIDED mode.')
        
        # next, try to arm the drone
        # keep trying until arming detected in state message, or 60 attempts
        for try_arm in range(60):
            self.arm_request()
            self.get_logger().info('Arming request sent.')
            self.wait_for_new_status()
            if self.last_state.armed:
                self.get_logger().info('Arming successful')
                # armed - grab init alt for relative working
                if self.last_pos:
                    self.init_alt = self.last_pos.altitude
                break
        else:
            self.get_logger().error('Failed to arm')

        # take off and climb to 20.0m at current location
        self.takeoff(20.0)
        self.get_logger().info('Takeoff request sent.')

        # wait for drone to reach desired altitude, or 60 attempts
        for try_alt in range(60):
            self.wait_for_new_status()
            self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                break

#the coordinates
        # Takeoff#
        Take_Off = (-2.6714965, 51.4234019)

        # Flight Region
        FR_A = (-2.6717208, 51.4234260)
        FR_B = (-2.6701340, 51.4212462)
        FR_C = (-2.6656878, 51.4224401)
        FR_D = (-2.6670602, 51.4246918)

        # No Fly Zone
        NFZ_A = (-2.6720597, 51.4224669)
        NFZ_B = (-2.6673723, 51.4235482)
        NFZ_C = (-2.6670013, 51.4228490)
        NFZ_D = (-2.6714016, 51.4217450)
        No_Fly_Zone=(NFZ_A,NFZ_B,NFZ_C,NFZ_D)

        # Area of interest
        Area_of_Interest = (-2.6687700, 51.4219206)

        # Outside area of interest annulus
        Target1 = (-2.6684432, 51.4219778)

        #for overlap
        AB1=(-2.671951,51.423361)
        AB2=(-2.666637,51.424813)
        BC1=(-2.667174,51.425057)
        BC2=(-2.665585,51.422312)
        CD1=(-2.665397,51.422514)
        CD2=(-2.670492,51.421145)
        DA1=(-2.670029,51.421089)
        DA2=(-2.671802,51.423564)

        #points outside the flight boundary to create polygon
        AB=(-2.669641, 51.424326)
        BC=(-2.665659, 51.423752)
        CD=(-2.667447, 51.421697)
        DA=NFZ_A
        #create polygon for flight region boundary
        FR1=(AB1,AB2,AB)
        FR2=(BC1,BC2,BC)
        FR3=(FR_C,CD2,CD)
        FR4=(DA1,DA2,DA)

        # self.last_loc_target.pose.orientation.x = 0.5
        # self.last_loc_target.pose.position.y = 0.5
        # self.last_loc_target.pose.position.z = 20
        # self.loc_target_pub.publish(self.last_loc_target)
        # self.get_logger().info('Sent drone to x:{}, y:{}, z:{}'.format(0.5,0.5,0.5)) 

       # move drone by sending setpoint message
        waypoints = get_waypoints()
        for waypoint in waypoints:
            self.flyto(waypoint[0],waypoint[1],self.init_alt - 30.0)
            for try_arrive in range(60):
                self.wait_for_new_status()
                d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
                d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
                self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
                if abs(d_lon) < 0.0001 and abs(d_lat) < 0.0001:
                        self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                        break
        self.yaw(-1.0)
        for try_yaw in range(10):
            self.wait_for_new_status()
            self.get_logger().info('yaw {}'.format(self.last_twist.angular.z))

        self.flyto(51.4219104,-2.6687656,self.init_alt - 30.0)
        for try_arrive in range(60):
            self.wait_for_new_status()
            d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            if abs(d_lon) < 0.0001 and abs(d_lat) < 0.0001:
                self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                break
            
#RTL coordinates loop                    
        # return home and land
        self.get_logger().info('Request sent for RTL mode.')
        #self.self.get_logger().info('waypoints from path plan code,{}'.format(waypoints))
        RTLwaypoints=waypoints[::-1]
        for waypoint in RTLwaypoints:
        #for waypoint in waypoints:
            self.flyto(waypoint[0],waypoint[1],self.init_alt - 30.0)
            for try_arrive in range(60):
                self.wait_for_new_status()
                d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
                d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
                self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
                if abs(d_lon) < 0.0001 and abs(d_lat) < 0.0001:
                        self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                        break
        self.change_mode("RTL")
    
        # now just serve out the time until process killed
        while rclpy.ok():
            rclpy.spin_once(self)

def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()

if __name__ == '__main__':
    main()