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

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String


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
Area_of_Interest = (51.4219206,-2.6687700)

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

waypoints = get_waypoints()

class FenswoodDroneController(Node):

    def __init__(self):
        super().__init__('finite_state_controller')
        self.last_status = None     # global for last received status message
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

        # initial state for finite state machine
        self.control_state = 'init'
        # timer for time spent in each state
        self.state_timer = 0
        
    # on receiving status message, save it to global
    def state_callback(self,msg):
        self.last_status = msg
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
        if self.last_status:
            # if had a message before, wait for higher timestamp
            last_stamp = self.last_status.header.stamp.sec
            for try_wait in range(60):
                rclpy.spin_once(self)
                if self.last_status.header.stamp.sec > last_stamp:
                    break
        else:
            # if never had a message, just wait for first one          
            for try_wait in range(60):
                if self.last_status:
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


    def linear_move(self,linx):
        self.last_twist.linear.x =linx
        self.target_twist_pub.publish(self.last_twist)
        self.get_logger().info('Set drone to {}'.format(linx)) 

    def state_transition(self):
        if self.control_state =='init':
            if self.last_status.system_status==3:
                self.get_logger().info('Drone initialized')
                # send command to request regular position updates
                self.request_data_stream(33, 1000000)
                self.request_data_stream(32, 1000000)
                self.request_data_stream(30, 1000000)
                self.get_logger().info('Requested position stream')
                # change mode to GUIDED
                self.change_mode("GUIDED")
                self.get_logger().info('Request sent for GUIDED mode.')
                # move on to arming
                return('arming')
            else:
                return('init')

        elif self.control_state == 'arming':
            if self.last_status.armed:
                self.get_logger().info('Arming successful')
                if self.last_pos:
                    self.init_alt = self.last_pos.altitude
                self.takeoff(20.0)
                self.get_logger().info('Takeoff request sent.')
                return('climbing')
            elif self.state_timer > 60:                # timeout
                self.get_logger().error('Failed to arm')
                return('exit')
            else:
                self.arm_request()
                self.get_logger().info('Arming request sent.')
                return('arming')

        elif self.control_state == 'climbing':
            if self.last_alt_rel > 19.0:
                self.get_logger().info('Close enough to flight altitude')
                #self.flyto(waypoints[0][0], waypoints[0][1], self.init_alt - 30.0) # unexplained correction factor on altitude
                return('on_way1')
            elif self.state_timer > 60:                # timeout
                self.get_logger().error('Failed to reach altitude')
                return('RTH')
            else:
                self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
                return('climbing')

        elif self.control_state == 'on_way1':  
            # self.linear_move(10.0)    

            # self.flyto(51.4219206,-2.6687700,self.init_alt - 30.0)
            # for try_arrive in range(60):
            #     self.wait_for_new_status()
            #     d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            #     d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            #     self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            #     if abs(d_lon) < 0.0001 and abs(d_lat) < 0.0001:
            #         self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
            #         break

            # self.yaw(-0.8)

            fail=0
            for waypoint in waypoints:
                self.flyto(waypoint[0],waypoint[1],self.init_alt - 30.0)
                for try_arrive in range(60):
                    self.wait_for_new_status()
                    d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
                    d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
                    self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
                    if (abs(d_lon) < 0.00001) & (abs(d_lat) < 0.00001):
                        self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                        break
                    elif self.state_timer > 60:                        # timeout
                        self.get_logger().error('Failed to reach target')
                        fail+=1
                        break
                    else:
                        self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
            if fail==1:
                return('RTH')
            else:
                self.yaw(-0.8)
                for try_yaw in range(10):
                    self.wait_for_new_status()
                    self.get_logger().info('yaw {}'.format(self.last_twist.angular.z))
                return('on_way2')
                


        elif self.control_state == 'on_way2':
            self.get_logger().info('yaw {}'.format(self.last_twist.angular.z))
            self.flyto(Area_of_Interest[0],Area_of_Interest[1],self.init_alt - 30.0)
            d_lon = self.last_pos.longitude - self.last_target.pose.position.longitude
            d_lat = self.last_pos.latitude - self.last_target.pose.position.latitude
            if (abs(d_lon) < 0.00001) & (abs(d_lat) < 0.00001):
                self.get_logger().info('Close enough to target delta={},{}'.format(d_lat,d_lon))
                return('landing')
            elif self.state_timer > 60:     # timeout
                self.get_logger().error('Failed to reach target')
                return('RTH')
            else:
                self.get_logger().info('Target error {},{}'.format(d_lat,d_lon))
                return('on_way2')

        elif self.control_state == 'RTH':
            self.get_logger().info('Request sent for RTH mode.')
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
            return('exit')
            

        elif self.control_state == 'landing':
            fail2=0
            self.flyto(self.last_pos.latitude, self.last_pos.longitude,0.0) # unexplained correction factor on altitude
            for try_alt in range(60):
                self.wait_for_new_status()
                self.get_logger().info('landing, altitude {}m'.format(self.last_alt_rel))
                if self.last_alt_rel < 0.1:
                    self.get_logger().info('Close enough to ground')
                    break
                else:
                    self.get_logger().info('Climbing, altitude {}m'.format(self.last_alt_rel))
            return('RTH')


        elif self.control_state == 'exit':
            # nothing else to do
            return('exit')

    def run(self):

        # set up subscribers
        state_sub = self.create_subscription(State, '/vehicle_1/mavros/state', self.state_callback, 10)
        pos_sub = self.create_subscription(NavSatFix, '/vehicle_1/mavros/global_position/global', self.position_callback, 10)
        loc_pos_sub = self.create_subscription(PoseStamped, '/vehicle_1/mavros/local_position/pose', self.local_pose_callback, 10)
        twist_sub = self.create_subscription(TwistStamped, '/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped', self.twist_callback, 10)

        for try_loop in range(600):
            if rclpy.ok():
                self.wait_for_new_status()
                new_state = self.state_transition()
                if new_state == self.control_state:
                    self.state_timer = self.state_timer + 1
                else:
                    self.state_timer = 0
                self.control_state = new_state
                self.get_logger().info('Controller state: {} for {} steps'.format(self.control_state, self.state_timer))

def main(args=None):
    
    rclpy.init(args=args)

    controller_node = FenswoodDroneController()
    controller_node.run()


if __name__ == '__main__':
    main()