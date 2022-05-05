# from turtle import delay

# from pytest import fail
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int64

class FailSafe(Node):

    def __init__(self):
        super().__init__('fail_safe')
        
        self.img_publisher_ = self.create_publisher(Image, 'status_frames', 10)
        self.md_publisher_ = self.create_publisher(String, 'failsafe_md', 10)
        self.md = String()
        self.md.data = "stand by"
        self.md_pub()
        self.br = CvBridge()
    
    def md_pub(self):
       self.md_timer_ = self.create_timer(10, self.md_timer_callback) 

    def md_timer_callback(self): 
        self.md_publisher_.publish(self.md)
        # self.get_logger().info('md: {}'.format(self.md.data))
    
    # def img_pub_(self):
          
    
    def start(self):
        # set up subscriber for image
        state_sub = self.create_subscription(String, '/failsafe_md', self.failsafe_callback, 10)

    def img_timer_callback(self):
        if self.md.data == 'stand by':
            self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/Standby.png", cv2.IMREAD_COLOR)))
        elif self.md.data == 'land':
            self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/land.png", cv2.IMREAD_COLOR)))
        elif self.md.data == 'hover':
            self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/hover.png", cv2.IMREAD_COLOR)))
        elif self.md.data == 'take off':
            self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/takeoff.png", cv2.IMREAD_COLOR)))
        elif self.md.data == 'rth':
            self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/rth.png", cv2.IMREAD_COLOR)))
        else:
            self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/error.png", cv2.IMREAD_COLOR)))
  

    def failsafe_callback(self,msg):
        if self.md.data != msg.data:
            self.md.data = msg.data
            self.md_pub()
        self.img_timer_ = self.create_timer(1, self.img_timer_callback) 
        # self.img_pub_()
        # if self.md.data == 'stand by':
        #     self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/Standby.png", cv2.IMREAD_COLOR)))
        # elif self.md.data == 'land':
        #     self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/land.png", cv2.IMREAD_COLOR)))
        # # elif self.md.data == 'hover':
        # #     self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/hover.png", cv2.IMREAD_COLOR)))
        # # elif self.md.data == 'take off':
        # #     self.img_publisher_.publish(self.br.cv2_to_imgmsg(cv2.imread("/ros_ws/src/fenswood_drone_controller/images/takeoff.png", cv2.IMREAD_COLOR)))
        # # delay(2)

    
    

def main(args=None):
    rclpy.init(args=args)

    fail_safe = FailSafe()
    fail_safe.start()
    # fail_safe.initial()
    # fail_safe.start()

    rclpy.spin(fail_safe)

    

if __name__ == '__main__':
    main()
  