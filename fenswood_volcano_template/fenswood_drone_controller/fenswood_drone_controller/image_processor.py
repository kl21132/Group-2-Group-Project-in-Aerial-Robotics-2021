"""
Very simple image processor based on example from
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
"""
# from turtle import delay
import rclpy                                                    # type: ignore
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')
        
        self.img = None
    
        self.img_publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.red_publisher_ = self.create_publisher(String, 'red_zone_alert', 10)
        self.yellow_publisher_ = self.create_publisher(String, 'yellow_zone_alert', 10)
        
        # timer_period = 0.1  # seconds
        
        self.br = CvBridge()
       
    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        if (M['m00']!=0):
            cx= int(M['m10']/M['m00'])
            cy= int(M['m01']/M['m00'])
        return (cx, cy)
        
    def operations(self, mask):
        cp = []
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            cntre = self.get_contour_center(c)
            cp.append([area, cntre])
            # x,y,w,h = cv2.boundingRect(c)
        return cp

    def start(self):
        # set up subscriber for image
        state_sub = self.create_subscription(Image, '/vehicle_1/camera/image_raw', self.image_callback, 10)
        
    def img_pub_(self):
        self.img_timer_ = self.create_timer(0.05, self.timer_callback)

    def red_pub_(self):
        self.red_timer_ = self.create_timer(0.05, self.red_zone_timer_callback)

    def yellow_pub_(self):
        self.yellow_timer_ = self.create_timer(0.05, self.yellow_zone_timer_callback)

       
    def red_zone_timer_callback(self):
        red_zone = String()
        red_zone.data = self.r_alert
        self.red_publisher_.publish(red_zone)
    
    def yellow_zone_timer_callback(self):
        yellow_zone = String()
        yellow_zone.data = self.y_alert
        self.yellow_publisher_.publish(yellow_zone)

    def timer_callback(self): 
        
        # self.get_logger().info('Got an image of {} '.format(type(self.img)))
        # RGB_img = cv2.cvtColor(self.mask, cv2.COLOR_BGR2RGB)
        self.img_publisher_.publish(self.br.cv2_to_imgmsg(self.r_mask))
        # self.get_logger().info('Publishing video frame')
    
    # on receiving image, convert and log information
    def image_callback(self,msg):
        self.img = self.br.imgmsg_to_cv2(msg)
        self.hsv_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        b,g,r = cv2.split(self.img)

        lower_yellow = np.array([80,50,80]) 
        upper_yellow = np.array([90,255,255]) 

        lower_red = np.array([100,100,100]) 
        upper_red = np.array([180,255,255]) 

        # Threshold the HSV image using inRange function to get only red colors
        self.y_mask = cv2.inRange(self.hsv_image, upper_yellow , upper_yellow)
        self.r_mask = cv2.inRange(self.hsv_image, lower_red, upper_red)
        
        self.r_alert = '' #(where you should go [space] where you are) this is in string data type not a list
        self.y_alert = ''
        
        yellow_zones = self.operations(self.y_mask)
        red_zones = self.operations(self.r_mask)

        if yellow_zones != []:
            for zone in yellow_zones:
                if zone[0] >= 19200:
                    if zone[1][0]>160 and zone[1][0]<320:
                        self.y_alert = 'right -1'
                    elif zone[1][0]>320 and zone[1][0]<480:
                        self.y_alert = 'left 1'
                    elif zone[1][0]>0 and zone[1][0]<160:
                        self.y_alert = 'safe -2'
                    elif zone[1][0]>480 and zone[1][0]<640:
                        self.y_alert = 'safe 2'
        
        if red_zones != []:
            for zone in red_zones:
                if zone[0] >= 192:
                    if zone[1][0]>160 and zone[1][0]<320:
                        self.r_alert = 'right -1'
                    elif zone[1][0]>320 and zone[1][0]<480:
                        self.r_alert = 'left 1'
                    elif zone[1][0]>0 and zone[1][0]<160:
                        self.r_alert = 'safe  -2'
                    elif zone[1][0]>480 and zone[1][0]<640:
                        self.r_alert = 'safe 2'

        #include area and quadrant to know how close the obstacle is and what direction to take

        # can do OpenCV stuff on img now
        shp = self.img.shape # just get the size
        self.get_logger().info('red zone: {}'.format(self.r_alert))
        self.img_pub_()
        self.red_pub_()
        self.yellow_pub_()
                    

def main(args=None):
    
    rclpy.init(args=args)

    image_node = ImageProcessor()
    image_node.start()
    # delay((10))
    
    rclpy.spin(image_node)


if __name__ == '__main__':
    main()