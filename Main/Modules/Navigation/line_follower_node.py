""" This program publishes the radius and center of a colored blob  
    The radius wiil be zero if there is no detected object  

    published topics:  
        /processed_img [Image] 

    subscribed topics:  
        /camera   [Image]  
"""  

import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import Twist 
from collections import deque  

class LineFollowerNode(Node): 
    def __init__(self): 
        super().__init__('line_follower_node') 

        #Color Limits for black 
        self.colorLower = np.array([0, 0, 0]) 
        self.colorUpper = np.array([255, 255, 130]) 
        self.bridge = CvBridge() 
  
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10) # Fisico = video_source/raw , virtual = /camera
        self.pub = self.create_publisher(Image, '/processed_img', 10) 
        self.pub_error = self.create_publisher(Float32, '/error', 10)
        self.pub_zebra = self.create_publisher(String, '/zebra', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.image_received_flag = False #This flag is to ensure we received at least one image  
        self.pts = deque() 
        dt = 0.1 
        self.timer = self.create_timer(dt, self.timer_callback) 
        self.get_logger().info('ros_color_tracker Node started') 
        self.zebra = "false"  # Default value for zebra detection

    def camera_callback(self, msg): 
        try:  
            # We select bgr8 because its the OpenCV encoding by default  
            self.cv_img= self.bridge.imgmsg_to_cv2(msg, "bgr8")  

            scale_percent = 50 # percent of original size  
            self.width = int(self.cv_img.shape[1] * scale_percent / 100)  
            self.height = int(self.cv_img.shape[0] * scale_percent / 100)  
            dim = (self.width, self.height)  
            self.cv_img = cv2.resize(self.cv_img, dim, interpolation = cv2.INTER_AREA)  
            print(self.cv_img.shape)

            # Apply a ROI to the image
            # self.cv_img = self.cv_img[180:240, 0:320] # (y1:y2, x1:x2) Simulacion
            self.cv_img = self.cv_img[50:60, 0:80] #Crop the img to the bottom half SOLO PARA PUZZLEBOT

            self.image_received_flag = True  

        except: 
            self.get_logger().info('Failed to get an image') 

    def timer_callback(self): 
        #try:  
        if self.image_received_flag: 
            [cv_image, x, y, radius] = self.find_ball()  
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image,'bgr8')) 
            print("x: ", x) 
            print("y: ", y) 
            print("radius: ", radius)

            # Lógica de zebra
            if radius == 0.0:
                if self.zebra == "false":
                    self.zebra = "true"
                    # No se detecta línea
                    self.pub_zebra.publish(String(data=self.zebra))
                    stop_twist = Twist()
                    self.pub_cmd_vel.publish(stop_twist)
                    print("Zebra detectado, cambiando a modo odometría y deteniendo robot.")
            else:
                self.zebra = "false"
                self.pub_zebra.publish(String(data=self.zebra))

            # Calcular y publicar error solo si hay línea
            error = self.width/2.0 - float(x)
            error_msg = Float32()
            error_msg.data = error
            self.pub_error.publish(error_msg)

        #except: 
         #   self.get_logger().info('Failed to process image') 

    def find_ball(self):  

        """ Returns an image, the center(x,y) and radius of the detected ball 
            [cv_image, x, y, radius] = self.find_ball()  
            cv_image is an opencv image 
            (x,y)  is the center of the circle in [float pixels] 
            radius is the radius of the circle in [float pixels] 
        """ 

        # resize the cv_img, blur it, and convert it to the HSV color space  
        image = self.cv_img.copy() # Get a copy of the image to avoid changes while processing.   
         
        blurred = cv2.GaussianBlur(image, (11, 11), 0)  
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  

        # construct a mask for the color "RED", then perform  
        # a series of dilations and erosions to remove any small  
        # blobs left in the mask  
        mask = cv2.inRange(hsv, self.colorLower, self.colorUpper)  
        mask = cv2.erode(mask, None, iterations=2)  
        mask = cv2.dilate(mask, None, iterations=2)  

        # find contours in the mask and initialize the current  
        # (x, y) center of the ball  
        [cnts, hierarchy] = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  
        center = None  

        # only proceed if at least one contour was found  
        if len(cnts) > 0:  
            # find the largest contour in the mask, then use  
            # it to compute the minimum enclosing circle and  
            # centroid  
            c = max(cnts, key=cv2.contourArea)  
            ((x, y), radius) = cv2.minEnclosingCircle(c)  
            M = cv2.moments(c)  
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  

            # only proceed if the radius meets a minimum size  
            if radius > 5: #5 pixels 
                # Draw the circle and centroid on the cv_img. 
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
                cv2.circle(image, center, 5, (0, 0, 255), -1)  
            else: #If the detected object is too small 
                radius = 0.0  #Just set the radius of the object to zero 

        else:  
            # All the values will be zero if there is no object  
            x = 0.0 
            y = 0.0 
            radius=0.0 
        # Returns the opencv image 
        return [image, x, y, radius]  
     
def main(args=None): 
    rclpy.init(args=args) 
    cv_e = LineFollowerNode() 
    rclpy.spin(cv_e) 
    cv_e.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 