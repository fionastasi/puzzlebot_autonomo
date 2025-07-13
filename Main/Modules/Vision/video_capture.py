"""
This node will be used to record the video stream from the camera
and save it to a file.
"""
import rclpy 
from rclpy.node import Node 
import cv2 
import numpy as np 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 

class VideoCapture(Node):
    def __init__(self):
        super().__init__('video_capture_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        self.pub = self.create_publisher(Image, 'processed_img', 10)


        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.rec = cv2.VideoWriter('/home/danieldrg/Python/Videos/Verde.avi', self.fourcc, 15, (160, 120))

        self.record = False
        self.image_received_flag = False
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info("Vision node started")


    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().info("Failed to get an image")

    def timer_callback(self):
         if self.image_received_flag:
            self.image_received_flag = False
            #print("Size: ", self.cv_img.shape)
            self.cv_img = self.cv_img[0:120, 0:160] 
            cv2.imshow("Video en vivo", self.cv_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                print("Recording initialized")
                self.record = True
            elif key == ord('q'):
                print("Recording stopped")
                self.rec.release()
            else:
                cv2.waitKey(1)
            
            if self.record:
                self.rec.write(self.cv_img)

            
            self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_img, 'bgr8')) 



def main(args=None):
        rclpy.init(args=args)
        node = VideoCapture()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()